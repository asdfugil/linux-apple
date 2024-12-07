// SPDX-License-Identifier: GPL-2.0
/*
 * BQ27xxx battery monitor UART driver
 *
 * Copyright (C) 2020 Corellium LLC
 * Copyright (C) 2024, Nick Chan <towinchenmi@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/unaligned.h>
#include <linux/power/bq27xxx_battery.h>

#define DEBUG 1

#define BQ27540_BIT_SET 0xfe
#define BQ27540_BIT_UNSET 0xc0

#define MSG_MAX 64

#define TIMEOUT_MSEC 500

struct bq27540_hdquart {
	struct serdev_device *serdev;
	struct power_supply *batt;
	struct delayed_work work;
	struct completion done;
	spinlock_t lock;
	struct mutex mtx;
	unsigned first;

	unsigned nrxbit, irxbit;
	uint8_t rxbuf[MSG_MAX];
	uint8_t txstart[8];

	int s_t, s_v, s_f, s_cr, s_cf, s_ia, s_cy;
	unsigned good_data;
};

/**
 * bq27540_data_to_cmdbuf() - Convert data to command buffer
 * 
 * @data: Input data buffer
 * @cmdbuf: Command buffer, must be 8 times the size of data buffer
 * @data_size: Size of the data buffer
 */
static void bq27540_data_to_cmdbuf(u8 *data, u8 *cmdbuf, size_t data_size)
{
	for (u8 i = 0; i < data_size; i++)
		for (u8 j = 0; j < 8; j++)
			cmdbuf[i * 8 + j] = ((data[i] >> j) & 1u) ?
						    BQ27540_BIT_SET :
						    BQ27540_BIT_UNSET;
}

static int bq27545_hdquart_xfer(struct bq27xxx_device_info *di, uint8_t *txdata,
				size_t txsize, uint8_t *rxdata, size_t rxsize)
{
	struct bq27540_hdquart *bbq = dev_get_drvdata(di->dev);
	unsigned long timeout = msecs_to_jiffies(TIMEOUT_MSEC), flags;
	unsigned i, j;
	unsigned char buf[8];

	if (txsize + rxsize > MSG_MAX)
		return -EFBIG;

	serdev_device_break_ctl(bbq->serdev, true);
	usleep_range(250, 500);
	serdev_device_break_ctl(bbq->serdev, false);
	usleep_range(150, 500);

	reinit_completion(&bbq->done);
	spin_lock_irqsave(&bbq->lock, flags);
	bbq->irxbit = 0;
	bbq->nrxbit = (txsize + rxsize) * 8;
	spin_unlock_irqrestore(&bbq->lock, flags);

	for (i = 0; i < txsize; i++) {
		for (j = 0; j < 8; j++)
			buf[j] = ((txdata[i] >> j) & 1u) ? 0xFE : 0xC0;
		if (i == 0)
			memcpy(bbq->txstart, buf, 8);
		if (serdev_device_write(bbq->serdev, buf, sizeof(buf), HZ) !=
		    sizeof(buf)) {
			dev_err(&bbq->serdev->dev, "HDQ transmit failed\n");
			return -EINVAL;
		}
	}

	serdev_device_wait_until_sent(bbq->serdev, HZ);

	timeout = wait_for_completion_timeout(&bbq->done, timeout);
	if (timeout == 0) {
		dev_err(&bbq->serdev->dev, "HDQ receive timed out [%02x]\n",
			txdata[0]);
		return -ETIMEDOUT;
	}

	if (rxdata) {
		spin_lock_irqsave(&bbq->lock, flags);
		memcpy(rxdata, bbq->rxbuf + txsize, rxsize);
		spin_unlock_irqrestore(&bbq->lock, flags);
	}

	return 0;
}

static int bq27545_hdquart_read_word(struct bq27xxx_device_info *di, uint8_t key,
				     int *result)
{
	int ret;
	uint8_t h0, l0, h1, l1, key2 = key + 1;

	/* this is TI's recommended read process for 16-bit registers */
	ret = bq27545_hdquart_xfer(di, &key2, 1, &h0, 1);
	if (ret)
		return ret;
	ret = bq27545_hdquart_xfer(di, &key, 1, &l0, 1);
	if (ret)
		return ret;
	ret = bq27545_hdquart_xfer(di, &key2, 1, &h1, 1);
	if (ret)
		return ret;
	if (h0 == h1) {
		*result = (int16_t)(l0 | ((unsigned)h0 << 8));
		return 0;
	}
	ret = bq27545_hdquart_xfer(di, &key, 1, &l1, 1);
	if (ret)
		return ret;
	*result = (int16_t)(l1 | ((unsigned)h1 << 8));
	return 0;
}

static int bq27xxx_battery_uart_read(struct bq27xxx_device_info *di, u8 reg,
				     bool single)
{
	size_t xfer_sz = single ? 1 : 2;
	uint8_t rxdata;
	int result;

	if (single) {
		dev_info(di->dev, "single xfer\n");
		int ret = bq27545_hdquart_xfer(di, &reg, 1, &rxdata, xfer_sz);
		if (ret) return ret;
		return rxdata;
	}
	else {
		dev_info(di->dev, "16-bit xfer\n");
		int ret = bq27545_hdquart_read_word(di, reg, &result);
		if (ret) return ret;
		return result;
	}
}

static int bq27xxx_battery_uart_bulk_read(struct bq27xxx_device_info *di,
					  u8 reg, u8 *data, int len)
{
	return bq27545_hdquart_xfer(di, &reg, 1, data, len);
}

/*
static int bq27xxx_battery_uart_write(struct bq27xxx_device_info *di, u8 reg,
				      int value, bool single)
{
	u8 data[4];
	u8 cmd_buf[32];
	ssize_t ret;
	size_t len;
	unsigned long flags;
	struct bq27540_hdquart *bbq = dev_get_drvdata(di->dev);

	data[0] = reg;
	if (single) {
		data[1] = (u8)value;
		len = 2;
	} else {
		put_unaligned_le16(value, &data[1]);
		len = 3;
	}

	serdev_device_break_ctl(bbq->serdev, true);
	usleep_range(250, 500);
	serdev_device_break_ctl(bbq->serdev, false);
	usleep_range(150, 500);

	bq27540_data_to_cmdbuf(data, cmd_buf, len);

	spin_lock_irqsave(&bbq->lock, flags);

	ret = serdev_device_write(bbq->serdev, cmd_buf, len * 8, HZ);

	spin_unlock_irqrestore(&bbq->lock, flags);

	if (ret != len * 8) {
		dev_err(&bbq->serdev->dev, "%s: UART transmit failed\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

static int bq27xxx_battery_uart_bulk_write(struct bq27xxx_device_info *di,
					   u8 reg, u8 *data, int len)
{
	u8 buf[33];
	u8 *cmd_buf;
	ssize_t ret;
	unsigned long flags;
	struct bq27540_hdquart *bbq = dev_get_drvdata(di->dev);

	len += 1;

	cmd_buf = devm_kzalloc(di->dev, len * 8, GFP_KERNEL);
	if (!cmd_buf)
		return -ENOMEM;

	buf[0] = reg;
	memcpy(&buf[1], data, len);

	bq27540_data_to_cmdbuf(buf, cmd_buf, len);

	serdev_device_break_ctl(bbq->serdev, true);
	usleep_range(250, 500);
	serdev_device_break_ctl(bbq->serdev, false);
	usleep_range(150, 500);

	spin_lock_irqsave(&bbq->lock, flags);

	ret = serdev_device_write(bbq->serdev, cmd_buf, len * 8, HZ);

	spin_unlock_irqrestore(&bbq->lock, flags);

	devm_kfree(di->dev, cmd_buf);

	if (ret != len * 8) {
		dev_err(&bbq->serdev->dev, "%s: UART transmit failed\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}*/

static size_t bq27540_hdquart_receive_buf(struct serdev_device *serdev,
					  const unsigned char *buf, size_t size)
{
	struct device *dev = &serdev->dev;
	struct bq27540_hdquart *bbq = dev_get_drvdata(dev);
	unsigned i, j;
	unsigned long flags;

	dev_info(dev, "received data!\n");

	spin_lock_irqsave(&bbq->lock, flags);
	for (i = 0; i < size; i++) {
		j = bbq->irxbit;
		/* The controller should always echo the command first */
		if (j < 8 && buf[i] != bbq->txstart[j])
			continue;
		if (j < bbq->nrxbit) {
			if ((j & 7) == 0)
				bbq->rxbuf[j >> 3] = 0;
			if (buf[i] >= 0xF0)
				bbq->rxbuf[j >> 3] |= 1u << (j & 7);
			bbq->irxbit++;
			if (bbq->irxbit >= bbq->nrxbit)
				complete(&bbq->done);
		}
	}
	spin_unlock_irqrestore(&bbq->lock, flags);

	return size;
}

static const struct serdev_device_ops bq27540_hdquart_serdev_device_ops = {
	.receive_buf = bq27540_hdquart_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static int bq27xxx_battery_uart_probe(struct serdev_device *serdev)
{
	int ret = 0;
	struct device *dev = &serdev->dev;

	struct bq27xxx_device_info *di;
	struct bq27540_hdquart *bbq;

	di = devm_kzalloc(dev, sizeof(*di), GFP_KERNEL);

	if (!di)
		return -ENOMEM;

	bbq = devm_kzalloc(dev, sizeof(*bbq), GFP_KERNEL);

	if (!bbq)
		return -ENOMEM;

	bbq->serdev = serdev;
	dev_set_drvdata(dev, bbq);

	mutex_init(&bbq->mtx);
	spin_lock_init(&bbq->lock);
	init_completion(&bbq->done);

	serdev_device_set_client_ops(serdev,
				     &bq27540_hdquart_serdev_device_ops);
	ret = devm_serdev_device_open(dev, serdev);

	if (ret)
		return ret;

	serdev_device_set_baudrate(serdev, 57600);
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		return ret;

	ret = serdev_device_stopbit_ctl(serdev, true);
	if (ret)
		return ret;

	di->dev = &serdev->dev;
	di->chip = BQ27540;
	di->name = "bq27540-battery";
	di->bus.read = bq27xxx_battery_uart_read;
	//di->bus.write = bq27xxx_battery_uart_write;
	//di->bus.read_bulk = bq27xxx_battery_uart_bulk_read;
	//di->bus.write_bulk = bq27xxx_battery_uart_bulk_write;

	return bq27xxx_battery_setup(di);
}

static void bq27xxx_battery_uart_remove(struct serdev_device *serdev)
{
	return;
}

#ifdef CONFIG_OF
static const struct of_device_id bq27xxx_battery_uart_of_match_table[] = {
	{ .compatible = "ti,bq27540" },
	{},
};
MODULE_DEVICE_TABLE(of, bq27xxx_battery_uart_of_match_table);
#endif

static struct serdev_device_driver bq27xxx_battery_uart_driver = {
	.probe = bq27xxx_battery_uart_probe,
	.remove = bq27xxx_battery_uart_remove,
	.driver = {
		.name = "bq27xxx_battery_uart",
		.of_match_table = bq27xxx_battery_uart_of_match_table,
	},
};

module_serdev_device_driver(bq27xxx_battery_uart_driver);

MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
MODULE_DESCRIPTION("BQ27xxx battery monitor UART driver");
MODULE_LICENSE("GPL");
