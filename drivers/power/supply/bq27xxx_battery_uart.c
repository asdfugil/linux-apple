// SPDX-License-Identifier: GPL-2.0
/*
 * BQ27xxx battery monitor UART driver
 *
 * Copyright (C) 2024, Nick Chan <towinchenmi@gmail.com>
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/power/bq27xxx_battery.h>

#define BQ27540_BIT_SET 0xfe
#define BQ27540_BIT_UNSET 0xc0

#define MSG_MAX 64

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

static int bq27xxx_battery_uart_read(struct bq27xxx_device_info *di, u8 reg,
				     bool single)
{
	u8 reg_buf[8];

	bq27540_data_to_cmdbuf(&reg, &reg_buf, 1);

	if (serdev_device_write(di->dev, reg_buf, 1, HZ) != 8) {
		dev_err(&di->dev, "UART transmit failed\n");
		return -EINVAL;
	}
}

static int bq27xxx_battery_uart_bulk_read(struct bq27xxx_device_info *di,
					  u8 reg, u8 *data, int len)
{
}

static int bq27xxx_battery_uart_write(struct bq27xxx_device_info *di, u8 reg,
				      int value, bool single)
{
	u8 data[4];
	u8 cmd_buf[32];
	int ret;
	size_t len;

	data[0] = reg;
	if (single) {
		data[1] = (u8)value;
		len = 2;
	} else {
		put_unaligned_le16(value, &data[1]);
		len = 3;
	}

	bq27540_data_to_cmdbuf(&data, &cmd_buf, len);

	if (serdev_device_write(di->dev, cmd_buf, len * 8, HZ) != len * 8) {
		dev_err(&di->dev, "UART transmit failed\n");
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

	len += 1;

	cmd_buf = devm_kzalloc(&di->dev, len * 8, GFP_KERNEL);
	if (!cmd_buf)
		return -ENOMEM;

	buf[0] = reg;
	memcpy(&buf[1], data, len);

	bq27540_data_to_cmdbuf(&buf, &cmd_buf, len);

	ret = serdev_device_write(di->dev, &cmd_buf, len * 8, HZ);

	devm_kfree(di->dev, cmd_buf);

	if (ret != len) {
		dev_err(&di->dev, "UART transmit failed\n");
		return -EINVAL;
	}

	return 0;
}

static int bq27540_hdquart_receive_buf(struct serdev_device *serdev,
				       const unsigned char *buf, size_t size)
{
	return 0;
}

module_serdev_device_driver(bq27xxx_battery_uart_driver);

#ifdef CONFIG_OF
static const struct of_device_id bq27xxx_battery_uart_of_match_table[] = {
	{ .compatible = "ti,bq27540" },
	{},
};
MODULE_DEVICE_TABLE(of, bq27xxx_battery_uart_of_match_table);
#endif

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
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);

	di->dev = &serdev->dev;
	di->chip = BQ27540;
	di->name = "bq27540-battery";
	di->bus.read = bq27xxx_battery_uart_read;
	di->bus.write = bq27xxx_battery_uart_write;
	di->bus.read_bulk = bq27xxx_battery_uart_bulk_read;
	di->bus.write_bulk = bq27xxx_battery_uart_bulk_write;

	return bq27xxx_battery_setup(di);
}

static void bq27xxx_battery_uart_remove(struct serdev_device *serdev)
{
	return;
}

static const struct of_device_id bq27xxx_battery_uart_of_match[] = {
	{
		.compatible = "ti,bq27540",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bq27xxx_battery_uart_of_match);

static struct serdev_device_driver bq27xxx_battery_uart_driver = {
	.probe = bq27xxx_battery_uart_probe,
	.remove = bq27xxx_battery_uart_remove,
	.driver = {
		.name = "bq27xxx_battery_uart",
		.of_match_table = bq27xxx_battery_uart_of_match,
	},
};

module_serdev_device_driver(bq27xxx_battery_uart_driver);

MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
MODULE_DESCRIPTION("BQ27xxx battery monitor UART driver");
MODULE_LICENSE("GPL");
