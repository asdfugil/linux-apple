/*
 * Driver for bq27545 gas gauges on mobile devices with Hx chips.
 *
 * Copyright (C) 2020 Corellium LLC
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/sched.h>
#include <linux/serdev.h>
#include <linux/power_supply.h>
#include <linux/completion.h>
#include <linux/i2c.h>

#define POLL_INIT_MSEC 1500
#define POLL_MSEC 2000
#define TIMEOUT_MSEC 500
#define MSG_MAX 64

#define GG_Temperature 0x06
#define GG_Voltage 0x08
#define GG_Flags 0x0a
#define GG_RemainingCapacity 0x10
#define GG_FullChargeCapacity 0x12
#define GG_AverageCurrent 0x14
#define GG_CycleCount 0x2a

#define GG_Flags_OTC 0x8000
#define GG_Flags_OTD 0x4000
#define GG_Flags_BATHI 0x2000
#define GG_Flags_BATLOW 0x1000
#define GG_Flags_CHGINH 0x0800
#define GG_Flags_FC 0x0200
#define GG_Flags_CHG 0x0100
#define GG_Flags_OCVTAKEN 0x0080
#define GG_Flags_ISD 0x0040
#define GG_Flags_TDD 0x0020
#define GG_Flags_REV 0x0010
#define GG_Flags_SOC1 0x0004
#define GG_Flags_SOCF 0x0002
#define GG_Flags_DSG 0x0001

/* those are in sn2400-charger.c, which acts as a HDQ mux */
int sn2400_charger_register(struct device *charger, struct device *client);
int sn2400_charger_hdq_mux(struct device *charger, struct device *client,
			   unsigned route);

struct bq27545_hdquart {
	struct serdev_device *serdev;
	struct device *charger;
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

static int bq27545_hdquart_xfer(struct bq27545_hdquart *bbq, uint8_t *txdata,
				size_t txsize, uint8_t *rxdata, size_t rxsize);
static int bq27545_hdquart_read_word(struct bq27545_hdquart *bbq, uint8_t key,
				     int *result);

static void bq27545_hdquart_delayed_func(struct work_struct *_work)
{
	struct bq27545_hdquart *bbq =
		container_of(_work, struct bq27545_hdquart, work.work);
	int ret;

	ret = sn2400_charger_hdq_mux(bbq->charger, &bbq->serdev->dev, 1);
	if (ret)
		goto exit_early;

	msleep(10);

	mutex_lock(&bbq->mtx);

	ret = bq27545_hdquart_read_word(bbq, GG_Temperature, &bbq->s_t);
	ret |= bq27545_hdquart_read_word(bbq, GG_Voltage, &bbq->s_v);
	ret |= bq27545_hdquart_read_word(bbq, GG_Flags, &bbq->s_f);
	ret |= bq27545_hdquart_read_word(bbq, GG_RemainingCapacity, &bbq->s_cr);
	ret |= bq27545_hdquart_read_word(bbq, GG_FullChargeCapacity,
					 &bbq->s_cf);
	ret |= bq27545_hdquart_read_word(bbq, GG_AverageCurrent, &bbq->s_ia);
	ret |= bq27545_hdquart_read_word(bbq, GG_CycleCount, &bbq->s_cy);

	mutex_unlock(&bbq->mtx);

	if (bbq->first && !ret)
		bbq->first = 0;

	if (!ret)
		bbq->good_data = 3;
	else if (bbq->good_data)
		bbq->good_data--;

	sn2400_charger_hdq_mux(bbq->charger, &bbq->serdev->dev, 0);
exit_early:
	schedule_delayed_work(&bbq->work,
			      msecs_to_jiffies(bbq->first ? POLL_INIT_MSEC :
							    POLL_MSEC));
}

static int bq27545_hdquart_read_word(struct bq27545_hdquart *bbq, uint8_t key,
				     int *result)
{
	int ret;
	uint8_t h0, l0, h1, l1, key2 = key + 1;

	/* this is TI's recommended read process for 16-bit registers */
	ret = bq27545_hdquart_xfer(bbq, &key2, 1, &h0, 1);
	if (ret)
		return ret;
	ret = bq27545_hdquart_xfer(bbq, &key, 1, &l0, 1);
	if (ret)
		return ret;
	ret = bq27545_hdquart_xfer(bbq, &key2, 1, &h1, 1);
	if (ret)
		return ret;
	if (h0 == h1) {
		*result = (int16_t)(l0 | ((unsigned)h0 << 8));
		return 0;
	}
	ret = bq27545_hdquart_xfer(bbq, &key, 1, &l1, 1);
	if (ret)
		return ret;
	*result = (int16_t)(l1 | ((unsigned)h1 << 8));
	return 0;
}

static int bq27545_hdquart_xfer(struct bq27545_hdquart *bbq, uint8_t *txdata,
				size_t txsize, uint8_t *rxdata, size_t rxsize)
{
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

static size_t bq27545_hdquart_receive_buf(struct serdev_device *serdev,
				       const unsigned char *buf, size_t size)
{
	struct device *dev = &serdev->dev;
	struct bq27545_hdquart *bbq = dev_get_drvdata(dev);
	unsigned i, j;
	unsigned long flags;

	spin_lock_irqsave(&bbq->lock, flags);
	for (i = 0; i < size; i++) {
		j = bbq->irxbit;
		if (j < 8 && buf[i] != bbq->txstart[j]) /* echo must matching */
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

static const struct serdev_device_ops bq27545_hdquart_serdev_device_ops = {
	.receive_buf = bq27545_hdquart_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static enum power_supply_property bq27545_hdquart_battery_prop[] = {
	POWER_SUPPLY_PROP_STATUS,      POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,      POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL, POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,    POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CYCLE_COUNT
};

static int bq27545_hdquart_battery_get_property(struct power_supply *psy,
						enum power_supply_property psp,
						union power_supply_propval *val)
{
	struct bq27545_hdquart *bbq = dev_get_drvdata(psy->dev.parent);

	mutex_lock(&bbq->mtx);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if ((bbq->s_f & GG_Flags_CHG) || (bbq->s_ia > 0))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if ((bbq->s_f & GG_Flags_DSG) || (bbq->s_ia < 0))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (bbq->s_cr * 1000 >= bbq->s_cf * 980) /* 98% = full */
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bbq->good_data ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (bbq->s_f & (GG_Flags_OTC | GG_Flags_OTD))
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (bbq->s_f & GG_Flags_BATLOW)
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bbq->s_v * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bbq->s_ia * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bbq->s_cf * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = bbq->s_cr * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (bbq->s_cf) {
			val->intval =
				(bbq->s_cr * 100 + bbq->s_cf / 2) / bbq->s_cf;
			if (val->intval < 0)
				val->intval = 0;
			if (val->intval > 100)
				val->intval = 100;
		} else
			val->intval = 99;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bbq->s_t - 2732; /* 0.1 K -> 0.1 'C */
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = bbq->s_cy;
		break;
	default:
		mutex_unlock(&bbq->mtx);
		return -EINVAL;
	}
	mutex_unlock(&bbq->mtx);
	return 0;
}

static const struct power_supply_desc bq27545_hdquart_batt_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = bq27545_hdquart_battery_prop,
	.num_properties = ARRAY_SIZE(bq27545_hdquart_battery_prop),
	.get_property = bq27545_hdquart_battery_get_property,
};

static int bq27545_hdquart_register_batt(struct bq27545_hdquart *bbq)
{
	bbq->batt = power_supply_register(&bbq->serdev->dev,
					  &bq27545_hdquart_batt_desc, NULL);
	if (IS_ERR(bbq->batt))
		return PTR_ERR(bbq->batt);
	return 0;
}

static int bq27545_hdquart_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct bq27545_hdquart *bbq;
	struct device_node *of_charger;
	int ret;

	bbq = devm_kzalloc(dev, sizeof(*bbq), GFP_KERNEL);
	if (!bbq)
		return -ENOMEM;

	of_charger = of_parse_phandle(dev->of_node, "charger", 0);
	if (of_charger) {
		bbq->charger =
			bus_find_device_by_of_node(&i2c_bus_type, of_charger);
		if (!bbq->charger)
			return -EPROBE_DEFER;
		ret = sn2400_charger_register(bbq->charger, dev);
		if (ret)
			return ret;
	}

	bbq->serdev = serdev;
	dev_set_drvdata(dev, bbq);

	mutex_init(&bbq->mtx);
	spin_lock_init(&bbq->lock);
	init_completion(&bbq->done);

	serdev_device_set_client_ops(serdev,
				     &bq27545_hdquart_serdev_device_ops);
	ret = devm_serdev_device_open(dev, serdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(serdev, 57600);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	serdev_device_stopbit_ctl(serdev, true);

	ret = bq27545_hdquart_register_batt(bbq);
	if (ret) {
		dev_err(dev, "battery register err: %d.", ret);
		return ret;
	}

	ret = devm_of_platform_populate(dev);
	if (ret)
		return ret;

	bbq->first = 1;

	INIT_DELAYED_WORK(&bbq->work, bq27545_hdquart_delayed_func);
	schedule_delayed_work(&bbq->work, msecs_to_jiffies(POLL_INIT_MSEC));

	return 0;
}

static const struct of_device_id bq27545_hdquart_dt_ids[] = {
	{ .compatible = "ti,bq27545-hdquart" },
	{}
};
MODULE_DEVICE_TABLE(of, bq27545_hdquart_dt_ids);

static struct serdev_device_driver bq27545_hdquart_drv = {
    .probe = bq27545_hdquart_probe,
    .driver = {
        .name = "bq27545-hdquart",
        .of_match_table	= bq27545_hdquart_dt_ids,
    },
};
module_serdev_device_driver(bq27545_hdquart_drv);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("bq27545 HDQ-UART driver");
