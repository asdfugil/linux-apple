/*
 * Driver for SN2400 'Tigris' charger
 *
 * Copyright (C) 2020 Corellium LLC
 *
 * Based on: lp8727_charger.c
 *  Copyright (C) 2011 Texas Instruments
 *  Copyright (C) 2011 National Semiconductor
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/of.h>

#define REG_EVTMSK_0 0x01
#define REG_EVTMSK_1 0x02
#define REG_EVTMSK_2 0x03
#define REG_EVTSTS_0 0x04
#define REG_EVTSTS_1 0x05
#define REG_EVTSTS_2 0x06
#define REG_STAT 0x07
#define REG_STAT_POWER (1 << 0)
#define REG_STAT_ALIVE (1 << 4)
#define REG_STAT_VBUS (1 << 5)
#define REG_STAT_CHARGING (1 << 7)
#define REG_IBUSLIMIT 0x09
#define REG_ICHGLIMIT 0x0D
#define REG_ICHG 0x0E
#define REG_VCHG 0x0F
#define REG_CTRL 0x10
#define REG_CTRL_SPRDSPCT (3 << 0)
#define REG_CTRL_RSVD (1 << 2)
#define REG_CTRL_OFF (1 << 3)
#define REG_ADCCTL 0x11
#define REG_ADCCTL_RESET (1 << 0)
#define REG_ADCCTL_CH_MASK (7 << 1)
#define REG_ADCCTL_CH_SHIFT 1
#define REG_ADCCTL_CH_VBUS (0 << 1)
#define REG_ADCCTL_CH_IBUS (1 << 1)
#define REG_ADCCTL_CH_VSYS (3 << 1)
#define REG_ADCCTL_CH_VBAT (5 << 1)
#define REG_ADCCTL_START (1 << 4)
#define REG_ADCDATA 0x12
#define REG_HDQ 0x1D
#define REG_HDQ_FORCE_AP (1 << 0)
#define REG_HDQ_FORCE_TIGRIS (1 << 1)
#define REG_HDQ_REQ_AP (1 << 2)
#define REG_HDQ_REQ_TIGRIS (1 << 3)
#define REG_HDQ_ACK_AP (1 << 5)
#define REG_HDQ_RTE_AP (1 << 6)
#define REG_HDQ_RTE_TIGRIS (2 << 6)

#define POLL_INIT_MSEC 500
#define POLL_FAST_MSEC 300
#define POLL_MSEC 2000

struct sn2400_psy {
	struct power_supply *usb;
	struct power_supply *batt;
};

struct sn2400_chg {
	struct device *dev;
	struct i2c_client *client;
	struct mutex xfer_lock;
	struct sn2400_psy *psy;
	struct delayed_work work;

	u32 dt_charge_current;

	u8 stat;
	/* values in mV / mA */
	int usb_voltage, usb_current, usb_current_max;
	int batt_voltage;
	int batt_charge_current_max;
	int batt_charge_voltage, batt_charge_current;

	int ibus_foldback;
	int vbus_noload;
	int ibus_target;
};

#define IBUS_FOLDBACK_INIT -1
#define IBUS_FOLDBACK_IDLE 0
#define IBUS_FOLDBACK_NOLOAD 1
#define IBUS_FOLDBACK_PROBE 2
#define IBUS_FOLDBACK_DEBOUNCE 3
#define IBUS_FOLDBACK_SETTLE 4

#define MAX_DROOP 400 /* mV */
#define IBUS_PROBE_INIT 300 /* mA */
#define IBUS_PROBE_STEP 25 /* mA */

static int sn2400_read_bytes(struct sn2400_chg *pchg, u8 reg, u8 *data, u8 len)
{
	s32 ret;

	mutex_lock(&pchg->xfer_lock);
	ret = i2c_smbus_read_i2c_block_data(pchg->client, reg, len, data);
	mutex_unlock(&pchg->xfer_lock);

	return (ret != len) ? -EIO : 0;
}

static inline int sn2400_read_byte(struct sn2400_chg *pchg, u8 reg, u8 *data)
{
	return sn2400_read_bytes(pchg, reg, data, 1);
}

static int sn2400_write_byte(struct sn2400_chg *pchg, u8 reg, u8 data)
{
	int ret;

	mutex_lock(&pchg->xfer_lock);
	ret = i2c_smbus_write_byte_data(pchg->client, reg, data);
	mutex_unlock(&pchg->xfer_lock);

	return ret;
}

static int sn2400_read_adc(struct sn2400_chg *pchg, u8 chan, int *result)
{
	int ret, retry = 20;
	u8 val;

	ret = sn2400_write_byte(pchg, REG_ADCCTL,
				(chan & REG_ADCCTL_CH_MASK) | REG_ADCCTL_START);
	if (ret)
		return ret;

	while (1) {
		ret = sn2400_read_byte(pchg, REG_ADCCTL, &val);
		if (ret)
			return ret;
		if (!(val & REG_ADCCTL_START))
			break;
		if (!retry)
			return -ETIMEDOUT;
		retry--;
		msleep(5);
	}

	ret = sn2400_read_byte(pchg, REG_ADCDATA, &val);
	if (ret)
		return ret;

	switch (chan) {
	case REG_ADCCTL_CH_VBUS:
		*result = 25 * val;
		break;
	case REG_ADCCTL_CH_IBUS:
		*result = 10 * val;
		break;
	case REG_ADCCTL_CH_VSYS:
	case REG_ADCCTL_CH_VBAT:
		*result = 2500 + 10 * val;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sn2400_init_device(struct sn2400_chg *pchg)
{
	int ret;

	ret = sn2400_write_byte(pchg, REG_ICHGLIMIT,
				pchg->dt_charge_current / 10);
	if (ret)
		return ret;

	return sn2400_read_byte(pchg, REG_STAT, &pchg->stat);
}

static int sn2400_set_usb_current_max(struct sn2400_chg *pchg, unsigned val)
{
	int ret1, ret2;
	u8 ibuslimit, ctrl;

	if (val && val < 90)
		val = 90;
	if (val > 2000)
		val = 2000;

	pchg->ibus_target = val;

	ibuslimit = (val - 90) / 10;
	if (val)
		ctrl = REG_CTRL_SPRDSPCT | REG_CTRL_RSVD;
	else
		ctrl = REG_CTRL_RSVD | REG_CTRL_OFF;

	ret1 = sn2400_write_byte(pchg, REG_CTRL, ctrl);
	if (ret1)
		dev_warn(pchg->dev, "failed writing charger control: %d\n",
			 ret1);
	ret2 = sn2400_write_byte(pchg, REG_IBUSLIMIT, ibuslimit);
	if (ret2)
		dev_warn(pchg->dev, "failed writing USB current limit: %d\n",
			 ret2);
	return ret1 ? ret1 : ret2;
}

int sn2400_charger_hdq_mux(struct device *charger, struct device *client,
			   unsigned route);

static void sn2400_delayed_func(struct work_struct *_work)
{
	struct sn2400_chg *pchg =
		container_of(_work, struct sn2400_chg, work.work);
	int ret;
	u8 val1, val2[4], prev_stat;
	unsigned msec;

	prev_stat = pchg->stat;
	ret = sn2400_read_byte(pchg, REG_STAT, &pchg->stat);
	if (ret)
		dev_warn(pchg->dev, "failed reading status: %d\n", ret);

	ret = sn2400_read_adc(pchg, REG_ADCCTL_CH_VBUS, &pchg->usb_voltage);
	if (ret)
		dev_warn(pchg->dev, "failed reading USB voltage: %d\n", ret);

	ret = sn2400_read_adc(pchg, REG_ADCCTL_CH_IBUS, &pchg->usb_current);
	if (ret)
		dev_warn(pchg->dev, "failed reading USB current: %d\n", ret);

	ret = sn2400_read_byte(pchg, REG_IBUSLIMIT, &val1);
	if (!ret) {
		ret = sn2400_read_bytes(pchg, REG_ICHGLIMIT, &val2[0],
					sizeof(val2));
		if (!ret) {
			if (val2[REG_CTRL - REG_ICHGLIMIT] & REG_CTRL_OFF)
				pchg->usb_current_max = 0;
			else
				pchg->usb_current_max = 10 * val1 + 90;
			pchg->batt_charge_current =
				10 * val2[REG_ICHG - REG_ICHGLIMIT];
			pchg->batt_charge_voltage =
				10 * val2[REG_VCHG - REG_ICHGLIMIT] + 3000;
			pchg->batt_charge_current_max =
				10 * val2[REG_ICHGLIMIT - REG_ICHGLIMIT];
		} else
			dev_warn(pchg->dev,
				 "failed reading charger controls: %d\n", ret);
	} else
		dev_warn(pchg->dev, "failed reading USB current limit: %d\n",
			 ret);

	ret = sn2400_read_adc(pchg, REG_ADCCTL_CH_VBAT, &pchg->batt_voltage);
	if (ret)
		dev_warn(pchg->dev, "failed reading battery voltage: %d\n",
			 ret);

	if (((prev_stat ^ pchg->stat) & REG_STAT_VBUS) ||
	    pchg->ibus_foldback == IBUS_FOLDBACK_INIT) {
		if (pchg->stat & REG_STAT_VBUS)
			pchg->ibus_foldback = IBUS_FOLDBACK_NOLOAD;
		else
			pchg->ibus_foldback = IBUS_FOLDBACK_IDLE;
		sn2400_set_usb_current_max(pchg, 0);
	} else {
		switch (pchg->ibus_foldback) {
		case IBUS_FOLDBACK_NOLOAD:
			pchg->vbus_noload = pchg->usb_voltage;
			sn2400_charger_hdq_mux(pchg->dev, pchg->dev, 0);
			sn2400_set_usb_current_max(pchg, IBUS_PROBE_INIT);
			pchg->ibus_foldback = IBUS_FOLDBACK_PROBE;
			break;
		case IBUS_FOLDBACK_PROBE:
			if (pchg->vbus_noload - pchg->usb_voltage > MAX_DROOP)
				pchg->ibus_foldback = IBUS_FOLDBACK_DEBOUNCE;
			else if (pchg->ibus_target < pchg->usb_current)
				sn2400_set_usb_current_max(
					pchg,
					pchg->ibus_target + IBUS_PROBE_STEP);
			else
				sn2400_set_usb_current_max(pchg,
							   pchg->ibus_target);
			break;
		case IBUS_FOLDBACK_DEBOUNCE:
			if (pchg->vbus_noload - pchg->usb_voltage > MAX_DROOP) {
				sn2400_set_usb_current_max(
					pchg,
					pchg->ibus_target - IBUS_PROBE_STEP);
				pchg->ibus_foldback = IBUS_FOLDBACK_SETTLE;
			} else
				pchg->ibus_foldback = IBUS_FOLDBACK_PROBE;
			break;
		}
	}

#if 0
    dev_warn(pchg->dev, "usb:%d vbus:%d ibus:%d ibuslim:%d foldback:%d vbat:%d vchg:%d ichg:%d ichglim:%d\n",
        !!(pchg->stat & REG_STAT_VBUS), pchg->usb_voltage, pchg->usb_current, pchg->usb_current_max, pchg->ibus_foldback,
        pchg->batt_voltage, pchg->batt_charge_voltage, pchg->batt_charge_current, pchg->batt_charge_current_max);
#endif

	power_supply_changed(pchg->psy->usb);
	power_supply_changed(pchg->psy->batt);

	if (pchg->ibus_foldback == IBUS_FOLDBACK_NOLOAD ||
	    pchg->ibus_foldback == IBUS_FOLDBACK_PROBE ||
	    pchg->ibus_foldback == IBUS_FOLDBACK_DEBOUNCE)
		msec = POLL_FAST_MSEC;
	else
		msec = POLL_MSEC;
	schedule_delayed_work(&pchg->work, msecs_to_jiffies(msec));
}

static enum power_supply_property sn2400_charger_prop[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

/* other properties would have to be read from the HDQ gas gauge */
static enum power_supply_property sn2400_battery_prop[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

static int sn2400_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sn2400_chg *pchg = dev_get_drvdata(psy->dev.parent);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(pchg->stat & REG_STAT_VBUS);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pchg->usb_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = pchg->usb_current * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = pchg->usb_current_max * 1000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sn2400_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sn2400_chg *pchg = dev_get_drvdata(psy->dev.parent);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (pchg->stat & REG_STAT_VBUS) {
			if (pchg->stat & REG_STAT_CHARGING)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_FULL;
		} else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = pchg->batt_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = pchg->batt_charge_current * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = pchg->batt_charge_current_max * 1000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		val->intval = pchg->batt_charge_voltage * 1000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct power_supply_desc sn2400_usb_desc = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sn2400_charger_prop,
	.num_properties = ARRAY_SIZE(sn2400_charger_prop),
	.get_property = sn2400_charger_get_property,
};

static const struct power_supply_desc sn2400_batt_desc = {
	.name = "charger",
	.type = POWER_SUPPLY_TYPE_UPS,
	.properties = sn2400_battery_prop,
	.num_properties = ARRAY_SIZE(sn2400_battery_prop),
	.get_property = sn2400_battery_get_property,
};

static char *battery_supplied_to[] = {
	"charger",
};

static int sn2400_register_psy(struct sn2400_chg *pchg)
{
	struct power_supply_config psy_cfg = {};
	struct sn2400_psy *psy;

	psy = devm_kzalloc(pchg->dev, sizeof(*psy), GFP_KERNEL);
	if (!psy)
		return -ENOMEM;

	pchg->psy = psy;

	psy_cfg.supplied_to = battery_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(battery_supplied_to);

	psy->usb = power_supply_register(pchg->dev, &sn2400_usb_desc, &psy_cfg);
	if (IS_ERR(psy->usb))
		goto err_psy_usb;

	psy->batt = power_supply_register(pchg->dev, &sn2400_batt_desc, NULL);
	if (IS_ERR(psy->batt))
		goto err_psy_batt;

	return 0;

err_psy_batt:
	power_supply_unregister(psy->usb);
err_psy_usb:
	return -EPERM;
}

/*static int sn2400_charger_register(struct device *charger, struct device *client)
{
	struct sn2400_chg *pchg = dev_get_drvdata(charger);
	if (!pchg)
		return -EPROBE_DEFER;
	return 0;
}*/

int sn2400_charger_hdq_mux(struct device *charger, struct device *client,
			   unsigned route)
{
	int ret, timeout = 100;
	struct sn2400_chg *pchg = dev_get_drvdata(charger);
	u8 hdqr;

	if (!pchg)
		return -EPROBE_DEFER;

	ret = sn2400_read_byte(pchg, REG_STAT, &hdqr);
	if (ret)
		return ret;

	if ((hdqr & REG_STAT_CHARGING) && !route) {
		ret = sn2400_write_byte(pchg, REG_HDQ, 0);
		return ret;
	}

	ret = sn2400_write_byte(pchg, REG_HDQ, REG_HDQ_REQ_AP);
	if (ret)
		return ret;

	while (1) {
		ret = sn2400_read_byte(pchg, REG_HDQ, &hdqr);
		if (ret)
			return ret;
		if (hdqr & REG_HDQ_ACK_AP)
			break;
		timeout--;
		if (!timeout) {
			dev_err(pchg->dev,
				"timed out switching HDQ routing to %d\n",
				route);
			return -ETIMEDOUT;
		}
		msleep(10);
	}

	if (!route) {
		ret = sn2400_write_byte(pchg, REG_HDQ,
					REG_HDQ_REQ_AP | REG_HDQ_FORCE_TIGRIS);
		if (ret)
			return ret;
	}

	return 0;
}

static int sn2400_probe(struct i2c_client *cl)
{
	struct device_node *node = cl->dev.of_node;
	struct sn2400_chg *pchg;
	int ret;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	pchg = devm_kzalloc(&cl->dev, sizeof(*pchg), GFP_KERNEL);
	if (!pchg)
		return -ENOMEM;

	if (of_property_read_u32_index(node, "charge-current", 0,
				       &pchg->dt_charge_current)) {
		dev_err(pchg->dev,
			"charger requires 'charge-current' in device tree.\n");
		return -EINVAL;
	}

	pchg->client = cl;
	pchg->dev = &cl->dev;
	i2c_set_clientdata(cl, pchg);

	mutex_init(&pchg->xfer_lock);

	ret = sn2400_init_device(pchg);
	if (ret) {
		dev_err(pchg->dev, "i2c communication err: %d.", ret);
		return ret;
	}

	ret = sn2400_register_psy(pchg);
	if (ret) {
		dev_err(pchg->dev, "power supplies register err: %d.", ret);
		return ret;
	}

	pchg->ibus_foldback = IBUS_FOLDBACK_INIT;

	INIT_DELAYED_WORK(&pchg->work, sn2400_delayed_func);
	schedule_delayed_work(&pchg->work, msecs_to_jiffies(POLL_INIT_MSEC));

	return 0;
}

static const struct of_device_id sn2400_dt_ids[] = {
	{
		.compatible = "ti,sn2400",
	},
	{}
};

MODULE_DEVICE_TABLE(of, sn2400_dt_ids);

static struct i2c_driver sn2400_driver = {
    .driver = {
        .name = "sn2400",
        .of_match_table = of_match_ptr(sn2400_dt_ids),
    },
    .probe = sn2400_probe,
};
module_i2c_driver(sn2400_driver);

MODULE_DESCRIPTION("SN2400 charger driver");
MODULE_AUTHOR("Corellium LLC");
MODULE_LICENSE("GPL");
