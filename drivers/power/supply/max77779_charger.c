// SPDX-License-Identifier: GPL-2.0-only
/*
 * Conservative wired charger driver for the MAX77779.
 *
 * Copyright 2026 Oleksii Onchul <oleksiionchul@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/gpio/consumer.h>
#include <linux/linear_range.h>
#include <linux/mfd/max77779.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/workqueue.h>

#define MAX77779_SAFE_CHARGE_CURRENT_UA	466680
#define MAX77779_SAFE_INPUT_CURRENT_UA	500000
#define MAX77779_SAFE_FLOAT_VOLTAGE_MV	4200
#define MAX77779_POLL_INTERVAL		(5 * HZ)

static const struct linear_range max77779_chgcc_ranges[] = {
	LINEAR_RANGE(133330, 0x0, 0x2, 0),
	LINEAR_RANGE(200000, 0x3, 0x3c, 66670),
};

static const struct linear_range max77779_chgcv_ranges[] = {
	LINEAR_RANGE(3800, 0x38, 0x39, 100),
	LINEAR_RANGE(4000, 0x0, 0x32, 10),
};

static const struct linear_range max77779_chgin_ilim_ranges[] = {
	LINEAR_RANGE(100000, 0x3, 0x7f, 25000),
};

struct max77779_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply *input;
	struct regulator_dev *otg_rdev;
	struct gpio_desc *ext_bst_ctl;
	struct notifier_block notifier;
	struct delayed_work work;
	struct mutex lock; /* protects mode */
	enum max77779_chgr_mode mode;
	bool otg_enabled;
	int input_limit_ua;
	bool charging;
	bool hardware_fallback;
	int last_details;
};

static int __max77779_set_mode(struct max77779_charger *chg,
			       enum max77779_chgr_mode mode)
{
	int ret;

	if (chg->mode == mode)
		return 0;

	ret = regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_00,
				 MAX77779_CHGR_REG_CHG_CNFG_00_MODE, mode);
	if (!ret)
		chg->mode = mode;

	return ret;
}

static int max77779_set_mode(struct max77779_charger *chg,
			     enum max77779_chgr_mode mode)
{
	guard(mutex)(&chg->lock);

	/*
	 * Do not let the charger polling worker overwrite an active OTG mode.
	 * External-boost boards remain in OFF mode while sourcing VBUS.
	 */
	if (chg->otg_enabled && mode != chg->mode)
		return -EBUSY;

	return __max77779_set_mode(chg, mode);
}

static int max77779_otg_enable(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);
	int ret;

	guard(mutex)(&chg->lock);

	ret = regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_12,
				 MAX77779_CHGR_REG_CHG_CNFG_12_CHGINSEL |
				 MAX77779_CHGR_REG_CHG_CNFG_12_CHG_EN, 0);
	if (ret)
		return ret;

	chg->charging = false;
	chg->input_limit_ua = 0;
	chg->otg_enabled = true;

	if (chg->ext_bst_ctl) {
		ret = __max77779_set_mode(chg, MAX77779_CHGR_MODE_OFF);
		if (ret)
			dev_warn(chg->dev, "OTG: charger off failed: %d\n", ret);

		gpiod_set_value_cansleep(chg->ext_bst_ctl, 1);
		dev_info(chg->dev, "OTG VBUS on: external boost\n");
		return 0;
	}

	ret = __max77779_set_mode(chg, MAX77779_CHGR_MODE_OTG_BOOST_ON);
	if (ret)
		chg->otg_enabled = false;

	return ret;
}

static int max77779_otg_disable(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);

	guard(mutex)(&chg->lock);

	if (chg->ext_bst_ctl)
		gpiod_set_value_cansleep(chg->ext_bst_ctl, 0);

	chg->otg_enabled = false;
	return __max77779_set_mode(chg, MAX77779_CHGR_MODE_OFF);
}

static int max77779_otg_is_enabled(struct regulator_dev *rdev)
{
	struct max77779_charger *chg = rdev_get_drvdata(rdev);

	guard(mutex)(&chg->lock);

	return chg->otg_enabled;
}

static const struct regulator_ops max77779_otg_ops = {
	.enable = max77779_otg_enable,
	.disable = max77779_otg_disable,
	.is_enabled = max77779_otg_is_enabled,
};

static const struct regulator_desc max77779_otg_desc = {
	.name = "otg",
	.of_match = of_match_ptr("otg-regulator"),
	.owner = THIS_MODULE,
	.ops = &max77779_otg_ops,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int max77779_set_charge_current(struct max77779_charger *chg, int ua)
{
	bool found;
	unsigned int selector;

	linear_range_get_selector_high_array(max77779_chgcc_ranges,
					     ARRAY_SIZE(max77779_chgcc_ranges),
					     ua, &selector, &found);
	if (!found)
		return -EINVAL;

	return regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_02,
				  MAX77779_CHGR_REG_CHG_CNFG_02_CHGCC,
				  selector);
}

static int max77779_set_float_voltage(struct max77779_charger *chg, int mv)
{
	bool found;
	unsigned int selector;

	linear_range_get_selector_high_array(max77779_chgcv_ranges,
					     ARRAY_SIZE(max77779_chgcv_ranges),
					     mv, &selector, &found);
	if (!found)
		return -EINVAL;

	return regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_04,
				  MAX77779_CHGR_REG_CHG_CNFG_04_CHG_CV_PRM,
				  selector);
}

static int max77779_disable_watchdog(struct max77779_charger *chg)
{
	int ret;

	ret = regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_06,
				 MAX77779_CHGR_REG_CHG_CNFG_06_CHGPROT,
				 MAX77779_CHGR_REG_CHG_CNFG_06_CHGPROT);
	if (ret)
		return ret;

	ret = regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_15,
				 MAX77779_CHGR_REG_CHG_CNFG_15_WDTEN, 0);

	if (regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_06,
			       MAX77779_CHGR_REG_CHG_CNFG_06_CHGPROT, 0) &&
	    !ret)
		ret = -EIO;

	return ret;
}

static int max77779_set_input_limit(struct max77779_charger *chg, int ua)
{
	unsigned int selector;

	ua = clamp(ua, 100000, MAX77779_SAFE_INPUT_CURRENT_UA);
	linear_range_get_selector_within(max77779_chgin_ilim_ranges, ua,
					 &selector);

	return regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_09,
				  MAX77779_CHGR_REG_CHG_CNFG_09_CHGIN_ILIM,
				  selector);
}

static int max77779_input_valid(struct max77779_charger *chg)
{
	unsigned int value;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_INT_OK, &value);
	if (ret)
		return ret;

	return !!(value & MAX77779_CHGR_REG_CHG_INT_OK_CHGIN);
}

static int max77779_check_charging(struct max77779_charger *chg)
{
	unsigned int cnfg00;
	unsigned int cnfg12;
	unsigned int details;
	unsigned int int_ok;
	unsigned int chg_details;
	bool active;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_INT_OK, &int_ok);
	if (ret)
		return ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_DETAILS_01,
			  &details);
	if (ret)
		return ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_00,
			  &cnfg00);
	if (ret)
		return ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_12,
			  &cnfg12);
	if (ret)
		return ret;

	chg_details = FIELD_GET(MAX77779_CHGR_REG_CHG_DETAILS_01_CHG_DTLS,
				details);
	active = chg_details <= MAX77779_CHGR_CHG_DTLS_DONE;

	if (chg->last_details != chg_details) {
		chg->last_details = chg_details;
		dev_info(chg->dev,
			 "state int_ok=%#02x details=%#02x mode=%#02x cnfg12=%#02x\n",
			 int_ok, details,
			 (unsigned int)FIELD_GET(
				 MAX77779_CHGR_REG_CHG_CNFG_00_MODE, cnfg00),
			 cnfg12);
	}

	return active;
}

static int max77779_get_status(struct max77779_charger *chg)
{
	unsigned int value;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_DETAILS_01,
			  &value);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77779_CHGR_REG_CHG_DETAILS_01_CHG_DTLS, value)) {
	case MAX77779_CHGR_CHG_DTLS_PREQUAL:
	case MAX77779_CHGR_CHG_DTLS_CC:
	case MAX77779_CHGR_CHG_DTLS_CV:
	case MAX77779_CHGR_CHG_DTLS_TO:
		return POWER_SUPPLY_STATUS_CHARGING;
	case MAX77779_CHGR_CHG_DTLS_DONE:
		return POWER_SUPPLY_STATUS_FULL;
	case MAX77779_CHGR_CHG_DTLS_OFF:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	case MAX77779_CHGR_CHG_DTLS_TIMER_FAULT:
	case MAX77779_CHGR_CHG_DTLS_SUSP_BATT_THM:
	case MAX77779_CHGR_CHG_DTLS_OFF_HIGH_TEMP:
	case MAX77779_CHGR_CHG_DTLS_OFF_WDOG_TIMER:
	case MAX77779_CHGR_CHG_DTLS_SUSP_JEITA:
	case MAX77779_CHGR_CHG_DTLS_OFF_TEMP:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	default:
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}
}

static int max77779_get_charge_type(struct max77779_charger *chg)
{
	unsigned int value;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_DETAILS_01,
			  &value);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77779_CHGR_REG_CHG_DETAILS_01_CHG_DTLS, value)) {
	case MAX77779_CHGR_CHG_DTLS_PREQUAL:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case MAX77779_CHGR_CHG_DTLS_CC:
	case MAX77779_CHGR_CHG_DTLS_CV:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case MAX77779_CHGR_CHG_DTLS_TO:
		return POWER_SUPPLY_CHARGE_TYPE_STANDARD;
	case MAX77779_CHGR_CHG_DTLS_DONE:
	case MAX77779_CHGR_CHG_DTLS_OFF:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int max77779_get_health(struct max77779_charger *chg)
{
	unsigned int value;
	unsigned int details;
	int ret;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_DETAILS_00,
			  &value);
	if (ret)
		return ret;

	details = FIELD_GET(MAX77779_CHGR_REG_CHG_DETAILS_00_CHGIN_DTLS,
			    value);
	if (details == MAX77779_CHGR_CHGIN_DTLS_VBUS_UNDERVOLTAGE ||
	    details == MAX77779_CHGR_CHGIN_DTLS_VBUS_MARGINAL_VOLTAGE)
		return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	if (details == MAX77779_CHGR_CHGIN_DTLS_VBUS_OVERVOLTAGE)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_DETAILS_01,
			  &value);
	if (ret)
		return ret;

	switch (FIELD_GET(MAX77779_CHGR_REG_CHG_DETAILS_01_BAT_DTLS, value)) {
	case MAX77779_CHGR_BAT_DTLS_NO_BATT_CHG_SUSP:
		return POWER_SUPPLY_HEALTH_NO_BATTERY;
	case MAX77779_CHGR_BAT_DTLS_DEAD_BATTERY:
		return POWER_SUPPLY_HEALTH_DEAD;
	case MAX77779_CHGR_BAT_DTLS_BAT_CHG_TIMER_FAULT:
		return POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	case MAX77779_CHGR_BAT_DTLS_BAT_UNDERVOLTAGE:
		return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	case MAX77779_CHGR_BAT_DTLS_BAT_OVERVOLTAGE:
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	case MAX77779_CHGR_BAT_DTLS_BAT_OVERCURRENT:
		return POWER_SUPPLY_HEALTH_OVERCURRENT;
	default:
		return POWER_SUPPLY_HEALTH_GOOD;
	}
}

static const enum power_supply_property max77779_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int max77779_charger_get_property(struct power_supply *psy,
					 enum power_supply_property property,
					 union power_supply_propval *value)
{
	struct max77779_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	switch (property) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = max77779_input_valid(chg);
		if (ret > 0)
			ret = chg->mode == MAX77779_CHGR_MODE_CHG_BUCK_ON;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = max77779_input_valid(chg);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = max77779_get_status(chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = max77779_get_charge_type(chg);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = max77779_get_health(chg);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = MAX77779_SAFE_CHARGE_CURRENT_UA;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = MAX77779_SAFE_FLOAT_VOLTAGE_MV * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = chg->input_limit_ua;
		break;
	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;

	value->intval = ret;
	return 0;
}

static const struct power_supply_desc max77779_charger_desc = {
	.name = "max77779-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = max77779_charger_properties,
	.num_properties = ARRAY_SIZE(max77779_charger_properties),
	.get_property = max77779_charger_get_property,
};

static int max77779_sync_input(struct max77779_charger *chg)
{
	union power_supply_propval online;
	union power_supply_propval current_max;
	unsigned int jeita;
	int hardware_present;
	int ret;

	hardware_present = max77779_input_valid(chg);
	if (hardware_present < 0)
		return hardware_present;

	ret = power_supply_get_property(chg->input, POWER_SUPPLY_PROP_ONLINE,
					&online);
	if (ret)
		online.intval = 0;

	ret = power_supply_get_property(chg->input,
					POWER_SUPPLY_PROP_CURRENT_MAX,
					&current_max);
	if (ret)
		current_max.intval = 0;

	if (online.intval && current_max.intval > 0) {
		chg->input_limit_ua = min(current_max.intval,
					  MAX77779_SAFE_INPUT_CURRENT_UA);
		if (chg->hardware_fallback) {
			chg->hardware_fallback = false;
			dev_info(chg->dev, "TCPM input information is available\n");
		}
	} else if (hardware_present) {
		chg->input_limit_ua = MAX77779_SAFE_INPUT_CURRENT_UA;
		if (!chg->hardware_fallback) {
			chg->hardware_fallback = true;
			dev_warn(chg->dev,
				 "VBUS valid but TCPM offline, using %d mA fallback\n",
				 chg->input_limit_ua / 1000);
		}
	} else {
		ret = 0;
		goto disable;
	}

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_08, &jeita);
	if (ret)
		goto disable;
	if (!(jeita & MAX77779_CHGR_REG_CHG_CNFG_08_THM1_JEITA_EN)) {
		dev_err_ratelimited(chg->dev,
				    "refusing to charge without hardware JEITA\n");
		ret = -EPERM;
		goto disable;
	}

	ret = max77779_set_input_limit(chg, chg->input_limit_ua);
	if (ret)
		goto disable;

	ret = regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_12,
				 MAX77779_CHGR_REG_CHG_CNFG_12_CHGINSEL |
				 MAX77779_CHGR_REG_CHG_CNFG_12_WCINSEL |
				 MAX77779_CHGR_REG_CHG_CNFG_12_CHG_EN,
				 MAX77779_CHGR_REG_CHG_CNFG_12_CHGINSEL |
				 MAX77779_CHGR_REG_CHG_CNFG_12_CHG_EN);
	if (ret)
		goto disable;

	ret = max77779_set_mode(chg, MAX77779_CHGR_MODE_CHG_BUCK_ON);
	if (ret)
		goto disable;

	ret = max77779_check_charging(chg);
	if (ret < 0)
		goto disable;

	if (ret && !chg->charging) {
		chg->charging = true;
		dev_info(chg->dev, "wired charging started at %d mA\n",
			 chg->input_limit_ua / 1000);
	} else if (!ret && chg->charging) {
		chg->charging = false;
		dev_warn(chg->dev, "charger left the active state\n");
	}

	return 0;

disable:
	chg->input_limit_ua = 0;
	max77779_set_mode(chg, MAX77779_CHGR_MODE_OFF);
	regmap_update_bits(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_12,
			   MAX77779_CHGR_REG_CHG_CNFG_12_CHGINSEL, 0);
	chg->hardware_fallback = false;
	if (chg->charging) {
		chg->charging = false;
		dev_info(chg->dev, "wired charging stopped\n");
	}
	return ret;
}

static void max77779_charger_work(struct work_struct *work)
{
	struct max77779_charger *chg =
		container_of(work, struct max77779_charger, work.work);
	int ret;

	ret = max77779_sync_input(chg);
	if (ret && ret != -EPERM)
		dev_dbg(chg->dev, "charger input sync failed: %d\n", ret);

	power_supply_changed(chg->psy);
	schedule_delayed_work(&chg->work, MAX77779_POLL_INTERVAL);
}

static int max77779_charger_notifier(struct notifier_block *notifier,
				     unsigned long event, void *data)
{
	struct max77779_charger *chg =
		container_of(notifier, struct max77779_charger, notifier);

	if (event == PSY_EVENT_PROP_CHANGED && data == chg->input)
		mod_delayed_work(system_wq, &chg->work, 0);

	return NOTIFY_OK;
}

static void max77779_unregister_notifier(void *data)
{
	power_supply_unreg_notifier(data);
}

static int max77779_charger_probe(struct platform_device *pdev)
{
	struct power_supply_config supply_config = {};
	struct regulator_config regulator_config = {};
	struct max77779_charger *chg;
	unsigned int mode;
	int ret;

	device_set_of_node_from_dev(&pdev->dev, pdev->dev.parent);

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	chg->last_details = -1;
	chg->regmap = dev_get_regmap(pdev->dev.parent, "charger");
	if (!chg->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "missing charger regmap\n");

	ret = devm_mutex_init(&pdev->dev, &chg->lock);
	if (ret)
		return ret;

	chg->ext_bst_ctl = devm_gpiod_get_optional(&pdev->dev, "extbst",
						   GPIOD_OUT_LOW);
	if (IS_ERR(chg->ext_bst_ctl))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->ext_bst_ctl),
				     "failed to get external boost GPIO\n");

	ret = regmap_read(chg->regmap, MAX77779_CHGR_REG_CHG_CNFG_00, &mode);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to read charger mode\n");
	chg->mode = FIELD_GET(MAX77779_CHGR_REG_CHG_CNFG_00_MODE, mode);

	ret = max77779_set_mode(chg, MAX77779_CHGR_MODE_OFF);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to stop charger\n");

	ret = max77779_set_charge_current(chg,
					  MAX77779_SAFE_CHARGE_CURRENT_UA);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to set charge current\n");

	ret = max77779_set_float_voltage(chg,
					 MAX77779_SAFE_FLOAT_VOLTAGE_MV);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to set float voltage\n");

	ret = max77779_disable_watchdog(chg);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to disable charger watchdog\n");

	chg->input = devm_power_supply_get_by_reference(&pdev->dev,
							"power-supplies");
	if (IS_ERR(chg->input))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->input),
				     "failed to get USB Type-C supply\n");
	if (!chg->input)
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
				     "USB Type-C supply is not ready\n");

	regulator_config.dev = &pdev->dev;
	regulator_config.driver_data = chg;
	regulator_config.of_node = dev_of_node(&pdev->dev);
	chg->otg_rdev = devm_regulator_register(&pdev->dev,
						&max77779_otg_desc,
						&regulator_config);
	if (IS_ERR(chg->otg_rdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->otg_rdev),
				     "failed to register OTG regulator\n");

	supply_config.fwnode = dev_fwnode(&pdev->dev);
	supply_config.drv_data = chg;
	chg->psy = devm_power_supply_register(&pdev->dev,
					      &max77779_charger_desc,
					      &supply_config);
	if (IS_ERR(chg->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->psy),
				     "failed to register charger supply\n");

	ret = devm_delayed_work_autocancel(&pdev->dev, &chg->work,
					   max77779_charger_work);
	if (ret)
		return ret;

	chg->notifier.notifier_call = max77779_charger_notifier;
	ret = power_supply_reg_notifier(&chg->notifier);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register supply notifier\n");

	ret = devm_add_action_or_reset(&pdev->dev,
				       max77779_unregister_notifier,
				       &chg->notifier);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, chg);
	schedule_delayed_work(&chg->work, 0);

	dev_info(&pdev->dev,
		 "registered, charge limited to %d mA at %d mV (input %d mA)\n",
		 MAX77779_SAFE_CHARGE_CURRENT_UA / 1000,
		 MAX77779_SAFE_FLOAT_VOLTAGE_MV,
		 MAX77779_SAFE_INPUT_CURRENT_UA / 1000);

	return 0;
}

static const struct platform_device_id max77779_charger_id[] = {
	{ "max77779-charger" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max77779_charger_id);

static struct platform_driver max77779_charger_driver = {
	.driver = {
		.name = "max77779-charger",
	},
	.probe = max77779_charger_probe,
	.id_table = max77779_charger_id,
};
module_platform_driver(max77779_charger_driver);

MODULE_AUTHOR("Oleksii Onchul <oleksiionchul@gmail.com>");
MODULE_DESCRIPTION("MAX77779 wired battery charger and OTG regulator");
MODULE_LICENSE("GPL");
