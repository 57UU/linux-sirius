// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2025 Vasiliy Doylov <nekocwd@mainlining.org>
// Copyright (c) 2025 Frieder Hannenheim <mail@fhannenheim.net>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-event.h>

#define LC898219XI_NAME "lc898219xi"
/* Actuator has 11 bit resolution */
#define LC898219XI_MAX_FOCUS_POS (4096 - 1)
#define LC898219XI_MIN_FOCUS_POS 0
#define LC898219XI_FOCUS_STEPS 1

#define LC898219XI_MSB_ADDR 132

static const char *const lc898219xi_supply_names[] = {
	"vdd",
	"vio",
	"vana",
};

struct lc898219xi {
	struct regulator_bulk_data supplies[ARRAY_SIZE(lc898219xi_supply_names)];
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *focus;
	struct v4l2_subdev sd;
};

static inline struct lc898219xi *sd_to_lc898219xi(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct lc898219xi, sd);
}

static int lc898219xi_set_dac(struct lc898219xi *lc898219xi, u16 val)
{
	uint32_t EEPROM_3Fh = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&lc898219xi->sd);

	struct device* dev = lc898219xi->sd.dev;

	unsigned short addr_bak = client->addr;
	client->addr = 0x73;
	EEPROM_3Fh = i2c_smbus_read_byte_data(client, 0x3F);
	client->addr = addr_bak;
	uint32_t regdata = i2c_smbus_read_byte_data(client, 0xF0);
	if (regdata == 0xA5) {
		dev_info(dev, "check communication success\n");
	} else {
		dev_err(dev, "check communication error regdata: %x \n", regdata);
		// return -1;
	}
	usleep_range(1000, 1010);
	i2c_smbus_write_byte_data(client, 0x8E, 0x15);
	i2c_smbus_write_byte_data(client, 0x8D, 0x20);

	i2c_smbus_write_byte_data(client, 0x81, 0x80);
	usleep_range(1000, 1010);

	uint16_t init_temp = i2c_smbus_read_word_swapped(client, 0x58);
	uint16_t init_temp_A;
	uint16_t init_temp_B;
	
	if ( ((EEPROM_3Fh & 0x38) >> 3) == 2)
		init_temp_A = init_temp << 2;
	else if ( ((EEPROM_3Fh & 0x38) >> 3) == 1)
		init_temp_A = init_temp << 1;
	else
		init_temp_A = init_temp;

	if ( (EEPROM_3Fh & 0x07) == 2)
		init_temp_B = init_temp << 2;
	else if ( (EEPROM_3Fh & 0x07) == 1)
		init_temp_B = init_temp << 1;
	else
		init_temp_B = init_temp;

	i2c_smbus_write_byte_data(client, 0xE0, 0x01);
	msleep(8);

	int retry;
	for (retry = 0; retry < 10; retry++) {
		uint32_t check = i2c_smbus_read_byte_data(client, 0xB3);
		if ( (check & 0XE0) == 0 ) {
			break;
		} else {
			if (retry >= 9) {
				dev_err(dev, "LSI wake up check failed");
				// return -1;
			}
		}
		usleep_range(1000, 1010);
	}

	
	/* Write Init Temperature Data */
	i2c_smbus_write_word_swapped(client, 0x30, init_temp_A);
	i2c_smbus_write_word_swapped(client, 0x32, init_temp_A);
	i2c_smbus_write_word_swapped(client, 0x76, init_temp_B);
	i2c_smbus_write_word_swapped(client, 0x78, init_temp_B);

	i2c_smbus_write_byte_data(client, 0x8C, 0xE9);
	return i2c_smbus_write_word_swapped(client, LC898219XI_MSB_ADDR, val);
}

static int __maybe_unused lc898219xi_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898219xi *lc898219xi = sd_to_lc898219xi(sd);

	regulator_bulk_disable(ARRAY_SIZE(lc898219xi_supply_names),
			       lc898219xi->supplies);

	return 0;
}

static int __maybe_unused lc898219xi_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898219xi *lc898219xi = sd_to_lc898219xi(sd);
	int ret;
	uint32_t EEPROM_3Fh = 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(lc898219xi_supply_names),
				    lc898219xi->supplies);

	if (ret < 0) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	usleep_range(8000, 10000);

	dev_warn(dev, "writing enable bit\n");
	struct i2c_client *client = v4l2_get_subdevdata(&lc898219xi->sd);
	unsigned short addr_bak = client->addr;
	client->addr = 0x73;
	EEPROM_3Fh = i2c_smbus_read_byte_data(client, 0x3F);
	client->addr = addr_bak;
	uint32_t regdata = i2c_smbus_read_byte_data(client, 0xF0);
	if (regdata == 0xA5) {
		dev_info(dev, "check communication success\n");
	} else {
		dev_err(dev, "check communication error regdata: %x \n", regdata);
		// return -1;
	}
	usleep_range(1000, 1010);
	i2c_smbus_write_byte_data(client, 0x8E, 0x15);
	i2c_smbus_write_byte_data(client, 0x8D, 0x20);

	i2c_smbus_write_byte_data(client, 0x81, 0x80);
	usleep_range(1000, 1010);

	uint16_t init_temp = i2c_smbus_read_word_swapped(client, 0x58);
	uint16_t init_temp_A;
	uint16_t init_temp_B;

	if ( ((EEPROM_3Fh & 0x38) >> 3) == 2)
		init_temp_A = init_temp << 2;
	else if ( ((EEPROM_3Fh & 0x38) >> 3) == 1)
		init_temp_A = init_temp << 1;
	else
		init_temp_A = init_temp;

	if ( (EEPROM_3Fh & 0x07) == 2)
		init_temp_B = init_temp << 2;
	else if ( (EEPROM_3Fh & 0x07) == 1)
		init_temp_B = init_temp << 1;
	else
		init_temp_B = init_temp;

	i2c_smbus_write_byte_data(client, 0xE0, 0x01);
	msleep(8);

	int retry;
	for (retry = 0; retry < 10; retry++) {
		uint32_t check = i2c_smbus_read_byte_data(client, 0xB3);
		if ( (check & 0XE0) == 0 ) {
			break;
		} else {
			if (retry >= 9) {
				dev_err(dev, "LSI wake up check failed");
				// return -1;
			}
		}
		usleep_range(1000, 1010);
	}


	/* Write Init Temperature Data */
	i2c_smbus_write_word_swapped(client, 0x30, init_temp_A);
	i2c_smbus_write_word_swapped(client, 0x32, init_temp_A);
	i2c_smbus_write_word_swapped(client, 0x76, init_temp_B);
	i2c_smbus_write_word_swapped(client, 0x78, init_temp_B);

	i2c_smbus_write_byte_data(client, 0x8C, 0xE9);

	return ret;
}

static int lc898219xi_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898219xi *lc898219xi =
		container_of(ctrl->handler, struct lc898219xi, ctrls);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return lc898219xi_set_dac(lc898219xi, ctrl->val);

	return 0;
}

static const struct v4l2_ctrl_ops lc898219xi_ctrl_ops = {
	.s_ctrl = lc898219xi_set_ctrl,
};

static int lc898219xi_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int lc898219xi_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_mark_last_busy(sd->dev);
	pm_runtime_put_autosuspend(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops lc898219xi_int_ops = {
	.open = lc898219xi_open,
	.close = lc898219xi_close,
};

static const struct v4l2_subdev_core_ops lc898219xi_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops lc898219xi_ops = {
	.core = &lc898219xi_core_ops,
};

static int lc898219xi_init_controls(struct lc898219xi *lc898219xi)
{
	struct v4l2_ctrl_handler *hdl = &lc898219xi->ctrls;
	const struct v4l2_ctrl_ops *ops = &lc898219xi_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	lc898219xi->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
					      LC898219XI_MIN_FOCUS_POS,
					      LC898219XI_MAX_FOCUS_POS,
					      LC898219XI_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	lc898219xi->sd.ctrl_handler = hdl;

	return 0;
}

static int lc898219xi_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lc898219xi *lc898219xi;
	unsigned int i;
	int ret;

	dev_info(dev, "Running probe for lc898219xi\n");

	lc898219xi = devm_kzalloc(dev, sizeof(*lc898219xi), GFP_KERNEL);
	if (!lc898219xi)
		return -ENOMEM;

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&lc898219xi->sd, client, &lc898219xi_ops);

	for (i = 0; i < ARRAY_SIZE(lc898219xi_supply_names); i++)
		lc898219xi->supplies[i].supply = lc898219xi_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(lc898219xi_supply_names),
				      lc898219xi->supplies);

	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Initialize controls */
	ret = lc898219xi_init_controls(lc898219xi);
	if (ret)
		goto err_free_handler;

	/* Initialize subdev */
	lc898219xi->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
				V4L2_SUBDEV_FL_HAS_EVENTS;
	lc898219xi->sd.internal_ops = &lc898219xi_int_ops;

	ret = media_entity_pads_init(&lc898219xi->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_free_handler;

	lc898219xi->sd.entity.function = MEDIA_ENT_F_LENS;

	pm_runtime_enable(dev);
	ret = v4l2_async_register_subdev(&lc898219xi->sd);

	if (ret < 0) {
		dev_err(dev, "failed to register V4L2 subdev: %d", ret);
		goto err_power_off;
	}

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	dev_info(dev, "Probe done\n");
	return 0;

err_power_off:
	pm_runtime_disable(dev);
	media_entity_cleanup(&lc898219xi->sd.entity);
err_free_handler:
	v4l2_ctrl_handler_free(&lc898219xi->ctrls);

	dev_info(dev, "Probe error\n");

	return ret;
}

static void lc898219xi_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898219xi *lc898219xi = sd_to_lc898219xi(sd);
	struct device *dev = &client->dev;

	v4l2_async_unregister_subdev(&lc898219xi->sd);
	v4l2_ctrl_handler_free(&lc898219xi->ctrls);
	media_entity_cleanup(&lc898219xi->sd.entity);
	pm_runtime_disable(dev);
}

static const struct of_device_id lc898219xi_of_table[] = {
	{ .compatible = "onnn,lc898219xi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lc898219xi_of_table);

static const struct dev_pm_ops lc898219xi_pm_ops = {
	SET_RUNTIME_PM_OPS(lc898219xi_runtime_suspend,
			   lc898219xi_runtime_resume, NULL)
};

static struct i2c_driver lc898219xi_i2c_driver = {
	.driver = {
		.name = LC898219XI_NAME,
		.pm = &lc898219xi_pm_ops,
		.of_match_table = lc898219xi_of_table,
	},
	.probe = lc898219xi_probe,
	.remove = lc898219xi_remove,
};
module_i2c_driver(lc898219xi_i2c_driver);

MODULE_AUTHOR("Frieder Hannenheim <mail@fhannenheim.net>");
MODULE_DESCRIPTION("Onsemi LC898219XI VCM driver");
MODULE_LICENSE("GPL");
