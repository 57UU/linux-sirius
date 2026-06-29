// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (c) 2025 Vasiliy Doylov <nekocwd@mainlining.org>
// Copyright (c) 2025 Frieder Hannenheim <git@fhannenheim.net>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/math.h>
#include <media/v4l2-async.h>
#include <media/v4l2-cci.h>
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
#define LC898219XI_DAC_ADDR CCI_REG16(0x84)

static const char *const lc898219xi_supply_names[] = {
	"vdd",
	"vio",
	"vana",
};

struct lc898219xi {
	struct regulator_bulk_data supplies[ARRAY_SIZE(lc898219xi_supply_names)];
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct regmap *regmap;
};

static inline struct lc898219xi *sd_to_lc898219xi(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct lc898219xi, sd);
}

static int lc898219xi_set_dac(struct lc898219xi *lc898219xi, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&lc898219xi->sd);
	int ret;

	// Value ranges from -2047 (max focus) to 2048 in hardware
	s16 actuator_val = ((s16)val - 2048) * -1;

	// Focus driver expects a 2s complement 12 bit signed integer
	u16 actuator_val_s12 = (actuator_val >> 4) & LC898219XI_MAX_FOCUS_POS;

	ret = cci_write(lc898219xi->regmap, LC898219XI_DAC_ADDR,
			cpu_to_le16(actuator_val_s12), NULL);
	if (ret)
		dev_err(&client->dev, "failed to set DAC: %d\n", ret);

	return ret;
}

static int lc898219xi_power_on(struct lc898219xi *lc898219xi)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&lc898219xi->sd);

	ret = regulator_bulk_enable(ARRAY_SIZE(lc898219xi_supply_names),
				    lc898219xi->supplies);

	if (ret < 0) {
		dev_err(&client->dev, "failed to enable regulators\n");
		return ret;
	}

	usleep_range(8000, 10000);

	uint32_t regdata = i2c_smbus_read_byte_data(client, 0xF0);
	if (regdata != 0xA5) {
		dev_err(&client->dev, "communication error: regdata: %x \n",
		        regdata);
		return -1;
	}

	usleep_range(1000, 1010);

	i2c_smbus_write_byte_data(client, 0xE0, 0x01);
	msleep(8);

	int retry;
	for (retry = 0; retry < 10; retry++) {
		uint32_t check = i2c_smbus_read_byte_data(client, 0xB3);
		if ((check & 0XE0) == 0) {
			break;
		} else if (retry >= 9) {
			dev_err(&client->dev, "LSI wake up check failed");
			return -1;
		}
		usleep_range(1000, 1010);
	}

	i2c_smbus_write_byte_data(client, 0x8C, 0xE9);

	return ret;
}

static int lc898219xi_power_off(struct lc898219xi *lc898219xi)
{
	regulator_bulk_disable(ARRAY_SIZE(lc898219xi_supply_names),
			       lc898219xi->supplies);
	return 0;
}

static int __maybe_unused lc898219xi_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898219xi *lc898219xi = sd_to_lc898219xi(sd);

	lc898219xi_power_off(lc898219xi);
	return 0;
}

static int __maybe_unused lc898219xi_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898219xi *lc898219xi = sd_to_lc898219xi(sd);
	int ret;

	ret = lc898219xi_power_on(lc898219xi);
	if (ret < 0) {
		dev_err(dev, "power on failed\n");
		return ret;
	}

	return ret;
}

static int lc898219xi_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898219xi *lc898219xi = container_of(ctrl->handler,
						     struct lc898219xi, ctrls);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return lc898219xi_set_dac(lc898219xi, ctrl->val);

	return 0;
}

static const struct v4l2_ctrl_ops lc898219xi_ctrl_ops = {
	.s_ctrl = lc898219xi_set_ctrl,
};

static int lc898219xi_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct lc898219xi *lc898219xi = sd_to_lc898219xi(sd);
	__v4l2_ctrl_handler_setup(&lc898219xi->ctrls);

	return pm_runtime_resume_and_get(sd->dev);
}

static int lc898219xi_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put_autosuspend(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops lc898219xi_int_ops = {
	.open = lc898219xi_open,
	.close = lc898219xi_close,
};

static const struct v4l2_subdev_core_ops lc898219xi_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
};

static const struct v4l2_subdev_ops lc898219xi_ops = {
	.core = &lc898219xi_core_ops,
};

static int lc898219xi_init_controls(struct lc898219xi *lc898219xi)
{
	struct v4l2_ctrl_handler *hdl = &lc898219xi->ctrls;
	const struct v4l2_ctrl_ops *ops = &lc898219xi_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  LC898219XI_MIN_FOCUS_POS, LC898219XI_MAX_FOCUS_POS,
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

	lc898219xi = devm_kzalloc(dev, sizeof(*lc898219xi), GFP_KERNEL);
	if (!lc898219xi)
		return -ENOMEM;

	lc898219xi->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(lc898219xi->regmap))
		return dev_err_probe(dev, PTR_ERR(lc898219xi->regmap),
				     "failed to initialize CCI\n");

	/* Initialize subdev */
	v4l2_i2c_subdev_init(&lc898219xi->sd, client, &lc898219xi_ops);

	for (i = 0; i < ARRAY_SIZE(lc898219xi_supply_names); i++)
		lc898219xi->supplies[i].supply = lc898219xi_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(lc898219xi_supply_names),
				      lc898219xi->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = lc898219xi_power_on(lc898219xi);
	if (ret)
		return dev_err_probe(dev, ret, "power on failed\n");

	ret = lc898219xi_init_controls(lc898219xi);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init v4l2 controls\n");
		goto err_power_off;
	}

	/* Initialize subdev */
	lc898219xi->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	lc898219xi->sd.internal_ops = &lc898219xi_int_ops;

	ret = media_entity_pads_init(&lc898219xi->sd.entity, 0, NULL);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to init media entity pads");
		goto err_free_handler;
	}

	lc898219xi->sd.entity.function = MEDIA_ENT_F_LENS;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev(&lc898219xi->sd);
	if (ret < 0) {
		dev_err_probe(dev, ret, "failed to register V4L2 subdev\n");
		goto err_pm;
	}

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	media_entity_cleanup(&lc898219xi->sd.entity);
err_free_handler:
	v4l2_ctrl_handler_free(&lc898219xi->ctrls);
err_power_off:
	lc898219xi_power_off(lc898219xi);
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

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		lc898219xi_power_off(lc898219xi);
	pm_runtime_set_suspended(dev);
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

MODULE_AUTHOR("Frieder Hannenheim <git@fhannenheim.net>");
MODULE_DESCRIPTION("Onsemi LC898219XI VCM driver");
MODULE_LICENSE("GPL");
