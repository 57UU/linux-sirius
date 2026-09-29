// SPDX-License-Identifier: GPL-2.0-only
/*
 * tas2557.c - Mainline ASoC glue for TI TAS2557 SmartAmp (sirius)
 *
 * DSP/firmware core (tas2557-dsp.c / tas2557-io.c) is TI/CAF code from
 * android_kernel_xiaomi_sdm710 (techpack/audio). This file is the 6.x/7.x
 * "component model" replacement for the downstream tas2557-codec.c glue,
 * which used the removed snd_soc_codec API.
 *
 * v1 notes:
 *  - speaker variant forced by module param (this unit: AAC, spk_id GPIO116
 *    3-state readout not yet ported);
 *  - calibration file load stubbed out (persist is not mounted on mainline),
 *    firmware-default TMax/Re is used;
 *  - IRQ (tlmm 96) not requested yet; faults are visible via debug prints.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>

#include "tas2557.h"
#include "tas2557-core.h"
#include "tas2557-spk-id.h"

static int spk_type = VENDOR_ID_AAC;
module_param(spk_type, int, 0444);
MODULE_PARM_DESC(spk_type, "speaker vendor: 1=AAC (default), 3=GOER");

int spk_id_get_pin_3state(struct device_node *np)
{
	return 0;
}

struct tas2557_main {
	struct tas2557_priv core;
	struct gpio_desc *enable_gpio;
	struct mutex lock; /* serialize enable/disable */
};

static inline struct tas2557_main *to_main(struct tas2557_priv *p)
{
	return container_of(p, struct tas2557_main, core);
}

/* ---- regmap (flat 8/8; book/page switching done by the DSP core) ---- */
static const struct regmap_config tas2557_ml_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
	.max_register = 128,
};

/* ---- ALSA controls ---- */
static int tas2557_vol_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *p = snd_soc_component_get_drvdata(c);
	unsigned char gain = 0;

	tas2557_get_DAC_gain(p, &gain);
	ucontrol->value.integer.value[0] = gain;
	return 0;
}

static int tas2557_vol_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *p = snd_soc_component_get_drvdata(c);
	struct tas2557_main *m = to_main(p);
	unsigned int gain = ucontrol->value.integer.value[0];
	int ret;

	if (gain > 0x0f)
		gain = 0x0f;
	mutex_lock(&m->lock);
	ret = tas2557_set_DAC_gain(p, gain);
	mutex_unlock(&m->lock);
	return ret < 0 ? ret : 0;
}

static int tas2557_pwr_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *p = snd_soc_component_get_drvdata(c);

	ucontrol->value.integer.value[0] = p->mbPowerUp ? 1 : 0;
	return 0;
}

static int tas2557_pwr_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *c = snd_soc_kcontrol_component(kcontrol);
	struct tas2557_priv *p = snd_soc_component_get_drvdata(c);
	struct tas2557_main *m = to_main(p);
	int ret;

	mutex_lock(&m->lock);
	ret = tas2557_enable(p, ucontrol->value.integer.value[0] ? true : false);
	mutex_unlock(&m->lock);
	return ret < 0 ? ret : 0;
}

static const struct snd_kcontrol_new tas2557_ml_controls[] = {
	SOC_SINGLE_EXT("Speaker Driver Playback Volume", SND_SOC_NOPM,
		       0, 0x0f, 0, tas2557_vol_get, tas2557_vol_put),
	SOC_SINGLE_EXT("SmartPA Enable", SND_SOC_NOPM, 0, 1, 0,
		       tas2557_pwr_get, tas2557_pwr_put),
};

/* ---- DAPM ---- */
static const struct snd_soc_dapm_widget tas2557_ml_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASI1 Capture", "ASI1 Capture", 1, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUT_DRV("ClassD", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("NDivider", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("OUT"),
};

static const struct snd_soc_dapm_route tas2557_ml_routes[] = {
	{"DAC", NULL, "ASI1"},
	{"ClassD", NULL, "DAC"},
	{"OUT", NULL, "ClassD"},
	{"DAC", NULL, "PLL"},
	{"DAC", NULL, "NDivider"},
};

/* ---- DAI ops ---- */
static int tas2557_ml_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct tas2557_priv *p = snd_soc_dai_get_drvdata(dai);
	struct tas2557_main *m = to_main(p);
	int ret = 0;

	mutex_lock(&m->lock);
	if (p->mpFirmware && p->mpFirmware->mpPrograms)
		ret = tas2557_set_sampling_rate(p, params_rate(params));
	else
		dev_info(p->dev, "firmware not ready yet, defer config\n");
	mutex_unlock(&m->lock);
	return ret;
}

static int tas2557_ml_mute(struct snd_soc_dai *dai, int mute, int direction)
{
	struct tas2557_priv *p = snd_soc_dai_get_drvdata(dai);
	struct tas2557_main *m = to_main(p);
	int ret;

	mutex_lock(&m->lock);
	ret = tas2557_enable(p, !mute);
	mutex_unlock(&m->lock);
	return ret;
}

static int tas2557_ml_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static int tas2557_ml_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				 unsigned int freq, int dir)
{
	return 0;
}

static const struct snd_soc_dai_ops tas2557_ml_dai_ops = {
	.hw_params	= tas2557_ml_hw_params,
	.mute_stream	= tas2557_ml_mute,
	.set_fmt	= tas2557_ml_set_fmt,
	.set_sysclk	= tas2557_ml_set_sysclk,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver tas2557_ml_dai[] = {
	{
		.name = "tas2557 ASI1",
		.id = 0,
		.playback = {
			.stream_name = "ASI1 Playback",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S20_3LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.capture = {
			.stream_name = "ASI1 Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S20_3LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &tas2557_ml_dai_ops,
		.symmetric_rate = 1,
	},
};

/* ---- component ---- */
static int tas2557_ml_suspend(struct snd_soc_component *component)
{
	struct tas2557_priv *p = snd_soc_component_get_drvdata(component);
	struct tas2557_main *m = to_main(p);

	mutex_lock(&m->lock);
	if (p->mbPowerUp)
		tas2557_enable(p, false);
	mutex_unlock(&m->lock);
	return 0;
}

static const struct snd_soc_component_driver tas2557_ml_component = {
	.controls		= tas2557_ml_controls,
	.num_controls		= ARRAY_SIZE(tas2557_ml_controls),
	.dapm_widgets		= tas2557_ml_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tas2557_ml_widgets),
	.dapm_routes		= tas2557_ml_routes,
	.num_dapm_routes	= ARRAY_SIZE(tas2557_ml_routes),
	.suspend		= tas2557_ml_suspend,
	.idle_bias_on		= 1,
	.endianness		= 1,
};

/* ---- i2c probe ---- */
static int tas2557_ml_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct tas2557_main *m;
	struct tas2557_priv *p;
	const char *fwname;
	unsigned int pgid = 0;
	int ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	p = &m->core;
	mutex_init(&m->lock);
	mutex_init(&p->dev_lock);
	p->dev = dev;
	i2c_set_clientdata(client, p);

	m->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(m->enable_gpio))
		return PTR_ERR(m->enable_gpio);

	p->mpRegmap = devm_regmap_init_i2c(client, &tas2557_ml_regmap);
	if (IS_ERR(p->mpRegmap))
		return PTR_ERR(p->mpRegmap);

	p->read = tas2557_dev_read;
	p->write = tas2557_dev_write;
	p->bulk_read = tas2557_dev_bulk_read;
	p->bulk_write = tas2557_dev_bulk_write;
	p->update_bits = tas2557_dev_update_bits;
	p->set_config = tas2557_set_config;
	p->set_calibration = tas2557_set_calibration;
	p->mnCurrentBook = 0xff;
	p->mnCurrentPage = 0xff;
	p->mnResetGPIO = -1;
	p->mnGpioINT = -1;

	/* reset pulse (same timing as downstream hw_reset) */
	if (m->enable_gpio) {
		gpiod_set_value_cansleep(m->enable_gpio, 0);
		msleep(5);
		gpiod_set_value_cansleep(m->enable_gpio, 1);
		msleep(2);
	}

	/* software reset + sanity read */
	ret = p->write(p, TAS2557_SW_RESET_REG, 0x01);
	if (ret < 0) {
		dev_err(dev, "no response from TAS2557: %d\n", ret);
		return ret;
	}
	msleep(1);
	ret = p->read(p, TAS2557_REV_PGID_REG, &pgid);
	if (ret < 0)
		return ret;
	p->mnPGID = pgid;
	dev_info(dev, "TAS2557 PGID=0x%02x\n", pgid);
	if (pgid != TAS2557_PG_VERSION_1P0 &&
	    pgid != TAS2557_PG_VERSION_2P0 &&
	    pgid != TAS2557_PG_VERSION_2P1)
		dev_warn(dev, "unexpected silicon rev, trying anyway\n");

	if (spk_type == VENDOR_ID_GOER) {
		p->mnSpkType = VENDOR_ID_GOER;
		fwname = TAS2557_GOER_FW_NAME;
	} else {
		p->mnSpkType = VENDOR_ID_AAC;
		fwname = TAS2557_AAC_FW_NAME;
	}

	p->mpFirmware = devm_kzalloc(dev, sizeof(struct TFirmware), GFP_KERNEL);
	p->mpCalFirmware = devm_kzalloc(dev, sizeof(struct TFirmware), GFP_KERNEL);
	if (!p->mpFirmware || !p->mpCalFirmware)
		return -ENOMEM;

	hrtimer_init(&p->mtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	p->mtimer.function = temperature_timer_func;
	INIT_WORK(&p->mtimerwork, timer_work_routine);

	dev_set_drvdata(dev, p);
	ret = devm_snd_soc_register_component(dev, &tas2557_ml_component,
					      tas2557_ml_dai,
					      ARRAY_SIZE(tas2557_ml_dai));
	if (ret)
		return ret;

	dev_info(dev, "requesting firmware %s\n", fwname);
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT, fwname,
				       dev, GFP_KERNEL, p, tas2557_fw_ready);
}

static void tas2557_ml_remove(struct i2c_client *client)
{
	struct tas2557_priv *p = i2c_get_clientdata(client);

	if (p->mbPowerUp)
		tas2557_enable(p, false);
	hrtimer_cancel(&p->mtimer);
	cancel_work_sync(&p->mtimerwork);
}

static const struct i2c_device_id tas2557_ml_id[] = {
	{ "tas2557", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tas2557_ml_id);

static const struct of_device_id tas2557_ml_of_match[] = {
	{ .compatible = "ti,tas2557" },
	{ }
};
MODULE_DEVICE_TABLE(of, tas2557_ml_of_match);

static struct i2c_driver tas2557_ml_driver = {
	.driver = {
		.name = "tas2557",
		.of_match_table = tas2557_ml_of_match,
	},
	.probe = tas2557_ml_probe,
	.remove = tas2557_ml_remove,
	.id_table = tas2557_ml_id,
};
module_i2c_driver(tas2557_ml_driver);

MODULE_AUTHOR("TI, ported for sirius mainline");
MODULE_DESCRIPTION("TAS2557 SmartAmp ASoC driver (sirius mainline glue)");
MODULE_LICENSE("GPL v2");
