/*
 * ASoC driver for the codec part of the LeapFrog LFP100 codec/power chip.
 *
 * Copyright (c) 2011 Leapfrog Enterprises Inc.
 *
 * Scott Esters <sesters@leapfrog.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/i2c.h>
#include "lfp100.h"
#include "../lf1000/lf1000-pcm.h"

#include <mach/lfp100.h>

#define CODEC_NAME	"lfp100-codec"

static struct snd_soc_codec *lfp100_codec = NULL;

/* match up with sound system interface for read and write */
unsigned int lfp100_codec_read_reg(struct snd_soc_codec *codec,
		unsigned int reg)
{
	return lfp100_read_reg(reg);
}

int lfp100_codec_write_reg(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	return lfp100_write_reg(reg, value);
}

int lfp100_mute(void)
{
	u8 volume_reg;
	u8 mgain_reg;
	u8 a_control_reg;


	/* strip mute and dac bits */
	a_control_reg = lfp100_read_reg(LFP100_A_CONTROL) &
		(~(LFP100_A_CONTROL_DAC_SW |
		   LFP100_A_CONTROL_DAC_EN |
		   LFP100_A_CONTROL_AUTO_MASK)
		);
	volume_reg    =	/* unmute digital volume */
		lfp100_read_reg(LFP100_VOLUME) & (~LFP100_VOLUME_MUTE);
	mgain_reg     = /* unmute analog gain */
		lfp100_read_reg(LFP100_MGAIN) & (~LFP100_MGAIN_AMUTE);

	/* disable Auto Route */ 
	lfp100_write_reg(LFP100_A_CONTROL, a_control_reg);

	volume_reg |= LFP100_VOLUME_MUTE;
	lfp100_write_reg(LFP100_VOLUME, volume_reg);

	mgain_reg |= LFP100_MGAIN_AMUTE;
	lfp100_write_reg(LFP100_MGAIN, mgain_reg);

	/* disable HP and SPK amplifier */
	a_control_reg &= (~(LFP100_A_CONTROL_SPK_EN |
				LFP100_A_CONTROL_HP_EN)
			);
	lfp100_write_reg(LFP100_A_CONTROL, a_control_reg);
	return 0;
}

int lfp100_unmute(void)
{
	u8 volume_reg;
	u8 mgain_reg;
	u8 a_control_reg;

	volume_reg    =	/* unmute digital volume */
		lfp100_read_reg(LFP100_VOLUME) & (~LFP100_VOLUME_MUTE);
	lfp100_write_reg(LFP100_VOLUME, volume_reg);

	mgain_reg     = /* unmute analog gain */
		lfp100_read_reg(LFP100_MGAIN) & (~LFP100_MGAIN_AMUTE);
	lfp100_write_reg(LFP100_MGAIN, mgain_reg);

	/* Enable DAC and AUTO-ROUTE if needed */
	a_control_reg = lfp100_read_reg(LFP100_A_CONTROL);
	if (!(a_control_reg & LFP100_A_CONTROL_DAC_SW) ||
	    !(a_control_reg & LFP100_A_CONTROL_DAC_EN) ||
	    !(a_control_reg & LFP100_A_CONTROL_AUTO_ROUTE)) {

		a_control_reg &= (~(LFP100_A_CONTROL_DAC_SW |
				    LFP100_A_CONTROL_DAC_EN |
				    LFP100_A_CONTROL_AUTO_MASK |
				    LFP100_A_CONTROL_SPK_EN |
				    LFP100_A_CONTROL_HP_EN));
		a_control_reg |= LFP100_A_CONTROL_DAC_EN;
		lfp100_write_reg(LFP100_A_CONTROL, a_control_reg);

		a_control_reg |= LFP100_A_CONTROL_DAC_SW;
		lfp100_write_reg(LFP100_A_CONTROL, a_control_reg);

		a_control_reg |= LFP100_A_CONTROL_AUTO_ROUTE;
		lfp100_write_reg(LFP100_A_CONTROL, a_control_reg);
		lfp100_unmute_hp_sp();
	}
	return 0;
}

int lfp100_unmute_headphones_only(void)
{
	u8 volume_reg;
	u8 mgain_reg;
	u8 a_control_reg;

	volume_reg    =	/* unmute digital volume */
		lfp100_read_reg(LFP100_VOLUME) & (~LFP100_VOLUME_MUTE);
	lfp100_write_reg(LFP100_VOLUME, volume_reg);

	mgain_reg     = /* unmute analog gain */
		lfp100_read_reg(LFP100_MGAIN) & (~LFP100_MGAIN_AMUTE);
	lfp100_write_reg(LFP100_MGAIN, mgain_reg);

	/* Enable headphone DAC */
	a_control_reg = lfp100_read_reg(LFP100_A_CONTROL);
	if (!(a_control_reg & LFP100_A_CONTROL_DAC_SW) ||
	    !(a_control_reg & LFP100_A_CONTROL_DAC_EN) ||
	     (a_control_reg & LFP100_A_CONTROL_AUTO_ROUTE) ||
	    !(a_control_reg & LFP100_A_CONTROL_HP_EN)) {

		a_control_reg &= (~(LFP100_A_CONTROL_DAC_SW |
				    LFP100_A_CONTROL_DAC_EN |
				    LFP100_A_CONTROL_AUTO_MASK |
				    LFP100_A_CONTROL_SPK_EN |
				    LFP100_A_CONTROL_HP_EN));

		a_control_reg |= LFP100_A_CONTROL_DAC_EN;
		lfp100_write_reg(LFP100_A_CONTROL, a_control_reg);

		a_control_reg |= LFP100_A_CONTROL_DAC_SW;
		lfp100_write_reg(LFP100_A_CONTROL, a_control_reg);

		a_control_reg |= LFP100_A_CONTROL_HP_EN;
		lfp100_write_reg(LFP100_A_CONTROL, a_control_reg);
	}
	return 0;
}

static int lfp100_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	return 0;
}

static int lfp100_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
		unsigned int freq, int dir)
{
	return 0;
}

static int lfp100_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int format)
{
	return 0;
}

static int lfp100_dai_mute(struct snd_soc_dai *dai, int mute)
{
	struct lfp100_private *lfp100 = dai->codec->private_data;

	/* set bits as needed */
	if (mute || lfp100->manual_mute) {
		lfp100_mute();
	} else {	/* unmute audio */
		switch(lfp100->force_audio) {
		case AUDIO_NORMAL:
			lfp100_unmute();
			break;
		case AUDIO_HEADPHONES_ONLY:
			lfp100_unmute_headphones_only();
			break;
		default:
			break;
		}
	}

	return 0;
}

static int lfp100_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return 0;
}

static void lfp100_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return;
}

static struct snd_soc_dai_ops lfp100_dai_ops = {
	.startup	= lfp100_startup,
	.shutdown	= lfp100_shutdown,
	.hw_params	= lfp100_hw_params,
	.set_sysclk	= lfp100_set_dai_sysclk,
	.set_fmt	= lfp100_set_dai_fmt,
	.digital_mute	= lfp100_dai_mute,
};

#define LFP100_FORMATS (SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
		SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S18_3BE | \
		SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
		SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_3BE | \
		SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE)

#define LFP100_RATES        (SNDRV_PCM_RATE_CONTINUOUS | \
		SNDRV_PCM_RATE_5512 | SNDRV_PCM_RATE_8000 | \
		SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | \
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_88200 | \
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
		SNDRV_PCM_RATE_192000)


struct snd_soc_dai lfp100_dai = {
	.name		= CODEC_NAME,
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= LFP100_RATES,
		.formats	= LFP100_FORMATS,
	},
	.ops		= &lfp100_dai_ops,
};
EXPORT_SYMBOL_GPL(lfp100_dai);

static int lfp100_i2c_probe(struct i2c_client *i2c_client,
		const struct i2c_device_id *id)
{
	struct snd_soc_codec *codec;
	struct lfp100_private *lfp100;
	int ret;

	dev_info(&i2c_client->dev, "i2c-0x%X %s\n", i2c_client->addr,
			__FUNCTION__);

	if (lfp100_codec)
		return -ENODEV;

	lfp100 = kzalloc(sizeof(struct lfp100_private), GFP_KERNEL);
	if (!lfp100)
		return -ENOMEM;
	codec = &lfp100->codec;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->dev = &i2c_client->dev;
	codec->name = CODEC_NAME;
	codec->owner = THIS_MODULE;
	codec->dai = &lfp100_dai;
	codec->num_dai = 1;
	codec->private_data = lfp100;
	codec->control_data = i2c_client;
	codec->read = lfp100_codec_read_reg;
	codec->write = lfp100_codec_write_reg;
	codec->reg_cache = NULL;
	codec->reg_cache_size = LFP100_NUMREGS;

	if (!lfp100_have_lfp100()) {
		dev_err(&i2c_client->dev, "i2c-%X is not an LFP100\n",
				i2c_client->addr);
		ret = -ENODEV;
		goto out_codec;
	}

	lfp100_dai.dev = &i2c_client->dev;

	/* Register the DAI. If all the other ASoC driver have already
	 * registered , then this will call our probe function, so
	 * lfp100_codec needs to be ready.
	 */
	lfp100_codec = codec;
	ret = snd_soc_register_dai(&lfp100_dai);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "failed to register DAI\n");
		goto out_codec;
	}

	i2c_set_clientdata(i2c_client, lfp100);

	return 0;

out_codec:
	kfree(lfp100);
	lfp100_codec = NULL;
	lfp100_dai.dev = NULL;

	return ret;
}

static int lfp100_i2c_remove(struct i2c_client *i2c_client)
{
	struct lfp100_private *lfp100 = i2c_get_clientdata(i2c_client);

	kfree(lfp100);
	lfp100_codec = NULL;
	lfp100_dai.dev = NULL;
	
	return 0;
}

static struct i2c_device_id lfp100_id[] = {
	{CODEC_NAME, LFP100_ADDR},
	{}
};
MODULE_DEVICE_TABLE(i2c, lfp100_id);

/* I2C bus identification */
static struct i2c_driver lfp100_i2c_driver = {
	.driver = {
		.name	= CODEC_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table	= lfp100_id,
	.probe		= lfp100_i2c_probe,
	.remove		= lfp100_i2c_remove,
};
int lfp100_soc_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	int value;

	/* read physical value and convert to logical range */
	value = snd_soc_read(codec, reg);
	if (value == 0) {
		value = 0x1F;
	} else {
		value -= 0x21;
	}
	ucontrol->value.integer.value[0] = value;
	return 0;
}

static int lfp100_soc_put_volsw(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned short val;

	/*
	 * expect values between 0 and 0x1F and map
	 * to 0x21, 0x22, 0x23 ... 0x3E, 0x3F, 0x00
	 */
	val = ucontrol->value.integer.value[0];
	
	if (0x1F <= val) {
		/* 0x1F -> 0x00 */
		val = 0;
	} else {
		val += 0x21;
	}
	return snd_soc_update_bits(codec, reg, 0x3F, val);
}

static int lfp100_soc_put_mute(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct lfp100_private *lfp100 = codec->private_data;

	lfp100->manual_mute = !ucontrol->value.integer.value[0];

	if (lfp100->manual_mute) {
		lfp100_mute();
	} else {	/* unmute audio */
		switch(lfp100->force_audio) {
		case AUDIO_NORMAL:
			lfp100_unmute();
			break;
		case AUDIO_HEADPHONES_ONLY:
			lfp100_unmute_headphones_only();
			break;
		default:
			break;
		}
	}
	return 0;
}

static const struct snd_kcontrol_new lfp100_snd_controls[] = {
	SOC_SINGLE_EXT("Master Playback Volume", LFP100_VOLUME, 0, 0x1F, 0,
	lfp100_soc_get_volsw, lfp100_soc_put_volsw),
	SOC_SINGLE_EXT("Master Playback Switch", LFP100_VOLUME, 6, 1, 1,
	snd_soc_get_volsw, lfp100_soc_put_mute),
};

static int lfp100_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = lfp100_codec;
	int ret;

	BUG_ON(!socdev);
	BUG_ON(!codec);

	/* connect the codec to the socdev */
	socdev->card->codec = codec;

	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms\n");
		return ret;
	}

	ret = snd_soc_add_controls(codec, lfp100_snd_controls,
			ARRAY_SIZE(lfp100_snd_controls));
	if (ret < 0) {
		dev_err(codec->dev, "failed to add controls\n");
		snd_soc_free_pcms(socdev);
		return ret;
	}

	return 0;
}

static int lfp100_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);

	return 0;
}

/* Assign this to the codec_dev field of the machine driver's snd_soc_device
 * structure */
struct snd_soc_codec_device soc_codec_dev_lfp100 = {
	.probe	= lfp100_probe,
	.remove	= lfp100_remove,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_lfp100);

static int __init lfp100_init(void)
{
	return i2c_add_driver(&lfp100_i2c_driver);
}
module_init(lfp100_init);

static void __exit lfp100_exit(void)
{
	i2c_del_driver(&lfp100_i2c_driver);
}
module_exit(lfp100_exit);

MODULE_AUTHOR("Scott Esters <sesters@leapfrog.com>");
MODULE_DESCRIPTION("LFP100 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
