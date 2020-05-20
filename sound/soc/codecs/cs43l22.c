/*
 * ASoC driver for the Cirrus Logic CS43L22 codec.
 *
 * Copyright (c) 2010 Leapfrog Enterprises Inc.
 *
 * Andrey Yurovsky <ayurovsky@leapfrog.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <linux/i2c.h>

#include "cs43l22.h"

#define CODEC_NAME	"CS43L22"

#define CS43L22_FIRSTREG	0x01
#define CS43L22_LASTREG		0x26
#define	CS43L22_NUMREGS		(CS43L22_LASTREG - CS43L22_FIRSTREG + 1)

#define CS43L22_SOC_DOUBLE_R(xname, reg_left, reg_right, xmin, xmax ) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
        .info = cs43l22_snd_soc_info_volsw_2r, \
        .get = cs43l22_snd_soc_get_volsw_2r, \
	.put = cs43l22_snd_soc_put_volsw_2r, \
        .private_value = (unsigned long)&(struct soc_mixer_control) \
                {.reg = reg_left, .rreg = reg_right, .shift = 0, \
                .min = xmin, .max = xmax, .invert = 0} }

struct cs43l22_private {
	enum snd_soc_control_type control_type;
	void *control_data;
	u8 reg_cache[CS43L22_NUMREGS];
	bool manual_mute;
};

static int cs43l22_write_reg(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value)
{
	struct i2c_client *client = codec->control_data;
	u8 *cache = codec->reg_cache;
	struct i2c_msg msg;
	char buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = value & 0xFF;

	msg.addr = CS43L22_ADDR;
	msg.buf = buf;
	msg.len = 2;
	msg.flags = 0; /* write */

	printk(KERN_ERR "cs43l22_write_reg %x %x %x %d\n", client, client->adapter, &msg, client->adapter->retries);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return -EIO;

	cache[reg - CS43L22_FIRSTREG] = value;

	return 0;
}


static unsigned int cs43l22_read_reg(struct i2c_client *client,
		unsigned int reg)
{
//	struct i2c_client *client = codec->control_data;
	struct i2c_msg msg[2];
	char buf[2];
	int ret;

	buf[0] = reg & 0xFF;
	buf[1] = 0;

	msg[0].addr = CS43L22_ADDR;
	msg[0].buf = buf;
	msg[0].len = 1;
	msg[0].flags = 0; /* write */

	msg[1].addr = CS43L22_ADDR;
	msg[1].buf = buf;
	msg[1].len = 2;
	msg[1].flags = I2C_M_RD;

	ret = i2c_transfer(client->adapter, msg, 2);

	return ret < 0 ? ret : buf[1];
}

static unsigned int cs43l22_read_reg_soc(struct snd_soc_codec *codec, unsigned int reg) {
	return cs43l22_read_reg(codec->control_data, reg);
}

static int cs43l22_write_verify_reg(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	struct i2c_client *client = codec->control_data;	
	int ret;
	int retries;
	printk(KERN_ERR "cs43l22 write_verify_reg %x %x %x\n", codec, reg, value);

	for (retries = 0; retries < 5; retries++) {
		cs43l22_write_reg(codec, reg, value);
		ret = cs43l22_read_reg(client, reg);
		if (ret == value)
			return 0;
		else
			dev_err(codec->dev,
		"%s i2c error, retry=%d, reg=%d, value=%d, read=%d\n",
				__FUNCTION__, retries, reg, value, ret); 
	}
	return 1;
}

static int cs43l22_fill_cache(struct snd_soc_codec *codec)
{
	u8 *cache = codec->reg_cache;
	int i, ret;

	for (i = CS43L22_FIRSTREG; i <= CS43L22_LASTREG; i++) {
		ret = cs43l22_read_reg(codec->control_data, i);
		if (ret < 0) {
			struct i2c_client *client = codec->control_data;

			dev_err(codec->dev, "i2c-%X read failure\n",
					client->addr);
			return -EIO;
		}
		*(cache + i) = ret;
	}

	return 0;
}

static int cs43l22_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	return 0;
}

static int cs43l22_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
		unsigned int freq, int dir)
{
	return 0;
}

static int cs43l22_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int format)
{
	return 0;
}

static int cs43l22_dai_mute(struct snd_soc_dai *dai, int mute)
{
	struct cs43l22_private *cs43l22 = i2c_get_clientdata(dai->codec->control_data);
	u8 reg;
       
	reg = cs43l22_read_reg(dai->codec->control_data, CS43L22_PLAYBACK_CONTROL_2) & ~0xF0;
	if (mute || cs43l22->manual_mute)
		reg |= 0xF0;
	cs43l22_write_verify_reg(dai->codec, CS43L22_PLAYBACK_CONTROL_2,
		reg);
	printk(KERN_ERR "Digital Mute %d\n", mute || cs43l22->manual_mute);

	return 0;
}

static int cs43l22_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_codec *codec = rtd->codec;
	
	/* power up amplifier */
	cs43l22_write_verify_reg(codec, CS43L22_POWER_CONTROL_1, 0x9E);
        printk(KERN_ERR "Powered up amplifier\n");	
	return 0;
}

static void cs43l22_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_codec *codec = rtd->codec;

	/* power down amplifier */
	cs43l22_write_verify_reg(codec, CS43L22_POWER_CONTROL_1, 0x9F);
	printk(KERN_ERR "Powered down amplifier\n");
}

static struct snd_soc_dai_ops cs43l22_dai_ops = {
	.startup	= cs43l22_startup,
	.shutdown	= cs43l22_shutdown,
	.hw_params	= cs43l22_hw_params,
	.set_sysclk	= cs43l22_set_dai_sysclk,
	.set_fmt	= cs43l22_set_dai_fmt,
	.digital_mute	= cs43l22_dai_mute,
};

#define CS43L22_FORMATS (SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
		SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S18_3BE | \
		SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
		SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_3BE | \
		SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE)

#define CS43L22_FORMATS_OLD (SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE) 


#define CS43L22_RATES        (SNDRV_PCM_RATE_CONTINUOUS | \
		SNDRV_PCM_RATE_5512 | SNDRV_PCM_RATE_8000 | \
		SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | \
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_88200 | \
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
		SNDRV_PCM_RATE_192000)


struct snd_soc_dai_driver cs43l22_dai = {
	.name		= "cs43l22-hifi",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= CS43L22_RATES,
		.formats	= CS43L22_FORMATS,
	},
	.ops		= &cs43l22_dai_ops,
};
EXPORT_SYMBOL_GPL(cs43l22_dai);

/*
 * cs43l22_snd_soc_info_volsw_2r - double mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a double mixer control that
 * spans 2 codec registers.
 *
 * Returns 0 for success.
 */
int cs43l22_snd_soc_info_volsw_2r(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_info *uinfo)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        int max = mc->max;

        if (max == 1 && !strstr(kcontrol->id.name, " Volume"))
                uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
        else
                uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

        uinfo->count = 2;
        uinfo->value.integer.min = 0;	/* range from min to max is 256 */
        uinfo->value.integer.max = 0xff;
        return 0;
}
EXPORT_SYMBOL_GPL(cs43l22_snd_soc_info_volsw_2r);

/**
 * cs43l22_snd_soc_get_volsw_2r - double mixer get callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to get the value of a double mixer control that spans 2 registers.
 * Map the physical range '.min through .max' to the logical range from 0x00
 * through 0xFF.
 *
 * Returns 0 for success.
 */
int cs43l22_snd_soc_get_volsw_2r(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
        unsigned int reg = mc->reg;
        unsigned int reg2 = mc->rreg;
	int min = mc->min;
	int value;

	/* read physical value and convert to logical range */
	value = snd_soc_read(codec, reg) - min;
	if (value < 0) value += 0x100;
	ucontrol->value.integer.value[0] = value;
	
	value = snd_soc_read(codec, reg2) - min;
	if (value < 0) value += 0x100;
	ucontrol->value.integer.value[1] = value;

        return 0;
}
EXPORT_SYMBOL_GPL(cs43l22_snd_soc_get_volsw_2r);

/**
 * cs43l22_snd_soc_put_volsw_2r - double mixer set callback
 * @kcontrol: mixer control
 * @ucontrol: control element information
 *
 * Callback to set the value of a double mixer control that spans 2 registers.
 *
 * Returns 0 for success.
 */
int cs43l22_snd_soc_put_volsw_2r(struct snd_kcontrol *kcontrol,
        struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
        struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
        unsigned int reg = mc->reg;
        unsigned int reg2 = mc->rreg;
        int min = mc->min;
        int err;
        unsigned short val, val2;

	/* convert logical range to physical range */
	val = ucontrol->value.integer.value[0] + min;
	if (val > 0xFF) val -= 0x100;

	val2 = ucontrol->value.integer.value[1] + min;
	if (val2 > 0xFF) val2 -= 0x100;

        err = snd_soc_update_bits(codec, reg, 0xFF, val);
        if (err < 0)
                return err;

        err = snd_soc_update_bits(codec, reg2, 0xFF, val2);
        return err;
}
EXPORT_SYMBOL_GPL(cs43l22_snd_soc_put_volsw_2r);

static int cs43l22_soc_put_mute(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cs43l22_private *cs43l22 = snd_soc_codec_get_drvdata(codec);
        unsigned int reg = mc->reg;
	int err;

	cs43l22->manual_mute = !ucontrol->value.integer.value[0];
	if (cs43l22->manual_mute)
		err = snd_soc_update_bits(codec, reg, 0xF0, 0xF0);
	else
		err = snd_soc_update_bits(codec, reg, 0xF0, 0x00);

	return err;
}

static const struct snd_kcontrol_new cs43l22_snd_controls[] = {
	CS43L22_SOC_DOUBLE_R("Master Playback Volume", CS43L22_MASTER_VOLUME_A,
			CS43L22_MASTER_VOLUME_B, 0x19, 0x18),
	CS43L22_SOC_DOUBLE_R("Headphone Playback Volume", CS43L22_HEADPHONE_A,
			CS43L22_HEADPHONE_B, 0x01, 0x00),
	CS43L22_SOC_DOUBLE_R("Speaker Playback Volume", CS43L22_SPEAKER_A,
			CS43L22_SPEAKER_B, 0x01, 0x00),
	SOC_SINGLE_EXT("Master Playback Switch", CS43L22_PLAYBACK_CONTROL_2,
			4, 1, 1, snd_soc_get_volsw, cs43l22_soc_put_mute),
};

static int cs43l22_probe(struct snd_soc_codec *codec)
{
        struct cs43l22_private *cs43l22 = snd_soc_codec_get_drvdata(codec);
	int ret, reg;

	BUG_ON(!codec);

	codec->control_data = cs43l22->control_data;
	
	ret = cs43l22_fill_cache(codec);
	if (ret < 0) {
		dev_err(codec->dev, "failed to fill register cache\n");
		return ret;
	}

	//ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	/*
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, cs43l22->control_type);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
	*/

	// TODO: DAC Configuration
	//
		/*
	 * DAC configuration
	 * - Use signal processor
	 * - auto mute
	 * - vol changes immediate
	 * - no de-emphasize
	 */
	/*
	reg = CS43L22_DAC_CTL_DATA_SEL(1)
		| CS43L22_DAC_CTL_AMUTE | CS43L22_DAC_CTL_DACSZ(0);
	ret = snd_soc_write(codec, CS43L22_DAC_CTL, reg);
	if (ret < 0)
		return ret;
        */
        


	ret = snd_soc_add_controls(codec, cs43l22_snd_controls,
			ARRAY_SIZE(cs43l22_snd_controls));
	if (ret < 0) {
		dev_err(codec->dev, "failed to add controls\n");
		//snd_soc_free_pcms(socdev);
		return ret;
	}

	return 0;
}

static struct snd_soc_codec_driver soc_codec_device_cs43l22 = {
	.probe =	cs43l22_probe,
	.reg_cache_size = CS43L22_NUMREGS,
	.reg_word_size = sizeof(u8),
	.write = cs43l22_write_verify_reg,
	.read = cs43l22_read_reg_soc,
};

static int cs43l22_i2c_probe(struct i2c_client *i2c_client,
		const struct i2c_device_id *id)
{
	struct cs43l22_private *cs43l22;
	int ret;
        /*
	codec = &cs43l22->codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
        
	codec->dev = &i2c_client->dev;
	codec->name = CODEC_NAME;
	codec->owner = THIS_MODULE;
	codec->dai = &cs43l22_dai;
	codec->num_dai = 1;
	codec->drvdata = cs43l22;
	codec->control_data = i2c_client;
	codec->read = cs43l22_read_reg;
	codec->write = cs43l22_write_verify_reg;
	codec->reg_cache = cs43l22->reg_cache;
	codec->reg_cache_size = CS43L22_NUMREGS;
        */
	/* Verify that we're talking to a CS43L22 */
	ret = cs43l22_read_reg(i2c_client, CS43L22_CHIPID); //i2c_client, CS43L22_CHIPID);
	//ret = i2c_smbus_read_byte_data(i2c_client, CS43L22_CHIPID);
	if ((ret & 0xF8) != 0xE0) {
		dev_err(&i2c_client->dev, "i2c-%X is not a CS43L22, got %X\n",
				i2c_client->addr, ret);
		ret = -ENODEV;
		goto out_codec;
	}

	dev_info(&i2c_client->dev, "found CS43L22 rev %X at i2c-%X\n",
			ret & 0x7, i2c_client->addr);

        cs43l22 = kzalloc(sizeof(struct cs43l22_private), GFP_KERNEL);
	if (!cs43l22)
		return -ENOMEM;
	i2c_set_clientdata(i2c_client, cs43l22);
	cs43l22->control_data = i2c_client;
	cs43l22->control_type = SND_SOC_I2C;

        /*	
	ret = cs43l22_fill_cache(cs43l22);
	if (ret < 0) {
		kfree(cs43l22);
		dev_err(&i2c_client->dev, "failed to fill register cache\n");
		goto out_codec;
	} */
	

	//cs43l22_dai->dev = &i2c_client->dev;

	/* Register the DAI. If all the other ASoC driver have already
	 * registered , then this will call our probe function, so
	 * cs43l22_codec needs to be ready.
	 */
	//cs43l22_codec = codec;
	//cs43l22 = snd_soc_codec_get_drvdata(codec);
	cs43l22->control_data = i2c_client;
	cs43l22->control_type = SND_SOC_I2C;
	ret = snd_soc_register_codec(&i2c_client->dev, &soc_codec_device_cs43l22, &cs43l22_dai, 1);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "failed to register DAI\n");
		goto out_codec;
	}

	return 0;

out_codec:
	kfree(cs43l22);

	return ret;
}

static int cs43l22_i2c_remove(struct i2c_client *i2c_client)
{
	struct cs43l22_private *cs43l22 = i2c_get_clientdata(i2c_client);
        snd_soc_unregister_codec(&i2c_client->dev);
	kfree(cs43l22);
	return 0;
}

static struct i2c_device_id cs43l22_id[] = {
	{CODEC_NAME, 0x94},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs43l22_id);

/* I2C bus identification */
static struct i2c_driver cs43l22_i2c_driver = {
	.driver = {
		.name	= "cs43l22-codec",
		.owner	= THIS_MODULE,
	},
	.id_table	= cs43l22_id,
	.probe		= cs43l22_i2c_probe,
	.remove		= cs43l22_i2c_remove,
};




static int cs43l22_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	//snd_soc_free_pcms(socdev);

	return 0;
}

static int __init cs43l22_init(void)
{
	return i2c_add_driver(&cs43l22_i2c_driver);
}
module_init(cs43l22_init);

static void __exit cs43l22_exit(void)
{
	i2c_del_driver(&cs43l22_i2c_driver);
}
module_exit(cs43l22_exit);

MODULE_AUTHOR("Andrey Yurovsky <ayurovsky@leapfrog.com>");
MODULE_DESCRIPTION("CS43L22 ALSA SoC Codec Driver");
MODULE_LICENSE("GPL");
