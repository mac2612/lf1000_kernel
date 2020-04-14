/*
 * linux/sound/soc/lf1000/didj-lfp100.c
 *
 * ALSA Machine driver for the LeapFrog LF1000 Didj style game console.
 * Supports the audio codec part of the LeapFrog LFP100 power/codec chip,
 * connected to the LF1000 SoC's I2S controller.
 *
 * Authors: Scott Esters <sesters@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation. 
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>

#include <sound/core.h>
#include <sound/soc.h>
#include <sound/pcm.h>

#include <mach/gpio.h>

#include "lf1000-pcm.h"
#include "lf1000-i2s.h"
#include "didj-lfp100.h"
#include "../codecs/lfp100.h"

#define DIDJ_DEFAULT_RATE	32000

static struct platform_device *didj_snd_device = NULL;

/*
 * Board ops
 */

static int didj_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, DIDJ_DEFAULT_RATE, 1);
	if (ret)
		return ret;

	return 0;
}

static int didj_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

	return snd_soc_dai_set_sysclk(cpu_dai, 0, params_rate(params), 1);

	return 0;
}

static struct snd_soc_ops didj_ops = {
	.startup	= didj_startup,
	.hw_params	= didj_hw_params,
};

/* didj digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link didj_dai_lfp100[] = {
	{
		.name = "LFP100",
		.stream_name = "LFP100",
		.cpu_dai = &lf1000_i2s_dai,
		.codec_dai = &lfp100_dai,
		.ops = &didj_ops,
	},
};

/* didj audio machine driver */
static struct snd_soc_card snd_soc_didj_lfp100 = {
	.name = "Didj-LFP100",
	.platform = &lf1000_soc_platform,
	.dai_link = didj_dai_lfp100,
	.num_links = ARRAY_SIZE(didj_dai_lfp100),
};

/* didj audio private data */
static struct lfp100_setup_data didj_lfp100_setup = {
	.i2c_bus	= 0,
	.i2c_address	= LFP100_ADDR,
};

/* didj audio subsystem */
static struct snd_soc_device didj_snd_devdata_lfp100 = {
	.card = &snd_soc_didj_lfp100,
	.codec_dev = &soc_codec_dev_lfp100,
	.codec_data = &didj_lfp100_setup,
	.dev = NULL, /* set in didj_audio_probe */
};

/*
 * sysfs interface
 */

static ssize_t show_force_audio(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct lfp100_private *lfp100 = didj_dai_lfp100->codec_dai->codec->private_data;

        return(sprintf(buf,"%u\n", lfp100->force_audio));
}

static ssize_t set_force_audio(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	struct lfp100_private *lfp100 = didj_dai_lfp100->codec_dai->codec->private_data;
        unsigned int value;

        if (sscanf(buf, "%x", &value) != 1)
                return -EINVAL;

        lfp100->force_audio = value;
        return count;
}

static DEVICE_ATTR(force_audio, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
        show_force_audio, set_force_audio);

static struct attribute *lfp100_attributes[] = {
        &dev_attr_force_audio.attr,
        NULL
};

static struct attribute_group lfp100_attr_group = {
	.attrs = lfp100_attributes
};

static int didj_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_codec *codec;
	int ret;
	int i;

	dev_info(&pdev->dev, "%s.%d\n", __FUNCTION__, __LINE__);

	didj_snd_device = platform_device_alloc("soc-audio", -1);
	if (!didj_snd_device)
		return -ENOMEM;

	platform_set_drvdata(didj_snd_device, &didj_snd_devdata_lfp100);
	didj_snd_devdata_lfp100.dev = &didj_snd_device->dev;
	
	ret = platform_device_add(didj_snd_device);
	if (ret) {
		dev_err(&pdev->dev, "can't add sound device\n");
		platform_device_put(didj_snd_device);
		return ret;
	}

	/* program codec defaults */
	codec = didj_snd_devdata_lfp100.card->codec;
	for (i = 0; i < ARRAY_SIZE(lfp100_settings); i++) {
		codec->write(codec, lfp100_settings[i][0],
			lfp100_settings[i][1]);
	}

	sysfs_create_group(&pdev->dev.kobj, &lfp100_attr_group);
	return 0;
}

static int __devexit didj_audio_remove(struct platform_device *pdev)
{
	platform_device_unregister(didj_snd_device);
	sysfs_remove_group(&pdev->dev.kobj, &lfp100_attr_group);
	didj_snd_device = NULL;

	return 0;
}

static struct platform_driver didj_audio_driver = {
	.probe	= didj_audio_probe,
	.remove	= didj_audio_remove,
	.driver	= {
		.name	= "didj-asoc",
		.owner	= THIS_MODULE,
	},
};

static int __init didj_audio_init(void)
{
	return platform_driver_register(&didj_audio_driver);
}
module_init(didj_audio_init);

static void __exit didj_audio_exit(void)
{
	platform_driver_unregister(&didj_audio_driver);
}
module_exit(didj_audio_exit);

MODULE_AUTHOR("Scott Esters <sesters@leapfrog.com>");
MODULE_AUTHOR("Andrey Yurovsky <ayurovsky@leapfrog.com>");
MODULE_DESCRIPTION("ALSA SoC Didj Consoles");
MODULE_LICENSE("GPL");
