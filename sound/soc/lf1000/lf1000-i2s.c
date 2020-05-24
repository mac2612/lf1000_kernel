/* sound/soc/lf1000/lf1000-i2s.c
 *
 * ALSA SoC Audio Layer - LF1000 I2S driver
 *
 * Copyright (c) 2010 LeapFrog Enterprises Inc.
 * 	Andrey Yurovsky <ayurovsky@leapfrog.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/platform.h>
#include <mach/gpio.h>

#include "lf1000-pcm.h"

#define DRIVER_NAME	"lf1000-i2s"

#ifdef CONFIG_SND_LF1000_SOC_DEBUG
#define dbg(x...)	printk(KERN_ALERT DRIVER_NAME ": " x)
#else
#define dbg(x...)
#endif

#define LF1000_I2S_RATES	(SNDRV_PCM_RATE_CONTINUOUS | \
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | \
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_88200 | \
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 | \
		SNDRV_PCM_RATE_192000)


/* registers */
#define I2S_CTRL			0x04
#define I2S_CONFIG			0x06
#define AUDIO_BUFF_CTRL			0x08
#define AUDIO_BUFF_CONFIG		0x0A
#define AUDIO_IRQ_ENA			0x0C
#define AUDIO_IRQ_PEND			0x0E
#define AUDIO_STATUS1			0x18
#define CLKENB				0x3C0	/* u32 */
#define CLKGEN0				0x3C4
#define CLKGEN1				0x3C8

/* I2S control register (I2S_CTRL) */
#define I2SLINK_RUN			(0x1 << 1)
#define I2S_EN				(0x1)
#define I2S_RESET			(0)

/* I2S config register (I2S_CONFIG) */
#define IFMODE_I2S			(0x0)
#define IFMODE_LJ			(0x2 << 6)
#define IFMODE_RJ			(0x3 << 6)
#define IFMODE_MASK			(0x3 << 6)
#define SYNC_32FS			(0x0)
#define SYNC_48FS			(0x1 << 4)
#define SYNC_64FS			(0x2 << 4)
#define SYNC_MASK			(0x3 << 4)
#define LOOP_BACK			(0x1 << 3)
#define I2SI_EN				(0x1 << 2)
#define I2SO_EN				(0x1 << 1)
#define CTL_MST				(0)
#define CTL_SLV				(0x1)

/* Buffer control register (AUDIO_BUFF_CTRL) */
#define PCMIBUF_EN			(0x1 << 1)
#define PCMOBUF_EN			(0x1)

/* Buffer config register (AUDIO_BUFF_CONFIG) */
#define PI_WIDTH_MASK			(0x3 << 4)
#define PO_WIDTH_MASK			(0x3)

#define BUF_WIDTH_16BIT			(0x0)
#define BUF_WIDTH_18BIT			(0x3)

/* IRQ enable/status register (AUDIO_IRQ_ENA/AUDIO_IRQ_PEND) */
#define IRQ_PIOVER			(0x1 << 1)
#define IRQ_POUDR			(0x1)

/* CLKENB register (I2S_CLKENB) */
#define PCLKMODE_DYNAMIC	(0)
#define PCLKMODE_ALWAYS		(0x1 << 3)
#define CLKGENENB		(0x1 << 2)

/* CLKGEN0 register */
#define CLKDIV_MASK		(0x3F << 4)
#define set_clk_div(reg, x)	(reg |= ((x - 1) << 4))
#define clear_clk_div(reg)	(reg &= ~CLKDIV_MASK)
#define CLKSRC_PLL0		(0)
#define CLKSRC_PLL1		(0x1 << 1)
#define CLKSRC_BCLK		(0x3 << 1)
#define CLKSRC_IBCLK		(0x4 << 1)
#define CLKSRC_AVCLK		(0x5 << 1)
#define CLKSRC_IAVCLK		(0x6 << 1)

#define CLKSRC_MASK		(0x7 << 1)
        
#define OUTCLKINV		(0x1 << 0)
#define OUTCLKENB		(0x1 << 15)     // 0: Output, 1: Input

/* CLKGEN1 register */
#define CLKSRC_CLKGEN0		(0x7 << 1) 

struct i2s_snd_dev {
        struct i2s_plat_data      *pdata;
        struct snd_soc_dai_driver *pdrv;
};

const static struct lf1000_pcm_dma_params lf1000_i2s_pcm_stereo_out = {
	.name		= "I2S PCM Stereo out",
	.dma_addr	= LF1000_AUDIO_BASE,
	.dma_size	= 4,
};

const static struct lf1000_pcm_dma_params lf1000_i2s_pcm_stereo_in = {
	.name		= "I2S PCM Stereo in",
	.dma_addr	= LF1000_AUDIO_BASE,
	.dma_size	= 4,
};

struct lf1000_i2s_info {
	void __iomem *adi_base;
	u32 rate;
	u32 div;
	unsigned short	bitwidth;
	unsigned int i2s_format;
	u32 underruns;
	u32 overruns;

	int irq;

	struct dentry	*debug;
};

static struct lf1000_i2s_info lf1000_i2s;

static void lf1000_i2s_show_u16(struct seq_file *s, const char *nm, u32 reg)
{	
	struct lf1000_i2s_info *lf1000_i2s = s->private;

	seq_printf(s, "%17s:  0x%04X\n", nm, readw(lf1000_i2s->adi_base + reg));
}

static void lf1000_i2s_show_u32(struct seq_file *s, const char *nm, u32 reg)
{	
	struct lf1000_i2s_info *lf1000_i2s = s->private;

	seq_printf(s, "%17s:  0x%08X\n", nm, readl(lf1000_i2s->adi_base + reg));
}

static int lf1000_i2s_regs_show(struct seq_file *s, void *v)
{
	lf1000_i2s_show_u16(s, "I2S_CTRL", I2S_CTRL);
	lf1000_i2s_show_u16(s, "I2S_CONFIG", I2S_CONFIG);
	lf1000_i2s_show_u16(s, "AUDIO_BUFF_CTRL", AUDIO_BUFF_CTRL);
	lf1000_i2s_show_u16(s, "AUDIO_BUFF_CONFIG", AUDIO_BUFF_CONFIG);
	lf1000_i2s_show_u16(s, "AUDIO_IRQ_ENA", AUDIO_IRQ_ENA);
	lf1000_i2s_show_u16(s, "AUDIO_IRQ_PEND", AUDIO_IRQ_PEND);
	lf1000_i2s_show_u16(s, "AUDIO_STATUS1", AUDIO_STATUS1);
	lf1000_i2s_show_u32(s, "CLKENB", CLKENB);
	lf1000_i2s_show_u16(s, "CLKGEN0", CLKGEN0);
	lf1000_i2s_show_u16(s, "CLKGEN1", CLKGEN1);

	return 0;
}

static int lf1000_i2s_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lf1000_i2s_regs_show, inode->i_private);
}

static const struct file_operations lf1000_i2s_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= lf1000_i2s_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static irqreturn_t lf1000_i2s_irq(int irq, void *dev_id)
{
	struct lf1000_i2s_info *lf1000_i2s = dev_id;
	u16 reg = readw(lf1000_i2s->adi_base + AUDIO_IRQ_PEND);

	if (reg & IRQ_POUDR)
		lf1000_i2s->underruns++;

	if (reg & IRQ_PIOVER)
		lf1000_i2s->overruns++;
	
	writew(IRQ_POUDR | IRQ_PIOVER, lf1000_i2s->adi_base + AUDIO_IRQ_PEND);

	return IRQ_HANDLED;
}

/* Set the I2S format: we support data in I2S, left, and right justified format
 * as well as inverting the signals or not. */

static int lf1000_i2s_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	unsigned int regs;

	dbg("%s.%d entered\n", __FUNCTION__, __LINE__);

	/* set the I2S format */
	regs = readw(lf1000_i2s.adi_base + I2S_CONFIG);
	regs &= ~IFMODE_MASK;

	dbg("changing I2S format to 0x%X",
			fmt & SND_SOC_DAIFMT_FORMAT_MASK);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			// 00b is I2S
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			regs |= IFMODE_RJ;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			regs |= IFMODE_LJ;
			break;
		default:
			return -EINVAL;
	}
	writew(regs, lf1000_i2s.adi_base + I2S_CONFIG);

	/* set invert mode */
	regs = readw(lf1000_i2s.adi_base + CLKGEN1);

	dbg("changing I2S invert to: %d\n",
			fmt & SND_SOC_DAIFMT_INV_MASK);

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			regs &= ~OUTCLKINV;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			regs |= OUTCLKINV;
			break;
		default:
			return -EINVAL;
	}

	writew(regs, lf1000_i2s.adi_base + CLKGEN1);
	
	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;
}

static int lf1000_i2s_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	unsigned int regs;
	int pll = get_pll_freq(PLL0);

	dbg("%s.%d entered\n", __FUNCTION__, __LINE__);
	lf1000_i2s.div = lf1000_CalcDivider(pll, 256 * freq);
	if (lf1000_i2s.div <= 0 || lf1000_i2s.div > 64) {
		dev_err(dai->dev, "invalid clock div: %d (freq=%d)\n",
				lf1000_i2s.div, freq);
		lf1000_i2s.div = 64;
	}

	lf1000_i2s.rate = get_pll_freq(PLL0)/lf1000_i2s.div/256;

	if (lf1000_i2s.rate != freq)
		dev_info(dai->dev, "setting %dHz instead of %dHz\n",
				lf1000_i2s.rate, freq);

	/* use PLL0/lf1000_i2s.div, don't invert clock */
	regs = readw(lf1000_i2s.adi_base + CLKGEN0);

    dbg("lf1000-i2s: Clk div = %d", lf1000_i2s.div);
	clear_clk_div(regs);
	set_clk_div(regs, lf1000_i2s.div);
	writew(regs | CLKSRC_PLL0, lf1000_i2s.adi_base + CLKGEN0);

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;
}

static int lf1000_i2s_set_dai_clkdiv(struct snd_soc_dai *dai, int div_id,
		int div)
{
	unsigned int regs;

	dbg("%s.%d entered\n", __FUNCTION__, __LINE__);
 
	if (div <= 0 || div > 64)
		return -EINVAL;

	lf1000_i2s.div = div;
	lf1000_i2s.rate = get_pll_freq(PLL0)/lf1000_i2s.div/256;

	regs = readw(lf1000_i2s.adi_base + CLKGEN0);

	clear_clk_div(regs);
	set_clk_div(regs, div);
	writew(regs | CLKSRC_PLL0, lf1000_i2s.adi_base + CLKGEN0);

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;
}

static int lf1000_i2s_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int regs, bitwidth, rate, channels;

	bitwidth = params_format(params);
	dbg("%s.%d entered  - %p(%d)-%p,%d\n",
		__FUNCTION__, __LINE__, substream,
		substream->stream, snd_soc_dai_get_dma_data(rtd->cpu_dai, substream), bitwidth);

	// check parameter
	if (bitwidth != SNDRV_PCM_FORMAT_S16) {
		dev_err(dai->dev, "wrong format: %X\n", bitwidth);
		return -EINVAL;
	}

	channels = params_channels(params);
	rate = params_rate(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_dai_set_dma_data(rtd->cpu_dai, substream, (void *)&lf1000_i2s_pcm_stereo_out);
	else
		snd_soc_dai_set_dma_data(rtd->cpu_dai, substream, (void *)&lf1000_i2s_pcm_stereo_in);

	regs = readw(lf1000_i2s.adi_base + AUDIO_BUFF_CONFIG);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regs &= ~PO_WIDTH_MASK;
		regs |= BUF_WIDTH_16BIT;
	} else {
		regs &= ~PI_WIDTH_MASK;
		regs |= (BUF_WIDTH_16BIT << 4);
	}

	writew(regs, lf1000_i2s.adi_base + AUDIO_BUFF_CONFIG);
	lf1000_i2s.bitwidth = bitwidth;

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;
}

static int lf1000_i2s_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	unsigned int regs;

	dbg("%s.%d entered substream->stream=%d\n",
		__FUNCTION__, __LINE__, substream->stream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* enable PCM output buffer */
		regs = readw(lf1000_i2s.adi_base + AUDIO_BUFF_CTRL);
		regs |= PCMOBUF_EN;
		writew(regs, lf1000_i2s.adi_base + AUDIO_BUFF_CTRL);

		/* clear IRQ pending */
		regs = readw(lf1000_i2s.adi_base + AUDIO_IRQ_PEND);
		regs |= IRQ_POUDR;
		writew(regs, lf1000_i2s.adi_base + AUDIO_IRQ_PEND);

		/* Enable IRQ */
		regs = readw(lf1000_i2s.adi_base + AUDIO_IRQ_ENA);
		regs |= IRQ_POUDR;
		writew(regs, lf1000_i2s.adi_base + AUDIO_IRQ_ENA);
	} else {
		/* enable PCM input buffer */
		regs = readw(lf1000_i2s.adi_base + AUDIO_BUFF_CTRL);
		regs |= PCMIBUF_EN;
		writew(regs, lf1000_i2s.adi_base + AUDIO_BUFF_CTRL);

		/* clear IRQ pending */
		regs = readw(lf1000_i2s.adi_base + AUDIO_IRQ_PEND);
		regs |= IRQ_PIOVER;
		writew(regs, lf1000_i2s.adi_base + AUDIO_IRQ_PEND);

		/* Enable IRQ */
		regs = readw(lf1000_i2s.adi_base + AUDIO_IRQ_ENA);
		regs |= IRQ_PIOVER;
		writew(regs, lf1000_i2s.adi_base + AUDIO_IRQ_ENA);
	}

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;
}

static int lf1000_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	unsigned int regs;

	dbg("%s.%d Entered cmd=%d\n", __FUNCTION__, __LINE__, cmd);
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:

			/* Link Run */
			regs = readw(lf1000_i2s.adi_base + I2S_CTRL);
			regs |= I2SLINK_RUN;
			writew(regs, lf1000_i2s.adi_base + I2S_CTRL);

			/* Enable I/O */
			regs = readw(lf1000_i2s.adi_base + I2S_CONFIG);
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				regs |= I2SO_EN;
			else
				regs |= I2SI_EN;
			writew(regs, lf1000_i2s.adi_base + I2S_CONFIG);
		break;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		
			/* Link Stop */
			regs = readw(lf1000_i2s.adi_base + I2S_CTRL);
			regs &= ~I2SLINK_RUN;
			writew(regs, lf1000_i2s.adi_base + I2S_CTRL);
			
			/* Disable I/O */
			regs = readw(lf1000_i2s.adi_base + I2S_CONFIG);
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				regs &= ~I2SO_EN;
			else
				regs &= ~I2SI_EN;
			writew(regs, lf1000_i2s.adi_base + I2S_CONFIG);
		break;

		default:
			return -EINVAL;
			break;
	}

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;
}

static int lf1000_i2s_register_init(struct lf1000_i2s_info *i2sdev)
{
	unsigned int regs;

	dbg("%s.%d entered\n", __FUNCTION__, __LINE__);
	/* I2S Control */
	regs = I2S_RESET;
	writew(regs, i2sdev->adi_base + I2S_CTRL);

	/* clock enable */
	regs = PCLKMODE_ALWAYS | CLKGENENB;
	writew(regs, i2sdev->adi_base + CLKENB);

	/* I2S Control */
	regs = I2S_EN;
	writew(regs, i2sdev->adi_base + I2S_CTRL);

	/* I2S Config */
	regs = IFMODE_I2S | SYNC_64FS | CTL_MST;
	writew(regs, i2sdev->adi_base + I2S_CONFIG);

	/* calculate master clock initially for 32KHz */
	i2sdev->div = lf1000_CalcDivider(get_pll_freq(PLL0), 256 * 32000);
	if (i2sdev->div <= 0 || i2sdev->div > 64) {
		dbg("invalid clock div: %d (freq=%d)\n",
				i2sdev->div, 32000);
		i2sdev->div=64;
	}

	/* Turn on Master Clock */
	regs = readw(i2sdev->adi_base + CLKGEN0);
	clear_clk_div(regs);
	set_clk_div(regs, i2sdev->div);
	regs |= CLKSRC_PLL0;

	/* invert i2s MCLK on LFP100 only */
	regs &= ~OUTCLKINV;

	writew(regs, i2sdev->adi_base + CLKGEN0);

	/* start bit clock too */
	regs = readw(i2sdev->adi_base + CLKGEN1);
	clear_clk_div(regs);
	set_clk_div(regs, 4);
	writew(regs | CLKSRC_CLKGEN0, i2sdev->adi_base + CLKGEN1);
	
	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;
}

static int lf1000_i2s_probe(struct snd_soc_dai *dai)
{
	struct i2s_snd_dev  *i2s = snd_soc_dai_get_drvdata(dai);
	struct i2s_plat_data *pdata = i2s->pdata;
	int ret = 0;
	dbg("Entered %s.%d\n", __FUNCTION__, __LINE__);
	memset(&lf1000_i2s, 0, sizeof(struct lf1000_i2s_info));

	lf1000_i2s.adi_base = (void __iomem*)IO_ADDRESS(LF1000_AUDIO_BASE);
	if (!lf1000_i2s.adi_base) {
		printk(KERN_ERR "lf1000 ADI ioremap error\n");
		ret = -ENOENT;
		goto out_remap;
	}

	lf1000_i2s.irq = LF1000_AUDIO_IRQ;

	/* I2S Data Bus (ALT1: I2S, ALT2: AC97)*/
	gpio_configure_pin(GPIO_PORT_A, GPIO_PIN21, GPIO_ALT1, 1, 0, 0);
	gpio_configure_pin(GPIO_PORT_A, GPIO_PIN22, GPIO_ALT1, 1, 0, 0);
	gpio_configure_pin(GPIO_PORT_A, GPIO_PIN23, GPIO_ALT1, 1, 0, 0);
	gpio_configure_pin(GPIO_PORT_A, GPIO_PIN24, GPIO_ALT1, 1, 0, 0);
	gpio_configure_pin(GPIO_PORT_A, GPIO_PIN25, GPIO_ALT1, 1, 0, 0);

	ret = request_irq(lf1000_i2s.irq, lf1000_i2s_irq, 0, DRIVER_NAME,
		&lf1000_i2s);
	if (ret) {
		printk(KERN_ERR "failed to request IRQ\n");
		goto out_irq;
	}

	lf1000_i2s_register_init(&lf1000_i2s);

	lf1000_i2s.debug = debugfs_create_dir("lf1000-i2s", NULL);
	if (!lf1000_i2s.debug || IS_ERR(lf1000_i2s.debug))
		lf1000_i2s.debug = NULL;
	
	if (lf1000_i2s.debug) {
		debugfs_create_file("registers", S_IRUSR, lf1000_i2s.debug,
			&lf1000_i2s, &lf1000_i2s_regs_fops);
		debugfs_create_u32("underruns", S_IRUSR, lf1000_i2s.debug,
				&lf1000_i2s.underruns);
		debugfs_create_u32("overruns", S_IRUSR, lf1000_i2s.debug,
				&lf1000_i2s.overruns);
		debugfs_create_u32("rate", S_IRUSR, lf1000_i2s.debug,
				&lf1000_i2s.rate);
		debugfs_create_u32("div", S_IRUSR, lf1000_i2s.debug,
				&lf1000_i2s.div);
	}
	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;

/* error condition exit */
out_irq:
	free_irq(lf1000_i2s.irq, &lf1000_i2s);
out_remap:
	return ret;
}

static int lf1000_i2s_remove(struct snd_soc_dai *dai)
{
	dbg("Entered: %s.%d\n", __FUNCTION__, __LINE__);

	if (lf1000_i2s.debug)
		debugfs_remove_recursive(lf1000_i2s.debug);

	free_irq(lf1000_i2s.irq, &lf1000_i2s);

	memset(&lf1000_i2s, 0, sizeof(struct lf1000_i2s_info));
	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
}

static struct snd_soc_dai_ops lf1000_i2s_dai_ops = {
	.set_sysclk	= lf1000_i2s_set_dai_sysclk,
	.set_clkdiv	= lf1000_i2s_set_dai_clkdiv,
	.set_fmt	= lf1000_i2s_set_dai_fmt,
	.prepare	= lf1000_i2s_prepare,
	.trigger	= lf1000_i2s_trigger,
	.hw_params	= lf1000_i2s_hw_params,
};


struct snd_soc_dai_driver lf1000_i2s_dai = {
	.probe		= lf1000_i2s_probe,
	.remove		= lf1000_i2s_remove,
	.playback	= {
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= LF1000_I2S_RATES,
		.formats	= SNDRV_PCM_FMTBIT_S16,
	},
	.capture	= {
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= LF1000_I2S_RATES,
		.formats	= SNDRV_PCM_FMTBIT_S16,
	},
	.ops = &lf1000_i2s_dai_ops,
};


static __devinit int lf1000_snd_i2s_probe(struct platform_device *pdev)
{
	printk(KERN_ERR "Probing i2s!\n");
        struct i2s_plat_data     * plat = pdev->dev.platform_data;
        struct i2s_snd_dev       * i2s  = NULL;
        struct snd_soc_dai_driver* dai  = &lf1000_i2s_dai;
        int    ret = 0;

        printk(KERN_WARNING "%s\n", __func__);

    /*  allocate i2c_port data */
    i2s = kzalloc(sizeof(struct i2s_snd_dev), GFP_KERNEL);
    if (!i2s) {
        printk(KERN_ERR "fail, %s allocate driver info ...\n", pdev->name);
        return -ENOMEM;
    }

        ret = snd_soc_register_dai(&pdev->dev, dai);
        if (ret) {
		printk(KERN_ERR "ERROR %d registering DAI", ret);
                goto err_out;
	    }

        /* invert MCLK for LFP100/LFP200 */
        //plat->clk_inv0 = 1;

        i2s->pdata = plat;
        i2s->pdrv  = dai;

        platform_set_drvdata(pdev, i2s);

        return ret;

err_out:
        kfree(i2s);
	printk(KERN_ERR "Error!!!!!");
        return ret;
}


static __devexit int lf1000_snd_i2s_remove(struct platform_device *pdev)
{
        struct i2s_snd_dev *i2s = platform_get_drvdata(pdev);

        printk(KERN_WARNING "%s\n", __func__);

        snd_soc_unregister_dai(&pdev->dev);

        if (i2s)
                kfree(i2s);

        return 0;
}

static struct platform_driver lf1000_i2s_driver = {
	.probe  = lf1000_snd_i2s_probe,
	.remove = lf1000_snd_i2s_remove,
	.driver = {
		.name 	= DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

EXPORT_SYMBOL_GPL(lf1000_i2s_dai);

static int __init lf1000_i2s_init(void)
{
	return platform_driver_register(&lf1000_i2s_driver);
}

static void __exit lf1000_i2s_exit(void)
{
	platform_driver_unregister(&lf1000_i2s_driver);
}

module_init(lf1000_i2s_init);
module_exit(lf1000_i2s_exit);

/* Module information */
MODULE_AUTHOR("Scott Esters");
MODULE_DESCRIPTION("LF1000 I2S SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lf1000-i2s");
