/* 
 * sound/soc/lf1000/lf1000-pcm.c
 *
 * ALSA SoC PCM/DMA interface for the LF1000 SoC.
 *
 * Copyright (c) 2010 LeapFrog Enterprises Inc.
 *
 * Scott Esters <sesters@leapfrog.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include <sound/soc.h>

#include <linux/module.h>

#include <mach/platform.h>
#include <mach/common.h>
#include <mach/dma.h>
#include <mach/memory.h>

#include "lf1000-pcm.h"

#define LF1000_PCM_NUM_STREAMS	2

#define DRIVER_NAME	"lf1000-pcm"

#ifdef CONFIG_SND_LF1000_SOC_DEBUG
#define dbg(x...)	printk(KERN_ALERT DRIVER_NAME ": " x)
#else
#define dbg(x...)
#endif

struct lf1000_runtime_data {
	spinlock_t lock;
	unsigned int dma_ch;
	dma_addr_t dma_buf;
	struct lf1000_pcm_dma_params *params;
};

static const struct snd_pcm_hardware lf1000_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
	              SNDRV_PCM_INFO_BLOCK_TRANSFER |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE | 
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min		= 8000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= LF1000_PCM_NUM_STREAMS,
	.period_bytes_min	= 64,
	.period_bytes_max	= 1024 * 16,
	.periods_min		= 1,
	.periods_max		= 32,
	.buffer_bytes_max	= 64 * 1024,
	.fifo_size		= 16,
};

static irqreturn_t lf1000_pcm_irqhandler(int ch, void *data)
{
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	if (substream)
		snd_pcm_period_elapsed(substream);
	
	return IRQ_HANDLED;
}

static int lf1000_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lf1000_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct lf1000_pcm_dma_params *dma = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	//unsigned long totbytes = params_buffer_bytes(params);
	int ret = 0;
  
	dbg("%s.%d enter %p(%d)\n", __FUNCTION__, __LINE__, substream,
		substream->stream);

	if (!dma) {
		dbg("%s.%d no dma\n", __FUNCTION__, __LINE__);
		return 0;
	}

	/* this may get called several times by oss emulation with different params -HW */
	if (prtd->params == NULL) {

		ret = dma_request("ASoC", DMA_PRIORITY_LV1,
				lf1000_pcm_irqhandler, substream, &prtd->dma_ch);
		if (ret < 0) {
			return ret;
		}
	}
	prtd->params = dma;
	//prtd->params = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
    //ptrd->params->dma_intr_handler = lf1000_pcm_irqhandler;
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);


	//runtime->dma_bytes = totbytes;

	spin_lock_irq(&prtd->lock);
	prtd->dma_buf = runtime->dma_addr;
	spin_unlock_irq(&prtd->lock);

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0; //snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
}

static int lf1000_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct lf1000_runtime_data *prtd = substream->runtime->private_data;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);
	if (prtd->dma_ch) {
		snd_pcm_set_runtime_buffer(substream, NULL);
		dma_release(prtd->dma_ch);
		prtd->dma_ch = 0;
	}
	return 0;
}

static int lf1000_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lf1000_runtime_data *prtd = runtime->private_data;
	struct dma_control ctrl;
	dma_addr_t addr_list[256], buf_addr;
	int ret = 0;
	int size, i;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	if (!prtd || !prtd->params) {
		dbg("%s invalid params\n", __FUNCTION__);
		return 0;
	}

	// get one frame size (bytes)
	size = frames_to_bytes(runtime, runtime->period_size);
	/* insert data */
	dbg("%d %d %d %d %d %d\n",
		(int)runtime->buffer_size,
		(int)runtime->periods,
		(int)runtime->period_size,
		(int)runtime->period_step,
		runtime->frame_bits, size);

	//buf_addr = prtd->dma_buf;
    buf_addr = runtime->dma_addr; // &runtime->dma_buffer_p;

	for (i = 0; i < runtime->periods; i++) {
		addr_list[i] = virt_to_phys(buf_addr);
		buf_addr += size;
	}

	memset(&ctrl, 0, sizeof(struct dma_control));

	dma_transfer_init(prtd->dma_ch, DMA_MEM_MAPPED);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ctrl.transfer = DMA_MEM_TO_IO;
		ctrl.interrupt = DMA_INT_EVERY_BLOCK;
		ctrl.request_id = DMA_PERI_PCMOUT;
		ctrl.io_addr_inc = 0;
		ctrl.src_width = runtime->frame_bits / 8;
		ctrl.dest_width = runtime->frame_bits / 8;

		dbg("%s DMA circular write from %X to %X, %d bufs of %d addr_list %x first %x\n",
				__FUNCTION__, runtime->dma_addr,
				prtd->params->dma_addr, runtime->periods,
				size, virt_to_phys((unsigned int *)addr_list), addr_list[0]);

		ret = dma_circ_write(prtd->dma_ch, (unsigned int *)addr_list,
				prtd->params->dma_addr, runtime->periods, size,
				&ctrl);
	} else {
		ctrl.transfer = DMA_IO_TO_MEM;
		ctrl.interrupt = DMA_INT_EVERY_BLOCK;
		ctrl.request_id = DMA_PERI_PCMIN;
		ctrl.io_addr_inc = 0;
		ctrl.src_width = runtime->frame_bits / 8;
		ctrl.dest_width = runtime->frame_bits / 8;
		
		
		dbg("%s DMA circular read from %X to %X, %d bufs of 0x%X\n",
				__FUNCTION__, prtd->params->dma_addr,
				runtime->dma_addr, runtime->periods,
				size);

		ret = dma_circ_read(prtd->dma_ch, prtd->params->dma_addr,
				(unsigned int *)addr_list, runtime->periods, size,
				&ctrl);
	}

	dbg("%s.%d leaving (ret=%d)\n", __FUNCTION__, __LINE__, ret);
	return ret;
}

static int lf1000_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct lf1000_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	spin_lock(&prtd->lock);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = dma_start(prtd->dma_ch);
		dbg("%s dma_start ret=%d\n", __FUNCTION__, ret);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = dma_stop(prtd->dma_ch);
		dbg("%s dma_stop ret=%d\n", __FUNCTION__, ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock(&prtd->lock);

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return ret;
}

static snd_pcm_uframes_t lf1000_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_runtime->cpu_dai;
	struct lf1000_runtime_data *prtd= runtime->private_data;
	//prtd = snd_soc_dai_get_dma_data(cpu_dai, substream);

	u32 ptr;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	spin_lock(&prtd->lock);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ptr = phys_to_virt(dma_get_write_addr(prtd->dma_ch)) - prtd->dma_buf;
	else
		ptr = phys_to_virt(dma_get_read_addr(prtd->dma_ch)) - prtd->dma_buf;

	spin_unlock(&prtd->lock);

	if (ptr >= snd_pcm_lib_buffer_bytes(substream)) {
		ptr = 0;
	}

	dbg("%s.%d leaving dma_buf=%x ptr=%u frames=%u\n", __FUNCTION__, __LINE__, prtd->dma_buf, ptr,
		bytes_to_frames(runtime, ptr));
	return bytes_to_frames(runtime, ptr);
}

static int lf1000_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lf1000_runtime_data *prtd;

	dbg("%s.%d enter", __FUNCTION__, __LINE__);

	snd_soc_set_runtime_hwparams(substream, &lf1000_pcm_hardware);

	snd_pcm_hw_constraint_integer(substream->runtime,
			SNDRV_PCM_HW_PARAM_PERIODS);

	prtd = kzalloc(sizeof(struct lf1000_runtime_data), GFP_KERNEL);

	if (prtd == NULL) {
		return -ENOMEM;
	}

	spin_lock_init(&prtd->lock);

	runtime->private_data = prtd;

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return 0;
}

static int lf1000_pcm_close(struct snd_pcm_substream *substream)
{
	struct lf1000_runtime_data *prtd = substream->runtime->private_data;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	WARN_ON(!prtd);

	kfree(prtd);

	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);

	return 0;
}



static int lf1000_pcm_mmap(struct snd_pcm_substream *substream,
		struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	dbg("%s.%d Doing mmap: area %x, addr %x, bytes %x", __FUNCTION__, __LINE__, runtime->dma_area, runtime->dma_addr, runtime->dma_bytes);
	
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

struct snd_pcm_ops lf1000_pcm_ops = {
	.open		= lf1000_pcm_open,
	.close		= lf1000_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= lf1000_pcm_hw_params,
	.hw_free	= lf1000_pcm_hw_free,
	.prepare	= lf1000_pcm_prepare,
	.trigger	= lf1000_pcm_trigger,
	.pointer	= lf1000_pcm_pointer,
	.mmap		= lf1000_pcm_mmap,
};

static int lf1000_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = lf1000_pcm_hardware.buffer_bytes_max;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->bytes = size;

	buf->area = dma_alloc_writecombine(pcm->card->dev, size, &buf->addr,
			GFP_KERNEL | GFP_DMA);
	if (!buf->area)
		return -ENOMEM;
	printk("dma_alloc_writecombine %d %x", size, buf->addr);
	if (!lf1000_is_shadow())
		buf->addr |= PHYS_OFFSET_NO_SHADOW;

	dbg("%s leaving dma_buffer = 0x%X\n", __FUNCTION__, buf->addr);
	return 0;
}

static void lf1000_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);
	for (stream = 0; stream < LF1000_PCM_NUM_STREAMS; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes, buf->area,
				buf->addr);
		buf->area = NULL;
	}
	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
}

static u64 lf1000_pcm_dmamask = DMA_BIT_MASK(32);

static int lf1000_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
		struct snd_pcm *pcm)
{
	int ret = 0;

	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &lf1000_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (dai->driver->playback.channels_min) {
		ret = lf1000_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
	}
	
	if (dai->driver->capture.channels_min) {
		ret = lf1000_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE);
	}


	dbg("%s.%d leaving\n", __FUNCTION__, __LINE__);
	return ret;
}

struct snd_soc_platform_driver lf1000_snd_pcm_plat_drv = {
	.ops	        = &lf1000_pcm_ops,
	.pcm_new	= lf1000_pcm_new,
	.pcm_free	= lf1000_pcm_free_dma_buffers,
};


int lf1000_pcm_probe(struct platform_device *pdev)
{
	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);
        return snd_soc_register_platform(&pdev->dev, &lf1000_snd_pcm_plat_drv);
}

int lf1000_pcm_remove(struct platform_device *pdev)
{
	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);
        snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

int lf1000_pcm_suspend(struct snd_soc_dai *dai)
{
	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	return 0;
}

int lf1000_pcm_resume(struct snd_soc_dai *dai)
{
	dbg("%s.%d enter\n", __FUNCTION__, __LINE__);

	return 0;
}

static struct platform_driver lf1000_snd_pcm_driver = {
	.probe = lf1000_pcm_probe,
	.remove = lf1000_pcm_remove,
	.driver = {
		.name = "lf1000-pcm",
		.owner = THIS_MODULE,
	},
};

EXPORT_SYMBOL_GPL(lf1000_snd_pcm_driver);

static int __init lf1000_snd_pcm_init(void)
{
	printk(KERN_WARNING "XXXX LF1000-PCM init");
	return platform_driver_register(&lf1000_snd_pcm_driver);
}

static void __exit lf1000_snd_pcm_exit(void)
{
	printk(KERN_ERR "XXXX LF1000 PCM exit");
	platform_driver_unregister(&lf1000_snd_pcm_driver);
}

module_init(lf1000_snd_pcm_init);
module_exit(lf1000_snd_pcm_exit);


MODULE_AUTHOR("Scott Esters");
MODULE_DESCRIPTION("LF1000 SoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lf1000-pcm");

