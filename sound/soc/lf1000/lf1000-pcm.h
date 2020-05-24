/*
 * Copyright (c) 2011 Leapfrog Enterprises Inc.
 *
 * Scott Esters <sesters@leapfrog.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#ifndef __LF1000_PCM_H__
#define __LF1000_PCM_H_
#ifndef __LF1000_AUDIOJACK__
#define __LF1000_AUDIOJACK__
enum {
        AUDIO_NORMAL          = 0,      /* normal audio operation */
        AUDIO_HEADPHONES_ONLY = 1,      /* headphones only on     */
};
#endif

enum {
        MIXER_NORMAL          = 0,      /* normal mixer operation */
        MIXER_MONO            = 1,      /* mixer always mono      */
};

struct lf1000_pcm_dma_params {
	char *name;		/* device name */
	dma_addr_t dma_addr;	/* physical address */
	int dma_size;		/* Source / Dest width */
};

#define LF1000_SND_SOC_DAI_PCM  "lf1000-pcm"

extern struct snd_soc_platform lf1000_soc_platform;

#endif /* __LF1000_PCM_H__ */
