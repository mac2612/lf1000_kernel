/*
 * LFP100 driver
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __LFP100_H__
#define __LFP100_H__

#define CODEC_NAME	"lfp100-codec"

extern struct snd_soc_dai lfp100_dai;
extern struct snd_soc_codec_device soc_codec_dev_lfp100;

struct lfp100_setup_data {
	int		i2c_bus;
	unsigned short	i2c_address;
};

struct lfp100_private {
	struct		snd_soc_codec codec;
	bool		manual_mute;
	unsigned int	force_audio;
	unsigned int	force_mixer;
};

#endif /* __LFP100_H__ */
