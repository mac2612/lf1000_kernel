/*
 * CS43L22 driver
 *
 * Author:
 *   Scott Esters <sesters@leapfrog.com>
 * Based on CS42L52 driver by Inchoon Choi <sonne@bokwang.com>, Aug 2009
 *
 * Description:
 *   Cirrus Logic CS43L22 driver
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */


#ifndef _CS43L22_H
#define _CS43L22_H

#define CS43L22SYSCLK	0
#define CS43L22_ADDR	0x94

#define CS43L22_CHIPID                  0x01
#define CS43L22_POWER_CONTROL_1         0x02
#define CS43L22_SPKCTL                  0x04
#define CS43L22_PLAYBACK_CONTROL_2      0x0F
#define CS43L22_MASTER_VOLUME_A         0x20
#define CS43L22_MASTER_VOLUME_B         0x21
#define CS43L22_HEADPHONE_A             0x22
#define CS43L22_HEADPHONE_B             0x23
#define CS43L22_SPEAKER_A               0x24
#define CS43L22_SPEAKER_B               0x25
#define CS43L22_MIXER                   0x26

#define	CS43L22_SPKCTL_NORMAL		0x50
#define CS43L22_SPKCTL_HEADPHONES_ONLY	0xAF

#define CS43L22_MIXER_MONO		0x50
#define CS43L22_MIXER_STEREO		0x00

extern struct snd_soc_dai cs43l22_dai;
extern struct snd_soc_codec_device soc_codec_dev_cs43l22;

struct cs43l22_setup_data {
	int		i2c_bus;
	unsigned short	i2c_address;
};

#endif
