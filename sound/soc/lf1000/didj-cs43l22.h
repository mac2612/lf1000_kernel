#ifndef __DIDJ_CS43L22_H__
#define __DIDJ_CS43L22_H__

/* register to value */
u8 cs43L22_settings[][2] = {
	{0x02, 0x01}, /* Power Control 1 */
	{0x03, 0x07}, /* Power Control 2 */
	{0x05, 0xA0}, /* Clocking Control */
	{0x06, 0x27}, /* Interface Control 1, Slave, I2S */
	{0x07, 0x00}, /* Interface Control 2 */
	{0x08, 0x81}, /* Input A Select ADCA and PGAA */
	{0x09, 0x81}, /* Input B Select ADCB and PGAB */
	{0x0A, 0xA5}, /* Analog & HPF Control */
	{0x0B, 0x00}, /* ADC HPF Corner Frequency */
	{0x0C, 0x00}, /* Misc ADC Control */
	{0x0D, 0x10}, /* Playback Control 1 */
	{0x0E, 0x02}, /* Passthru Analog, adjust volume at zero-crossings */
	{0x0F, 0xFA}, /* Playback Control 2, mute headphone and speaker */
	{0x10, 0x00}, /* MIC A */
	{0x11, 0x00}, /* MIC B */
	{0x12, 0x00}, /* ALC, PGA A */
	{0x13, 0x00}, /* ALC, PGA B */
	{0x14, 0x00}, /* Passthru A Volume */
	{0x15, 0x00}, /* Passthru B Volume */
	{0x16, 0x00}, /* ADC A Volume */
	{0x17, 0x00}, /* ADC B Volume */
	{0x18, 0x80}, /* ADC Mixer Channel A Mute */
	{0x19, 0x80}, /* ADC Mixer Channel B Mute */
	{0x1A, 0x00}, /* PCMA Mixer Volume */
	{0x1B, 0x00}, /* PCMB Mixer Volume */
	{0x1C, 0x00}, /* Beep Frequency */
	{0x1D, 0x00}, /* Beep On Time */
	{0x1E, 0x3F}, /* Beep & Tone Configuration */
	{0x1F, 0xFF}, /* Tone Control */
	{0x20, 0x06}, /* Master Volume Control MSTA */
	{0x21, 0x06}, /* Master Volume Control MSTB */
	{0x22, 0x90}, /* Headphone Volume Control HPA */
	{0x23, 0x90}, /* Headphone Volume Control HPB */
	{0x24, 0xD3}, /* Speaker Volume Control SPKA */
	{0x25, 0xD3}, /* Speaker Volume Control SPKB */
	{0x26, 0x50}, /* ADC & PCM Channel Mixer */
	{0x27, 0x04}, /* Limiter Control 1 */
	{0x28, 0x8A}, /* Limiter Control 2 */
	{0x29, 0xC3}, /* Limiter Attack Rate */
	{0x2A, 0x00}, /* ALC Enable & Attack Rate */
	{0x2B, 0x00}, /* ALC Release Rate */
	{0x2C, 0x00}, /* ALC Threshold */
	{0x2D, 0x00}, /* Noise Gate Control */
	{0x2F, 0x00}, /* Battery Compensation */
	{0x32, 0x00}, /* Temperature monitor Control */
	{0x33, 0x00}, /* Thermal Foldback */
	{0x34, 0x5F}, /* Charge Pump Frequency */
	{0x02, 0x9E}, /* Power Control 1 */
	{0x04, 0x50}, /* Power Control 3 (speaker control) */
};

#endif /* __DIDJ_CS43L22_H__ */
