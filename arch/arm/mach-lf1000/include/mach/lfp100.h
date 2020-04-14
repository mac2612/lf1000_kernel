/*
 *
 * lfp100.h -- LFP100 chip support
 *     access the lfp100 via getter / setter functions.
 *
 * Scott Esters <sesters@leapfrog.com>
 *
 */

#ifndef LFP100_H
#define LFP100_H

/*
 * LFP100 definitions
 */

#define LFP100_I2C_ADAPTER	0

/*
 * support first silicon from fab, remove when chips are gone
 */

#define LFP100_ADDR	0xCC
#define LFP100_NAME	"lfp100-chip"

/* calculate the password unlock value */
#define LFP100_UNLOCK(x) ( 0x7D ^ x )

/* LFP100 register list.  Names ending with _PW are password protected */
#define LFP100_CHIPID		0x00
#define LFP100_CONTROL		0x01
#define LFP100_CONTROL_STANDBY	(1 << 3)

#define LFP100_STATUS1		0x02
#define LFP100_STATUS1_SOURCE	    (0x3 << 3)
#define LFP100_STATUS1_SOURCE_AC    (0x3 << 3)
#define LFP100_STATUS1_SOURCE_USB   (0x2 << 3)
#define LFP100_STATUS1_SOURCE_BAT   (0x1 << 3)
#define LFP100_STATUS1_SOURCE_UNDEF (0x0 << 3)
#define LFP100_STATUS1_PB	    (0x1 << 5)

#define	LFP100_STATUS2		0x03
#define LFP100_STATUS2_HP	    (0x1 << 0)
#define LFP100_STATUS2_ABUSY	    (0x1 << 1)

#define LFP100_INT1		0x04
#define LFP100_INT2		0x05
#define LFP100_INT2_BP		(1 << 3)

#define LFP100_INT3		0x06
#define LFP100_MASK1		0x07
#define LFP100_MASK2		0x08
#define LFP100_MASK2_USBM	(1 << 5)
#define LFP100_MASK2_ACM	(1 << 4)
#define LFP100_MASK2_PBM	(1 << 3)

#define LFP100_MASK3		0x09
#define LFP100_MASK3_HP		(1 << 0)

#define LFP100_WLED		0x0A
#define LFP100_PPATH		0x0B
#define LFP100_IO		0x0C
#define LFP100_PASSWORD		0x0D
#define LFP100_P_ENABLE		0x0E
#define LFP100_P_ENABLE_WLED_EN	(1 << 5)

#define LFP100_DCDC1_PW		0x0F
#define LFP100_DCDC2_PW		0x10
#define LFP100_SLEW_PW		0x11
#define LFP100_LDO1_PW		0x12
#define LFP100_LDO2_PW		0x13
#define LFP100_LDO3_PW		0x14
#define LFP100_PG_PW		0x15
#define LFP100_UVLO_PW		0x16
#define LFP100_SEQ1_PW		0x17
#define LFP100_SEQ2_PW		0x18
#define LFP100_SEQ3_PW		0x19
#define LFP100_SEQ4_PW		0x1A
#define LFP100_SEQ5_PW		0x1B
#define LFP100_FORMAT_PW	0x1C
#define LFP100_FILTER		0x1D
#define LFP100_A_APOP_PW	0x1E
#define LFP100_A_CONTROL	0x1F
#define LFP100_A_CONTROL_MIC_EN		(1 << 0)
#define LFP100_A_CONTROL_HP_EN		(1 << 1)
#define LFP100_A_CONTROL_SPK_EN		(1 << 2)
#define LFP100_A_CONTROL_AUTO_MASK	(3 << 3)
#define LFP100_A_CONTROL_AUTO_MUTE	(1 << 3)
#define LFP100_A_CONTROL_AUTO_ROUTE	(2 << 3)
#define LFP100_A_CONTROL_DAC_EN		(1 << 6)
#define LFP100_A_CONTROL_DAC_SW		(1 << 7)

#define LFP100_GAINSL		0x20
#define LFP100_GAINADJ_PW	0x21
#define LFP100_MGAIN		0x22
#define LFP100_MGAIN_AMUTE		(1 << 7)

#define LFP100_VOLUME		0x23
#define LFP100_VOLUME_MUTE		(1 << 6)

#define LFP100_VLIMIT_PW	0x24
#define LFP100_MICGAIN		0x25

#define LFP100_LASTREG		LFP100_MICGAIN

#define LFP100_FIRSTREG		LFP100_CHIPID
#define	LFP100_NUMREGS	(LFP100_LASTREG - LFP100_FIRSTREG + 1)

int  lfp100_get_power_button(void);
int  lfp100_have_lfp100(void);
int  lfp100_is_battery(void);
int  lfp100_read_reg(unsigned int reg);
void lfp100_set_power_button_callback(void (*callback)(void));
void lfp100_set_power_standby(void);
void  lfp100_mute_hp_sp(void);
void  lfp100_unmute_hp_sp(void);
int  lfp100_write_reg(unsigned int reg, unsigned int value);

#endif /* LFP100_H */
