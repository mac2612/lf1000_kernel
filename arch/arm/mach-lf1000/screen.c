/*
 * arch/arm/mach-lf1000/screen.c
 *
 * Copyright 2010 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <ayurovsky@leapfrog.com>
 *
 * Display Controller (DPC) support.  We also describe platform screen modules
 * and provide information on the system's screen.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/delay.h>

#include <mach/platform.h>
#include <mach/screen.h>
#include <mach/gpio.h>

#define RESSIZE(res)	(((res)->end - (res)->start)+1)
#define DRIVER_NAME	"lf1000-dpc"
#define DPC_DEFAULT_HZ	6400000

static struct lf1000_screen_info screens[] = {
	{
		.name = "ILI9322",
		.xres	= 320,
		.yres	= 240,
		.clk_hz	= 6400000,
		.hsw	= 2,
		.hfp	= 50,
		.hbp	= 36,
		.vsw	= 1,
		.vfp	= 4,
		.vbp	= 17,
	},
	{
		.name = "ILI6480G2",
		.xres	= 480,
		.yres	= 272,
		.clk_hz = 9200000,
		.hsw	= 2,
		.hfp	= 5,
		.hbp	= 40,
		.vsw	= 1,
		.vfp	= 8,
		.vbp	= 8,
	},
};

static struct lf1000_screen_info *system_screen = &screens[0];

static int __init screen_module_name(char *str)
{
	int i;
	int l = 0;
	char c;

	do {
		c = *(str+l);
		l++;
	} while (c && c != ' ');

	for (i = 0; i < ARRAY_SIZE(screens); i++) {
		if (strlen(screens[i].name) == l-1 &&
		    !strncmp(str, screens[i].name, l-1)) {
			system_screen = &screens[i];
			break;
		}
	}

	return 0;
}

__setup("screen_module=", screen_module_name);

static struct dpc_priv {
	void __iomem			*mem;
	int				div;
	struct lf1000_screen_info	*screen;
	unsigned			tvout : 1;
	struct dentry			*debug;
} dpc;

struct lf1000_screen_info *lf1000_get_screen_info(void)
{
	return system_screen;
}
EXPORT_SYMBOL_GPL(lf1000_get_screen_info);

/* DPC Hardware */

static char pvd_pins[] = {  
	GPIO_PIN0, GPIO_PIN1, GPIO_PIN2, GPIO_PIN3, GPIO_PIN4, GPIO_PIN5,
	GPIO_PIN6, GPIO_PIN7, GPIO_PIN16, GPIO_PIN17, GPIO_PIN18, GPIO_PIN19,
	GPIO_PIN20, GPIO_PIN21, GPIO_PIN22, GPIO_PIN23, GPIO_PIN24,
	GPIO_PIN25, GPIO_PIN26, GPIO_PIN27, GPIO_PIN28, GPIO_PIN29, GPIO_PIN30,
	GPIO_PIN31                
};                                

static char pvd_ports[] = { 
	GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_A,
	GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_A, GPIO_PORT_B, GPIO_PORT_B,
	GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B,
	GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B,
	GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B, GPIO_PORT_B,
};

#define DPCHTOTAL	0x07C
#define DPCHSWIDTH	0x07E
#define DPCHASTART	0x080
#define DPCHAEND	0x082
#define DPCVTOTAL	0x084
#define DPCVSWIDTH	0x086
#define DPCVASTART	0x088
#define DPCVAEND	0x08A
#define DPCCTRL0	0x08C
#define DPCCTRL1	0x08E
#define DPCEVTOTAL	0x090
#define DPCEVSWIDTH	0x092
#define DPCEVASTART	0x094
#define DPCEVAEND	0x096
#define DPCCTRL2	0x098
#define DPCVSEOFFSET	0x09A
#define DPCVSSOFFSET	0x09C
#define DPCEVSEOFFSET	0x09E
#define DPCEVSSOFFSET	0x0A0
#define DPCDELAY0	0x0A2
#define DPCCLKENB	0x1C0
#define DPCCLKGEN0	0x1C4
#define DPCCLKGEN1	0x1C8

#define VENCCTRLA	0x002 
#define VENCCTRLB	0x004 
#define VENCSCH		0x008 
#define VENCHUE		0x00A 
#define VENCSAT		0x00C 
#define VENCCRT		0x00E 
#define VENCBRT		0x010 
#define VENCFSCADJH	0x012 
#define VENCFSCADJL	0x014
#define VENCDACSEL	0x020 
#define VENCICNTL	0x040
#define VENCHSVS0	0x048 
#define VENCHSOS	0x04A 
#define VENCHSOE	0x04C 
#define VENCVSOS	0x04E 
#define VENCVSOE	0x050  
#define DPUPSCALECON0	0x0A4
#define DPUPSCALECON1	0x0A6
#define DPUPSCALECON2	0x0A8

/* DPC CLOCK GENERATION CONTROL 0 REGISTER (DPCCLKGEN0) */
#define OUTCLKENB       15
#define OUTCLKDELAY0    12
#define CLKDIV0         4
#define CLKSRCSEL0      1
#define OUTCLKINV0      0

/* DPC CLOCK GENERATION CONTROL 1 REGISTER (DPCCLKGEN1) */
#define OUTCLKDELAY1    12
#define CLKDIV1         4
#define CLKSRCSEL1      1
#define OUTCLKINV1      0

/* debugging */

static void regs_show_u16(struct seq_file *s, const char *nm, u32 reg)
{
	struct dpc_priv *priv = s->private;

	seq_printf(s, "%10s:\t0x%04X\n", nm, readw(priv->mem + reg));
}

static void regs_show_u32(struct seq_file *s, const char *nm, u32 reg)
{
	struct dpc_priv *priv = s->private;

	seq_printf(s, "%10s:\t0x%08X\n", nm, readl(priv->mem + reg));
}

static int lf1000_regs_show(struct seq_file *s, void *v)
{
	struct dpc_priv *priv = s->private;
	void __iomem *mem = priv->mem;
	int n;

	for (n = 0; n < 2; n++, priv->mem += 0x400)
	{
	regs_show_u16(s, "HTOTAL", DPCHTOTAL);
	regs_show_u16(s, "HSWIDTH", DPCHSWIDTH);
	regs_show_u16(s, "HASTART", DPCHASTART);
	regs_show_u16(s, "HAEND", DPCHAEND);
	regs_show_u16(s, "VTOTAL", DPCVTOTAL);
	regs_show_u16(s, "VSWIDTH", DPCVSWIDTH);
	regs_show_u16(s, "VASTART", DPCVASTART);
	regs_show_u16(s, "VAEND", DPCVAEND);
	regs_show_u16(s, "CTRL0", DPCCTRL0);
	regs_show_u16(s, "CTRL1", DPCCTRL1);
	regs_show_u16(s, "EVTOTAL", DPCEVTOTAL);
	regs_show_u16(s, "EVSWIDTH", DPCEVSWIDTH);
	regs_show_u16(s, "EVASTART", DPCEVASTART);
	regs_show_u16(s, "EVAEND", DPCEVAEND);
	regs_show_u16(s, "CTRL2", DPCCTRL2);
	regs_show_u16(s, "VSEOFFSET", DPCVSEOFFSET);
	regs_show_u16(s, "VSSOFFSET", DPCVSSOFFSET);
	regs_show_u16(s, "EVSEOFFSET", DPCEVSEOFFSET);
	regs_show_u16(s, "EVSSOFFSET", DPCEVSSOFFSET);
	regs_show_u16(s, "DELAY0", DPCDELAY0);

	regs_show_u32(s, "CLKENB", DPCCLKENB);
	regs_show_u32(s, "CLKGEN0", DPCCLKGEN0);
	regs_show_u32(s, "CLKGEN1", DPCCLKGEN1);

	regs_show_u16(s, "VENCCTRLA", VENCCTRLA);
	regs_show_u16(s, "VENCCTRLB", VENCCTRLB);
	regs_show_u16(s, "VENCSCH", VENCSCH);
	regs_show_u16(s, "VENCHUE", VENCHUE);
	regs_show_u16(s, "VENCSAT", VENCSAT);
	regs_show_u16(s, "VENCCRT", VENCCRT);
	regs_show_u16(s, "VENCBRT", VENCBRT);
	regs_show_u16(s, "VENCFSCADJH", VENCFSCADJH);
	regs_show_u16(s, "VENCFSCADJL", VENCFSCADJL);
	regs_show_u16(s, "VENCDACSEL", VENCDACSEL);
	regs_show_u16(s, "VENCICNT", VENCICNTL);
	regs_show_u16(s, "VENCHSVS", VENCHSVS0);
	regs_show_u16(s, "VENCHS", VENCHSOS);
	regs_show_u16(s, "VENCHS", VENCHSOE);
	regs_show_u16(s, "VENCVS", VENCVSOS);
	regs_show_u16(s, "VENCVS", VENCVSOE);
	regs_show_u16(s, "DPUPSCALECON0", DPUPSCALECON0);
	regs_show_u16(s, "DPUPSCALECON1", DPUPSCALECON1);
	regs_show_u16(s, "DPUPSCALECON2", DPUPSCALECON2);
	}

	priv->mem = mem;

	return 0;
}

static int lf1000_dpc_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lf1000_regs_show, inode->i_private);
}

static const struct file_operations lf1000_dpc_regs_fops = {
	.owner		= THIS_MODULE,
	.open		= lf1000_dpc_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int screen_show(struct seq_file *s, void *v)
{
	seq_printf(s, "%s, %dx%d @ %dHz\n", system_screen->name,
			system_screen->xres, system_screen->yres,
			system_screen->clk_hz);
	seq_printf(s, "hsw: %d, hfp %d, hbp: %d\n", system_screen->hsw,
			system_screen->hfp, system_screen->hbp);
	seq_printf(s, "vsw: %d, vfp %d, vbp: %d\n", system_screen->vsw,
			system_screen->vfp, system_screen->vbp);

	return 0;
}

static int lf1000_dpc_screen_open(struct inode *inode, struct file *file)
{
	return single_open(file, screen_show, inode->i_private);
}

static const struct file_operations lf1000_dpc_screen_fops = {
	.owner		= THIS_MODULE,
	.open		= lf1000_dpc_screen_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* controls */

void lf1000_dpc_clear_int(void)
{
	u16 reg = readw(dpc.mem + DPCCTRL0);
	writew(reg | (1<<10), dpc.mem + DPCCTRL0);
}
EXPORT_SYMBOL_GPL(lf1000_dpc_clear_int);

bool lf1000_dpc_int_pending(void)
{
	return !!(readw(dpc.mem + DPCCTRL0) & (1<<10));
}
EXPORT_SYMBOL_GPL(lf1000_dpc_int_pending);

void lf1000_dpc_enable_int(bool en)
{
	u16 reg = readw(dpc.mem + DPCCTRL0);

	reg |= (1<<10);

	if (en)
		writew(reg | (1<<11), dpc.mem + DPCCTRL0);
	else
		writew(reg & ~(1<<11), dpc.mem + DPCCTRL0);

	lf1000_dpc_clear_int();
}
EXPORT_SYMBOL_GPL(lf1000_dpc_enable_int);

static void lf1000_dpc_set_enable(u8 index, bool en)
{
	void __iomem *mem = index ? dpc.mem + 0x400 : dpc.mem;
	u16 reg = readw(mem + DPCCTRL0) & ~(1<<15);

	if (en)
		reg |= (1<<15);
	writew(reg, mem + DPCCTRL0);
}

static void lf1000_dpc_set_clock_enable(u8 index, bool en)
{
	u32 reg = readl(dpc.mem + DPCCLKENB) & ~(1<<2);
	void __iomem *mem = index ? dpc.mem + 0x400 : dpc.mem;

	if (en)
		reg |= (1<<2);
	writel(reg, mem + DPCCLKENB);
}

static void lf1000_dpc_configure(void)
{
	u32 reg32;
	u16 reg16;
	int i;
	unsigned int div;
	struct lf1000_screen_info *screen = dpc.screen;

	for (i = 0; i < ARRAY_SIZE(pvd_pins); i++)
		gpio_configure_pin(pvd_ports[i], pvd_pins[i], GPIO_ALT1,
				1, 0, 0);

	/* set PCLK mode to "only when CPU accesses" */
	reg32 = readl(dpc.mem + DPCCLKENB) & ~(1<<3);
	writel(reg32, dpc.mem + DPCCLKENB);

	/* configure Clock 0 */
	div = dpc.div > 0 ? dpc.div - 1 : dpc.div;
	div &= 0x3F;
	reg32 = readl(dpc.mem + DPCCLKGEN0);
	reg32 &= ~((7<<CLKSRCSEL0)|(0x3F<<CLKDIV0)|(3<<OUTCLKDELAY0));
	reg32 |= (1<<CLKSRCSEL0);		/* clock source: PLL1 */
	reg32 |= (div<<CLKDIV0);		/* clock divider */
	reg32 |= (1<<OUTCLKINV0);		/* inverted */
	writel(reg32, dpc.mem + DPCCLKGEN0);

	/* configure Clock 1 to derive from Clock 0, no divider */
	reg32 = readl(dpc.mem + DPCCLKGEN1);
	reg32 &= ~((7<<CLKSRCSEL1)|(0x3F<<CLKDIV1)|(3<<OUTCLKDELAY1));
	reg32 |= (7<<CLKSRCSEL1);		/* clock source: DPC Clock 0 */
	reg32 |= (1<<OUTCLKINV1);		/* inverted */
	writel(reg32, dpc.mem + DPCCLKGEN1);

	lf1000_dpc_set_clock_enable(0, 1);

	/* set Control Register 0 flags */
	reg16 = readw(dpc.mem + DPCCTRL0) & ~(1<<10);
	reg16 &= ~(0x304); /* turn off: embedded sync, inverted, interlace */
	reg16 |= (1<<12); /* enable RGB mode */
	writew(reg16, dpc.mem + DPCCTRL0);

	/* set Control Register 1 flags (vid order: CbYCrY, don't swap RGB,
	 * no dithering), RGB888 format */
	reg16 = readw(dpc.mem + DPCCTRL1) & ~(0xAFFF);
	writew(reg16 | (3<<8), dpc.mem + DPCCTRL1);

	/* set Control Register 2 flags (pad Vclk set to nVCLK2) */
	reg16 = readw(dpc.mem + DPCCTRL2);
	reg16 &= ~(3<<0);
	writew(reg16 | (1<<0), dpc.mem + DPCCTRL2);
	/* set up horizonal sync */
	writew(screen->xres - 1 + screen->hbp + screen->hfp + screen->hsw,
			dpc.mem + DPCHTOTAL);
	writew(screen->hsw - 1, dpc.mem + DPCHSWIDTH);
	writew(screen->hsw + screen->hbp - 1, dpc.mem + DPCHASTART);
	writew(screen->hsw + screen->hbp + screen->xres - 1,
			dpc.mem + DPCHAEND);
	lf1000_dpc_clear_int();

	/* set up vertical sync */
	writew(screen->yres - 1 + screen->vbp + screen->vfp + screen->vsw,
			dpc.mem + DPCVTOTAL);
	writew(screen->vsw - 1, dpc.mem + DPCVSWIDTH);
	writew(screen->vsw + screen->vbp - 1, dpc.mem + DPCVASTART);
	writew(screen->vsw + screen->vbp + screen->yres - 1,
			dpc.mem + DPCVAEND);
	writew(0, dpc.mem + DPCEVTOTAL);
	writew(0, dpc.mem + DPCEVSWIDTH);
	writew(0, dpc.mem + DPCEVASTART);
	writew(0, dpc.mem + DPCEVAEND);
	lf1000_dpc_clear_int();

	/* delays */
	writew((7<<8)|(7<<4)|(7<<0), dpc.mem + DPCDELAY0);
	lf1000_dpc_clear_int();

	/* vsync offset */
	writew(1, dpc.mem + DPCVSEOFFSET);
	writew(1, dpc.mem + DPCVSSOFFSET);
	writew(1, dpc.mem + DPCEVSEOFFSET);
	writew(1, dpc.mem + DPCEVSSOFFSET);

	lf1000_dpc_set_enable(0, 1);
}

static void lf1000_dpc_set_encoder_powerdown(bool en)
{
	u16 reg;
	void __iomem *mem = dpc.mem + 0x400; /* second DPC register set */

	/* power-down mode */
	reg = readw(mem + VENCCTRLA) & ~(1<<7);
	if (en)
		reg |= (1<<7);
	writew(reg, mem + VENCCTRLA);

	/* DAC output */
	reg = (en) ? 0x0 : 0x1;
	writew(reg, mem + VENCDACSEL);
}

static void lf1000_dpc_set_dac_enable(bool en)
{
	u16 reg;
	void __iomem *mem = dpc.mem + 0x400; /* second DPC register set */

	reg = readw(mem + DPCCTRL0) & ~(1<<10);
	if (en)
		reg |= (1<<13);
	else
		reg &= ~(1<<13);
	reg |= (1<<14); /* ENCENB */
	writew(reg, mem + DPCCTRL0);

	if (en)
		writew(7, mem + VENCICNTL); /* encoder timing config */
}

static inline void lf1000_dpc_reset_encoder(void)
{
	lf1000_dpc_set_dac_enable(1);
	udelay(100);
	lf1000_dpc_set_clock_enable(1, 1);
	udelay(100);
	lf1000_dpc_set_dac_enable(0);
	udelay(100);
	lf1000_dpc_set_clock_enable(1, 0);
	udelay(100);
	lf1000_dpc_set_clock_enable(1, 1);
}

static void lf1000_dpc_enable_tvout(void)
{
	u32 reg32;
	u16 reg16;
	struct lf1000_screen_info *screen = dpc.screen;
	void __iomem *mem = dpc.mem + 0x400; /* second DPC register set */

	/* set PCLK mode to "only when CPU accesses" */
	reg32 = readl(mem + DPCCLKENB) & ~(1<<3);
	writel(reg32, mem + DPCCLKENB);

	/* configure Clock 0 */
	reg32 = readl(mem + DPCCLKGEN0);
	reg32 &= ~((7<<CLKSRCSEL0)|(0x3F<<CLKDIV0)|(3<<OUTCLKDELAY0));
	reg32 |= (5<<CLKSRCSEL0);		/* clock source: XTI */
	writel(reg32, mem + DPCCLKGEN0);

	/* configure Clock 1 to derive from Clock 0 */
	reg32 = readl(mem + DPCCLKGEN1);
	reg32 &= ~((7<<CLKSRCSEL1)|(0x3F<<CLKDIV1)|(3<<OUTCLKDELAY1));
	reg32 |= (7<<CLKSRCSEL1);		/* clock source: DPC Clock 0 */
	reg32 |= (1<<4);			/* divide by 2 */
	writel(reg32, mem + DPCCLKGEN1);

	/* set Control Register 0 flags */
	reg16 = readw(mem + DPCCTRL0) & ~(1<<10);
	reg16 &= ~(0x304); /* turn off: embedded sync, inverted */
	reg16 |= (1<<9); /* enable interlacing */
	reg16 &= ~(1<<12); /* disable RGB mode */
	writew(reg16, mem + DPCCTRL0);

	/* set Control Register 1 flags (vid order: CbYCrY, don't swap RGB,
	 * no dithering, CCIR601B format, clamp YCbCr) */
	reg16 = readw(mem + DPCCTRL1) & ~(0xAFFF);
	writew(reg16 | (1<<13) | (13<<8), mem + DPCCTRL1);

	/* set Control Register 2 flags (pad Vclk set to nVCLK2) */
	reg16 = readw(mem + DPCCTRL2);
	reg16 &= ~(3<<0);
	writew(reg16 | (1<<0), mem + DPCCTRL2);
	/* set up horizonal sync: 720 */
	writew(720 - 1 + 81 + 24 + 33, mem + DPCHTOTAL);
	writew(33 - 1, mem + DPCHSWIDTH);
	writew(33 + 81 - 1, mem + DPCHASTART);
	writew(33 + 81 + 720 - 1, mem + DPCHAEND);
	lf1000_dpc_clear_int();

	/* set up vertical sync: 240 */
	writew(240 - 1 + 16 + 3 + 3, mem + DPCVTOTAL);
	writew(3 - 1, mem + DPCVSWIDTH);
	writew(3 + 16 - 1, mem + DPCVASTART);
	writew(3 + 16 + 240 - 1, mem + DPCVAEND);
	writew(240 - 1 + 3 + 4 + 16, mem + DPCEVTOTAL);
	writew(3 - 1, mem + DPCEVSWIDTH);
	writew(3 - 1 + 16, mem + DPCEVASTART);
	writew(240 - 1 + 16 + 3, mem + DPCEVAEND);
	lf1000_dpc_clear_int();

	/* delays */
	writew((4<<8)|(4<<4)|(4<<0), mem + DPCDELAY0);
	lf1000_dpc_clear_int();

	/* vsync offset */
	writew(0, mem + DPCVSEOFFSET);
	writew(0, mem + DPCVSSOFFSET);
	writew(0, mem + DPCEVSEOFFSET);
	writew(0, mem + DPCEVSSOFFSET);

	lf1000_dpc_reset_encoder();
	lf1000_dpc_set_dac_enable(1);
	lf1000_dpc_set_encoder_powerdown(1);

	/* NTSC mode with pedestal */
	reg16 = readw(mem + VENCCTRLA);
	reg16 &= ~((0xF<<3)|(0x3<<0));
	writew(reg16 | (0x9<<3)|(0<<0), mem + VENCCTRLA);

	/* color burst frequency adjust */
	writew(0, mem + VENCFSCADJH);
	writew(0, mem + VENCFSCADJL);

	/* luma/chroma bandwidth */
	writew(0, mem + VENCCTRLB);

	/* color phase, hue, saturation, contrast, brightness */
	writew(0, mem + VENCSCH);
	writew(0, mem + VENCHUE);
	writew(0, mem + VENCSAT);
	writew(0, mem + VENCCRT);
	writew(0, mem + VENCBRT);

	/* set encoder timings */
	writew(((1716-1)>>8) & 0x7, mem + VENCHSVS0);
	writew(64-1, mem + VENCHSOS);
	writew(1716-1, mem + VENCHSOE);
	writew(0, mem + VENCVSOS);
	writew(3, mem + VENCVSOE);

	/* set horizontal upscaler: screen resolution to 720 */
	writew(screen->xres - 1, mem + DPUPSCALECON2);
	reg32 = ((screen->xres - 1)*(1<<11))/(720-1);
	reg16 = reg32;
	writew((reg16>>8) & 0x7F, mem + DPUPSCALECON1);
	writew(((reg16 & 0xFF) << 8) | 1, mem + DPUPSCALECON0);

	lf1000_dpc_set_encoder_powerdown(0);
	lf1000_dpc_set_enable(1, 1);
	lf1000_dpc_set_clock_enable(1, 1);
}

static void lf1000_dpc_disable_tvout(void)
{
	lf1000_dpc_set_dac_enable(0);
	lf1000_dpc_set_encoder_powerdown(1);
	lf1000_dpc_set_clock_enable(1, 0);
	lf1000_dpc_set_enable(1, 0);
}

/* sysfs interface */

static ssize_t show_tvout(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", dpc.tvout);
}

static ssize_t set_tvout(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int val;

	if ((sscanf(buf, "%i", &val) != 1) || (val != 0 && val != 1))
		return -EINVAL;

	if (val)
		lf1000_dpc_enable_tvout();
	else
		lf1000_dpc_disable_tvout();
	dpc.tvout = val;

	return count;
}

static DEVICE_ATTR(tvout, S_IRUGO | S_IWUGO, show_tvout, set_tvout);

static ssize_t show_xres(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", system_screen->xres);
}

static DEVICE_ATTR(xres, S_IRUGO , show_xres, NULL);

static ssize_t show_yres(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", system_screen->yres);
}

static DEVICE_ATTR(yres, S_IRUGO , show_yres, NULL);

static struct attribute *dpc_attributes[] = {
	&dev_attr_tvout.attr,
	&dev_attr_xres.attr,
	&dev_attr_yres.attr,
	NULL
};

static struct attribute_group dpc_attr_group = {
	.attrs = dpc_attributes,
};

static int lf1000_dpc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct dentry *dir;
	struct lf1000_screen_info *screen = lf1000_get_screen_info();

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get resource\n");
		return -ENXIO;
	}

	if (!screen->clk_hz)
		screen->clk_hz = DPC_DEFAULT_HZ;
	dpc.div = lf1000_CalcDivider(get_pll_freq(PLL1), screen->clk_hz);
	if (dpc.div < 0) {
		dev_err(&pdev->dev, "failed to get clock divider\n");
		return -EFAULT;
	}

	if (!request_mem_region(res->start, RESSIZE(res), DRIVER_NAME)) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		return -EBUSY;
	}

	dpc.mem = ioremap_nocache(res->start, RESSIZE(res));
	if (!dpc.mem) {
		dev_err(&pdev->dev, "failed to ioremap\n");
		release_mem_region(res->start, RESSIZE(res));
		return -ENOMEM;
	}

	dev_info(&pdev->dev, "screen: %dx%d\n", screen->xres, screen->yres);

	dpc.screen = screen;

	lf1000_dpc_configure();

	dpc.tvout = gpio_have_tvout();
	if (dpc.tvout)
		lf1000_dpc_enable_tvout();
	else
		lf1000_dpc_disable_tvout();

	sysfs_create_group(&pdev->dev.kobj, &dpc_attr_group);

	dir = debugfs_create_dir(DRIVER_NAME, NULL);
	if (!dir || IS_ERR(dir)) {
		dpc.debug = NULL;
	} else {
		debugfs_create_file("registers", S_IRUSR, dir, &dpc,
				&lf1000_dpc_regs_fops);
		debugfs_create_file("screen", S_IRUSR, dir, &dpc,
				&lf1000_dpc_screen_fops);
		debugfs_create_u32("div", S_IRUSR, dir, (u32 *)&dpc.div);
	}

	return 0;
}

static int lf1000_dpc_remove(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	sysfs_remove_group(&pdev->dev.kobj, &dpc_attr_group);

	iounmap(dpc.mem);
	if (res)
		release_mem_region(res->start, RESSIZE(res));

	if (dpc.debug)
		debugfs_remove_recursive(dpc.debug);

	return 0;
}

static struct platform_driver lf1000_dpc_driver = {
	.probe	= lf1000_dpc_probe,
	.remove	= lf1000_dpc_remove,
	.driver = {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init lf1000_dpc_init(void)
{
	return platform_driver_register(&lf1000_dpc_driver);
}

static void __exit lf1000_dpc_cleanup(void)
{
	platform_driver_unregister(&lf1000_dpc_driver);
}

module_init(lf1000_dpc_init);
module_exit(lf1000_dpc_cleanup);
MODULE_DESCRIPTION("LF1000 Display Controller Driver");
MODULE_AUTHOR("Andrey Yurovsky");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
