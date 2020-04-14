/*
 *  drivers/mtd/nand/lf1000.c
 *
 * Copyright 2007-2010 LeapFrog Enterprises Inc.
 *
 * Scott Esters <sesters@leapfrog.com>
 * Robert T. Dowling <rdowling@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <mach/platform.h>
#include <mach/common.h>
#include <mach/nand.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include "../ubi/ubi-media.h"
#include <linux/mtd/nand_ecc.h>

/*
 * For devices which display every fart in the system on a separate LED. Is
 * compiled away when LED support is disabled.
 */
DEFINE_LED_TRIGGER(nand_led_trigger);


#ifdef CPU_LF1000

    /* another type of ECC, needed for the Micron MT29F4G08ABADA */
#define NAND_ECC_INTERNAL   ((int)NAND_ECC_HW_SYNDROME + 1)

#define MAX_NUM_ERASE_BLOCKS    4096
static u32 block_erase_counts[MAX_NUM_ERASE_BLOCKS];
static u32 block_read_counts[MAX_NUM_ERASE_BLOCKS];
static u32 block_write_counts[MAX_NUM_ERASE_BLOCKS];
static u32 total_erases;
static u32 total_reads;
static u32 total_writes;
static u32 total_bitflips;

#define MAX_ECC_BYTES_PER_PAGE	(56)		
	/* this is for 4KB page with 4-bit ECC / 512 bytes */

static int  lf1000_cart_remove(void);
static int  lf1000_init_cart(u32 nand_base);
static void lf1000_init_for_MLC_nand(struct mtd_info *mtd, 
                                     struct nand_chip *chip);
static void lf1000_init_for_SLC_nand(struct mtd_info *mtd, 
                                     struct nand_chip *chip);
static void lf1000_init_mtd_info(struct mtd_info **ppmtd_info, u32 nand_base);
static void lf1000_prepare_for_4BitEcc( struct nand_chip * pchip);
static int  lf1000_nand_switch_ecc_mode(struct mtd_info *mtd, int ecc_mode);
static int lf1000_nand_scan(struct mtd_info * mtd, 
			    int		      maxchips, 
			    u32		    * p_nand_props, 
			    int		      cart_nand);
static int  lf1000_nand_scan_tail(struct mtd_info *mtd);
static int lf1000_verify_SLC_page(struct mtd_info *mtd, 
				  const uint8_t *buf, int len);

static int DisableInternalECC(struct mtd_info *mtd, 
		 	      struct nand_chip *chip,
			      u32 * p_nand_props,
			      int cart_nand);
static int EnableInternalECC(struct mtd_info *mtd, 
		 	     struct nand_chip *chip,
			     u32 * p_nand_props,
			     int cart_nand);
/* Here are prototypes of MTD Interface functions defined in nand_base.c */

int  nand_block_isbad(struct mtd_info *mtd, loff_t offs);
int  nand_block_markbad(struct mtd_info *mtd, loff_t ofs);
int  nand_do_read_oob(struct mtd_info *mtd, loff_t from,
                      struct mtd_oob_ops *ops);
uint8_t *nand_transfer_oob(struct nand_chip *chip, uint8_t *oob,
         				   struct mtd_oob_ops *ops, size_t len);
void nand_resume(struct mtd_info *mtd);
int  nand_suspend(struct mtd_info *mtd);
void nand_sync(struct mtd_info *mtd);
int  nand_write(struct mtd_info *mtd, loff_t to, size_t len,
			    size_t *retlen, const uint8_t *buf);
int  nand_write_oob(struct mtd_info *mtd, loff_t to,
    			    struct mtd_oob_ops *ops);
int  nand_read_subpage(struct mtd_info  *mtd, 
                       struct nand_chip *chip, 
                       uint32_t          data_offs, 
                       uint32_t          readlen, 
                       uint8_t          *bufpoi);


int  nand_get_device(struct nand_chip *chip, struct mtd_info *mtd,
			   		 int new_state);
void nand_release_device(struct mtd_info *mtd);
int  nand_do_write_oob(struct mtd_info *mtd, loff_t to,
			     		struct mtd_oob_ops *ops);
int  nand_check_wp(struct mtd_info *mtd);
int  nand_block_checkbad(struct mtd_info *mtd, loff_t ofs, int getchip,
			             int allowbbt);
int  nand_read_oob_std(struct mtd_info *mtd, struct nand_chip *chip,
			           int page, int sndcmd);
int  nand_write_oob_std(struct mtd_info *mtd, struct nand_chip *chip, int page);
void nand_write_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
        				 const uint8_t *buf);
int  nand_read_page_raw(struct mtd_info *mtd, struct nand_chip *chip,
    			        uint8_t *buf, int page);

int nand_read_page_swecc(struct mtd_info *mtd, struct nand_chip *chip,
				uint8_t *buf, int page);

/* nand_oob_8/16/64/128 are defined in nand_base.c */
extern struct nand_ecclayout nand_oob_8;
extern struct nand_ecclayout nand_oob_16;
extern struct nand_ecclayout nand_oob_64;
extern struct nand_ecclayout nand_oob_128;


/* Enable the following #define if you want to collect nand access timing data*/
//#define NAND_ACCESS_TIMING 1

#ifdef NAND_ACCESS_TIMING
#include <mach/platform.h>
#include <mach/common.h>
#include <mach/timer.h>


/* If you want to collect timing data for reads from nand, enable the
 * following #define, which enables timing in lf1000_nand_do_read_ops()
 */
#define TIME_NAND_READ_ENTIRE  1 

/* If you want to collect timing data for writes to nand, enable one of the
 * next two #defines.
 *
 * If TIME_NAND_WRITE_PARTS is defined, timing is collected separately for
 * calculation of ECC and for actual write to nand.
 * If TIME_NAND_WRITE_ENTIRE is defined, timing is collected for the entire
 * calculate/write process.
 * If both symbols are defined, the reported number of writes will increase by 2
 * and the reported total times will not be correct.
 */
//#define TIME_NAND_WRITE_PARTS   1
#define TIME_NAND_WRITE_ENTIRE  1


#define TIMER32(r)	REG32(IO_ADDRESS(TIMER_BASE+r))

/* 
 * settings 
 */
#define TIMER_BASE	LF1000_TIMER3_BASE

#define TIMER_CLK_SRC	1
#define TIMER_PRES	17			/* divide by 18 */
#define TIMER_CLK	2			/* divide by 8 */

/*
 * Timer Clock Rate:
 *
 * TIMER_SRC_RATE/(TIMER_PRES+1)/(TIMER_CLK+1) =
 *
 * 147461538/18/8 = 1024038 Hz, ~ 1.024MHz, ~1us tick
 */

/* start the stopwatch */
void timer_start(void)
{
	/* make sure the timer is stopped */
	BIT_CLR(TIMER32(TMRCONTROL), RUN);

	/* zero out the timer */
	TIMER32(TMRCOUNT) = 0;

	/* run the timer */
	BIT_SET(TIMER32(TMRCONTROL), RUN);
}

/* stop the stopwatch, and return the time */
u32 timer_stop(void)
{
	BIT_CLR(TIMER32(TMRCONTROL), RUN);	/* stop the timer */
	BIT_SET(TIMER32(TMRCONTROL), LDCNT);	/* get access to counter */
	return TIMER32(TMRCOUNT);		/* and return it */
}

u32 nand_num_reads;
u32 nand_read_time;
u32 nand_read_calc_time;
u32 nand_read_check_time;
u32 nand_num_writes;
u32 nand_write_time;
u32 nand_write_calc_time;

#endif  /* NAND_ACCESS_TIMING */



/* control registers */
#define MCU_Y_BASE	IO_ADDRESS(LF1000_MCU_Y_BASE)
#define NAND_BASE	IO_ADDRESS(LF1000_MCU_S_BASE)

/*
 * Private device structure
 *
 *  mtd_onboard points to info about the base NAND flash
 *  mtd_cart    points to info about the cartridge NAND flash
 *  controller is the control structure for the LF1000's NAND hardware controller
 */

struct lf1000_nand_device {
	void __iomem	       * mem;
	struct mtd_info        * mtd_onboard;
	struct mtd_info        * mtd_cart;
	struct platform_device * pdev;

	struct nand_hw_control   controller;
	int                      cart_ready;
	int                      cart_ubi;
	struct mutex             sem_hotswap;

	u32			 base_nand_props;	
	u32			 cart_nand_props;	
#define NAND_SUPPORTS_INTERNAL_ECC	1
#define NAND_INTERNAL_ECC_ENABLED	2
#define NAND_SUPPORTS_ONFI		4

#define NAND_INTERNAL_ECC_ENABLED_SHIFT	1
#define NAND_SUPPORTS_ONFI_SHIFT	2
};

static struct lf1000_nand_device nand = {
	.mem         = NULL,
	.mtd_onboard = NULL,
	.mtd_cart    = NULL,
	.cart_ready  = 0,
	.cart_ubi    = 0,
	.base_nand_props = 0,
	.cart_nand_props = 0
};

/*
 * Define partitions for flash devices
 */

#ifdef CONFIG_MTD_PARTITIONS
static const char * part_probes[] = { "cmdlinepart", NULL };
#endif

/* We might use a config parameter for this
 *#define LF_ERASE_BLK CONFIG_NAND_LF1000_MAX_ERASEBLK_SIZE
 *
 * for now, just make it 512KB, big enough for the MLC NAND chip we're using
 * (the symbol is used only for validating the partition sizes a few lines
 *  below)
 */
#define LF_ERASE_BLK 0x80000

/* Just shortening the names for clearer code */
#define LF_P0 (CONFIG_NAND_LF1000_P0_SIZE)
#define LF_P1 (CONFIG_NAND_LF1000_P1_SIZE)
#define LF_P2 (CONFIG_NAND_LF1000_P2_SIZE)
#define LF_P3 (CONFIG_NAND_LF1000_P3_SIZE)

#if ((LF_P0 % LF_ERASE_BLK) || (LF_P1 % LF_ERASE_BLK) || \
     (LF_P2 % LF_ERASE_BLK) || (LF_P3 % LF_ERASE_BLK))
#error "NAND partitions must be multiple of erase block."
#endif


static struct mtd_partition partition_info[] = {
  	{ .name		= "Emerald_Boot",
  	  .offset	= 0,
 	  .size		= LF_P0},
  	{ .name		= "Kernel",
 	  .offset	= LF_P0,
 	  .size		= LF_P1},
  	{ .name		= "RFS",
 	  .offset	= LF_P0 + LF_P1,
 	  .size		= LF_P2 },
  	{ .name		= "Data",
 	  .offset	= LF_P0 + LF_P1 + LF_P2,
 	  .size		= MTDPART_SIZ_FULL },
};

static struct mtd_partition partition_info_recovery[] = {
  	{ .name		= "Base",
  	  .offset	= 0,
 	  .size		= MTDPART_SIZ_FULL },
};

static struct mtd_partition partition_info_cart[] = {
	{ .name		= "Cartridge",
	  .offset	= 0,
 	  .size		= MTDPART_SIZ_FULL },
};

#undef LF_P0
#undef LF_P1
#undef LF_P2
#undef LF_P3
#undef LF_ERASE_BLK

/*******************
 * sysfs Interface *
 *******************/

/* Create a new MTD device as a subset, a slice of another device */
static ssize_t set_new_mtd( struct device           *dev, 
                            struct device_attribute *attr, 
                            const char              *buf, 
                            size_t                   count)
{
	int old_mtd_num;
	u32 offset, size;
	char tag[64];

	struct mtd_info *mtd;
	struct nand_chip *chip;
	
	static struct mtd_partition new_part[1] = { { NULL } };
	
	if (sscanf(buf, "%d %x %x %63s", &old_mtd_num, &offset, &size, tag) != 4)
		return -EINVAL;

	/* Nice if we could convert old_mtd_num to "1=cart, 0=base" type number */
	mtd = old_mtd_num ? nand.mtd_cart : nand.mtd_onboard;
	chip = mtd->priv;

	new_part[0].offset = offset;
	new_part[0].size = size;
	tag[63] = 0;
	new_part[0].name = kmalloc(1+strlen(tag), GFP_KERNEL);
	strcpy (new_part[0].name, tag);
	if (chip) {
		spin_lock(&chip->controller->lock);
		add_mtd_partitions(mtd, new_part, 1);
		spin_unlock(&chip->controller->lock);
		dev_alert(dev, "New mtd device '%s' offset=0x%x size=0x%x "
	                       "created on %s.\n", 
				tag, offset, size, 
				old_mtd_num ? "cart" : "onboard");
	}
	else {
		dev_alert(dev, "No MTD device found\n");
	}
	return count;
}

static DEVICE_ATTR(new_mtd, S_IWUSR|S_IWGRP|S_IWOTH, NULL, set_new_mtd);

typedef const volatile void __iomem * tpIO;

static ssize_t show_ramsize(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	uint size;

	size = (readw((tpIO)(MCU_Y_BASE + MEMCFG)) >> SDRCAP) & 3;
	switch(size) {
		case SDRCAP_64MBIT:	return(sprintf(buf, "%s\n",  "8MB"));
		case SDRCAP_128MBIT:	return(sprintf(buf, "%s\n", "16MB"));
		case SDRCAP_256MBIT:	return(sprintf(buf, "%s\n", "32MB"));
		case SDRCAP_512MBIT:	return(sprintf(buf, "%s\n", "64MB"));
		default:		return sprintf(buf, "unknown\n");
	}
}
static DEVICE_ATTR(ramsize, S_IRUSR|S_IRGRP|S_IROTH, show_ramsize, NULL);

#if defined CONFIG_MTD_NAND_LF1000_DEBUG
static ssize_t show_nand_timing(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	u32 tmp;

	tmp = readl((tpIO)(NAND_BASE+MEMTIMEACS));
	len += sprintf(buf+len, "0: ACS = %d\n", ((tmp>>22) & 0x3));
	tmp = readl((tpIO)(NAND_BASE+MEMTIMECOS));
	len += sprintf(buf+len, "1: COS = %d\n", ((tmp>>22) & 0x3));
	tmp = readl((tpIO)(NAND_BASE+MEMTIMEACCH));
	len += sprintf(buf+len, "2: ACC = %d\n", ((tmp>>12) & 0xF));
	tmp = readl((tpIO)(NAND_BASE+MEMTIMECOH));
	len += sprintf(buf+len, "3: COH = %d\n", ((tmp>>22) & 0x3));	
	tmp = readl((tpIO)(NAND_BASE+MEMTIMECAH));
	len += sprintf(buf+len, "4: CAH = %d\n", ((tmp>>22) & 0x3));	

	return len;
}

static ssize_t set_nand_timing(	struct device *dev,
				struct device_attribute *attr, 
				const char *buf, size_t count)
{
	unsigned int index, value;
	u32 tmp;

	if(sscanf(buf, "%u,%u", &index, &value) != 2)
		return -EINVAL;
	
	switch(index) {
		case 0: /* ACS */
		tmp = readl((tpIO)(NAND_BASE+MEMTIMEACS)) & ~(0x3<<22);
		tmp |= ((0x3 & value)<<22);
		writel(tmp, (tpIO)(NAND_BASE+MEMTIMEACS));
		break;
		case 1: /* COS */
		tmp =  readl((tpIO)(NAND_BASE+MEMTIMECOS)) & ~(0x3<<22);
		tmp |= ((0x3 & value)<<22);
		writel(tmp, (tpIO)(NAND_BASE+MEMTIMECOS));
		break;
		case 2: /* ACC */
		tmp = readl((tpIO)(NAND_BASE+MEMTIMEACCH)) & ~(0xF<<12);
		tmp |= ((0xF & value)<<12);
		writel(tmp, (tpIO)(NAND_BASE+MEMTIMEACCH));
		break;
		case 3: /* COH */   // TODO: repo shifts by 21
		tmp = readl((tpIO)(NAND_BASE+MEMTIMECOH)) & ~(0x3<<22);
		tmp |= ((0x3 & value)<<22);
		writel(tmp, (tpIO)(NAND_BASE+MEMTIMECOH));
		break;
		case 4: /* CAH */
		tmp = readl((tpIO)(NAND_BASE+MEMTIMECAH)) & ~(0x3<<22);
		tmp |= ((0x3 & value)<<22);
		writel(tmp, (tpIO)(NAND_BASE+MEMTIMECAH));
		break;
		default:
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(timing, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH, 
		show_nand_timing, set_nand_timing);
#endif


#ifdef CONFIG_MTD_NAND_LF1000_PROF
#include "prof.h"
static unsigned long ws_n[NS_MAX], ws_sum[NS_MAX], ws_min[NS_MAX], ws_max[NS_MAX];
static long ws_start[NS_MAX];
static int ws_any = 0;

/* From the timer module; 15 minutes worth of ticks at 4.593MHz fits in 32 bits */
extern int read_current_timer(unsigned long *timer_value);

void nand_stats_erase (void)
{
	int i;
	for (i=0; i<NS_MAX; i++)
		ws_n[i] = ws_sum[i] = 0;
}

/* Accumulate a start (in=1) or stop (in=0) time for a given type of access */
void nand_stats_accum (enum prof_type type, int in)
{
	long stop, delta;
	if (type >= NS_MAX) {
		dev_alert(&nand.pdev->dev,
				"nand_stats_accum: type=%d > NS_MAX", type);
		return;
	}
	if (!ws_any) {
		/* First time through, erase stats  */
		nand_stats_erase ();
		ws_any = 1;
	}
	read_current_timer ((unsigned long *)&stop);
	if (in) {
		ws_start[type] = stop;
	} else {
		delta = stop - ws_start[type];
		ws_sum[type] += delta;
		if (!ws_n[type]) {
			/* First data point, set min and max */
			ws_min[type] = ws_max[type] = delta;
		} else {
			if (ws_min[type] > delta)
				ws_min[type] = delta;
			if (ws_max[type] < delta)
				ws_max[type] = delta;
		}
		ws_n[type]++;
	}
}

static ssize_t show_write_stats(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	int x=0, i;
	static char *title[] = {"Read ", "Write", "Erase", "Lock "};
	for (i=0; i<NS_MAX; i++) {
		if (ws_n[i]) {
			x += sprintf (buf+x, "%s N=%ld %ld/%ld/%ld\n", 
				      title[i], ws_n[i], ws_min[i], 
					ws_sum[i]/ws_n[i], ws_max[i]);
		} else {
			x += sprintf (buf+x, "%s N=%ld %ld/%ld/%ld\n", 
					title[i], 0L,0L,0L,0L);
		}
	}
	return x;
}

static ssize_t clear_write_stats(struct device *dev, 
				 struct device_attribute *attr, 
				 const char *buf, 
				 size_t count)
{
	nand_stats_erase ();
	return count;
}

/* Write to this sysfs device:	clear stats.  
 * Read:			dump out stats.  
 * See profnand.c 
 */
static DEVICE_ATTR(write, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH, 
		   show_write_stats, clear_write_stats);

#endif
/* End #ifdef CONFIG_MTD_NAND_LF1000_PROF */


static ssize_t show_nand_accesses(struct device *dev, 
				  struct device_attribute *attr, char *buf)
{
	int x;

	x = sprintf (buf, "NAND accesses: page reads %d, "
			  "page writes %d, block erasures %d, bitflips %d\n", 
		     total_reads, total_writes, total_erases, total_bitflips);
	return x;
}


static ssize_t clear_nand_accesses(struct device *dev, 
				   struct device_attribute *attr, 
				   const char *buf, size_t count)
{
    total_erases   = 0;
    total_reads    = 0;
    total_writes   = 0;
    total_bitflips = 0;
	return count;
}

static DEVICE_ATTR(nand_accesses, 
		   S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH, 	
		   show_nand_accesses, clear_nand_accesses);

#ifdef CONFIG_MTD_NAND_LF1000_READ_DELAY
static volatile int read_delay = 0;

static ssize_t show_read_delay(	struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	return sprintf (buf, "%d\n", read_delay);
}

static ssize_t set_read_delay(struct device *dev, 
			      struct device_attribute *attr, 
			      const char *buf, size_t count)
{
	if(sscanf(buf, "%d", &read_delay) != 1)
		return -EINVAL;
	return count;
}

static DEVICE_ATTR(read_delay, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
		   show_read_delay, set_read_delay);
#endif /* ifdef CONFIG_MTD_NAND_LF1000_READ_DELAY */

#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
volatile int nand_stress = 0;

static ssize_t show_nand_stress(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	return sprintf (buf, "%d\n", nand_stress);
}

static ssize_t set_nand_stress(	struct device *dev, 
				struct device_attribute *attr, 
				const char *buf, size_t count)
{
	int ret, value;

	ret = get_option(&buf, &value);

	if(ret != 1)
		return -EINVAL;

	nand_stress = value;

	/*
	 * Restore any severed connections
	 */
	if(IS_CLR(nand_stress, CART_READ)) {
		stress_cut_cart(0);
	}

	return count;
}

static DEVICE_ATTR(stress, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
		   show_nand_stress, set_nand_stress);

#endif /* ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST */

static ssize_t show_cart_ecc_mode(struct device *dev, 
				  struct device_attribute *attr, char *buf)
{
	int cart_ecc_mode = -1;
	
	if (nand.mtd_cart) {
		struct nand_chip *cart;
		cart = (struct nand_chip *) (&nand.mtd_cart[1]);
		if (cart) {
			switch (cart->ecc.mode)
			{
			case NAND_ECC_NONE: 
			case NAND_ECC_SOFT: 
			case NAND_ECC_HW_SYNDROME:
			case NAND_ECC_INTERNAL:	
				cart_ecc_mode = cart->ecc.mode;
				break;
			default: cart_ecc_mode = -1; break;
			}
		} else {
			dev_alert(dev, "Funky cartridge device\n");
		}
	} else {
		dev_alert(dev, "No cartridge device\n");
	}
	return sprintf (buf, "%d\n", cart_ecc_mode);
}

static ssize_t set_cart_ecc_mode(struct device           *dev, 
                                 struct device_attribute *attr, 
                                 const char              *buf, 
                                 size_t                   count)
{
	int cart_ecc_mode;
	int ecc_mode = -1;
	if(sscanf(buf, "%d", &cart_ecc_mode) != 1)
		return -EINVAL;
	if (nand.mtd_cart) {
		struct mtd_info *mtd = nand.mtd_cart;
		struct nand_chip *cart;
		cart = (struct nand_chip *) (&nand.mtd_cart[1]);
		if (cart) {
			switch (cart_ecc_mode) {
			case 0: ecc_mode = NAND_ECC_NONE; break;
			case 1: ecc_mode = NAND_ECC_SOFT; break;
			case 3: ecc_mode = NAND_ECC_HW_SYNDROME; break;
			case 4: 
				if (nand.cart_nand_props 
					& NAND_SUPPORTS_INTERNAL_ECC) {
					ecc_mode = NAND_ECC_INTERNAL; 
				}
				else {
					dev_alert(dev, "nand doesn't support "
							"internal ecc\n"); 
				}
				break;
			default: 
				dev_alert(dev, "Bad ecc_mode %d\n", cart_ecc_mode); 
				break;
			}
			if (ecc_mode != -1) {
				/* Delete cart parts and reinstate them */
				struct nand_chip *chip = mtd->priv;
				int cart_parts_nb = 0;
				struct mtd_partition *cart_parts = NULL;
				int switch_ecc = 0;

				if (ecc_mode == chip->ecc.mode) {
					dev_alert(dev, 
						"Cart already in ecc_mode %d\n",
						 cart_ecc_mode); 
				}
				else if (ecc_mode == NAND_ECC_INTERNAL) {
					if (!(nand.cart_nand_props 
						& NAND_SUPPORTS_INTERNAL_ECC)) {
						dev_alert(dev,"Cart nand cannot"
							"support internal ecc\n"); 
					}
					else if (EnableInternalECC( mtd, chip,
							&nand.cart_nand_props, 1))
					{
						switch_ecc = 1;
					}
					else {
						dev_alert(dev,"Failed to enable"
							     " internal ecc\n"); 
					}
				}
				else {
					if (nand.cart_nand_props 
						& NAND_INTERNAL_ECC_ENABLED) {
						if (DisableInternalECC( mtd, chip,
							&nand.cart_nand_props,1))
						{
							switch_ecc = 1;
						}
						else {
							dev_alert(dev,
							     "Failed to disable"
							     " internal ecc\n"); 
						}
					}
					else {
						switch_ecc = 1;
					}
				}
				if (switch_ecc) {
					cart_parts = partition_info_cart;
					cart_parts_nb = 
						ARRAY_SIZE(partition_info_cart);
					spin_lock(&chip->controller->lock);
					del_mtd_partitions(mtd);
					lf1000_nand_switch_ecc_mode(mtd, 
								    ecc_mode);
					chip->scan_bbt(mtd);
					add_mtd_partitions(nand.mtd_cart, 
						     cart_parts, cart_parts_nb);
					spin_unlock(&chip->controller->lock);
				}
			}
		} else {
			dev_alert(dev, "Funky cartridge device\n");
		}
	}
	else {
		dev_alert(dev, "No cartridge device\n");
	}
	return count;
}

static DEVICE_ATTR(cart_ecc_mode, 
		   S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
		   show_cart_ecc_mode, set_cart_ecc_mode);


#ifdef CONFIG_MTD_NAND_LF1000_HOTSWAP
static ssize_t get_cart_hotswap_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ready = 0;

	/* first make sure mtd_cart is allocated */
	if(nand.mtd_cart != NULL) {
		ready = nand.cart_ready;
	}
	return sprintf (buf, "%d\t%d\n", ready, nand.cart_ubi);
}

static ssize_t set_cart_hotswap_state(struct device *dev, 
				      struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	struct resource *res;
	int cart_parts_nb = 0;
	int hotswap_state;
	struct platform_device *pdev__ ;	
	struct mtd_partition *cart_parts = NULL;
	size_t nread;
	uint32_t magic;
	ssize_t ret = count;

	if(sscanf(buf, "%d", &hotswap_state) != 1)
		return -EINVAL;

	pdev__= to_platform_device(dev);
	res = platform_get_resource(pdev__, IORESOURCE_MEM, 0);
	if(!res) {
		dev_err(dev, "nand: failed to get resource!\n");
		return -ENXIO;
	}

	if (down_interruptible(&nand.sem_hotswap))
		return -ERESTARTSYS;
	
	/* cart is inserted */
	if(hotswap_state) {
		
		/* check if a cartridge is inserted */
		gpio_configure_pin(NAND_CART_DETECT_PORT, NAND_CART_DETECT_PIN,
                		   GPIO_GPIOFN, 0, 1, 0);
		if (gpio_get_val(NAND_CART_DETECT_PORT, NAND_CART_DETECT_PIN) 
		    != NAND_CART_DETECT_LEVEL) {
			dev_err(dev, "cartridge insertion "
				     "can't be confirmed by driver\n");
			ret = -EAGAIN;
			goto out;
		} else {
			int i = 0;
			int scan;

			dev_info(dev, "cartridge inserted\n");
			
			if(nand.cart_ready == 1){
				dev_err(dev, "cartridge driver was ready\n");
				goto out;
			}
			
			if(lf1000_init_cart(res->start)) {
				nand.cart_ready = -1;
				if(nand.mtd_cart)
					kfree(nand.mtd_cart);
				ret = -EPERM;
				goto out;
			}
			
			do {
				scan = lf1000_nand_scan(nand.mtd_cart, 1,
							&nand.cart_nand_props,1);
			} while (scan && ++i < 4);

			if (i > 1)
				dev_info(dev, 
					"tried to scan cartridge %d times\n",i);
			
			if (scan) {
				nand.cart_ready = -1;
				dev_err(dev, "cartridge inserted, "
					     "but NAND not detected !\n");
				nand_release(nand.mtd_cart);
				kfree(nand.mtd_cart);
				nand.mtd_cart = NULL;
				ret = -EPERM;
				goto out;
			}
			
#ifdef CONFIG_MTD_PARTITIONS
			nand.mtd_cart->name = "lf1000-cart";
			cart_parts_nb = parse_mtd_partitions(nand.mtd_cart,
							     part_probes,
							     &cart_parts, 0);
#endif
			if (cart_parts_nb == 0) {
				cart_parts = partition_info_cart;
				cart_parts_nb = ARRAY_SIZE(partition_info_cart);
			}

			/* Register the cartridge partitions, if it exists */
			add_mtd_partitions(nand.mtd_cart, cart_parts, cart_parts_nb);

			nand.mtd_cart->read(nand.mtd_cart, 0, sizeof(uint32_t),
					   &nread, (void *)&magic);
			magic = be32_to_cpu(magic);
			if (magic == UBI_EC_HDR_MAGIC) {
				nand.cart_ubi=1;
				dev_info(dev, 
					"cartridge has UBI layer, nread=%d\n",
					 nread);
			} else {
				nand.cart_ubi=0;
				dev_info(dev,"cartridge has no UBI, nread=%d\n",
					 nread);
			}
			
			/* Error checking */
			if (   (gpio_get_val(NAND_CART_TYPE_PORT,
					     NAND_CART_TYPE_EMERALD) == 0) 
			    && (nand.cart_ubi==1))
			{
				dev_err(dev, "cart has UBI layer, "
					     "but is using Emerald Cart ID\n");
			} else if ((gpio_get_val(NAND_CART_TYPE_PORT,
						 NAND_CART_TYPE_EMERALD) == 1)
				   && (nand.cart_ubi==0)){
				dev_err(dev, "cart ID is Didj, but "
					     "no UBI layer found !!\n");
			}
			nand.cart_ready = 1;
			dev_info(dev, "cart driver ready !\n");
		}
	} else {  /* cart is removed */
		nand.cart_ready = 0;
		
		lf1000_cart_remove();
		
		dev_info(dev, "cartridge removed !\n");		
	}
out:	
	up(&nand.sem_hotswap);	
	
	return ret;
}
static DEVICE_ATTR(cart_hotswap, 
		   S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
		   get_cart_hotswap_state, set_cart_hotswap_state);
#endif

static ssize_t get_base_nand_internal_ecc_support(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	u32 tmp;

	tmp = nand.base_nand_props & NAND_SUPPORTS_INTERNAL_ECC;
	return sprintf(buf, "%d\n", tmp);
}

static DEVICE_ATTR(base_nand_internal_ecc_support, 
		   S_IRUSR|S_IRGRP|S_IROTH,
		   get_base_nand_internal_ecc_support, NULL);


static ssize_t get_base_nand_internal_ecc_enabled(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	u32 tmp;

	tmp = (nand.base_nand_props & NAND_INTERNAL_ECC_ENABLED)
		>> NAND_INTERNAL_ECC_ENABLED_SHIFT;
	return sprintf(buf, "%d\n", tmp);
}

static DEVICE_ATTR(base_nand_internal_ecc_enabled, 
		   S_IRUSR|S_IRGRP|S_IROTH,
		   get_base_nand_internal_ecc_enabled, NULL);


static ssize_t get_base_nand_ecc_mode(struct device *dev, 
				struct device_attribute *attr, char *buf)
{
	struct nand_chip *chip;
	u32    base_ecc_mode;

	chip = (struct nand_chip *)nand.mtd_onboard->priv;
	switch (chip->ecc.mode)
	{
	case NAND_ECC_NONE: 
	case NAND_ECC_SOFT: 
	case NAND_ECC_HW_SYNDROME:
	case NAND_ECC_INTERNAL:	
		base_ecc_mode = chip->ecc.mode;
		break;
	default: 
		base_ecc_mode = -1;
		break;
	}
	return sprintf (buf, "%d\n", base_ecc_mode);
}

static DEVICE_ATTR(base_nand_ecc_mode, 
		   S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
		   get_base_nand_ecc_mode, NULL);

/******************************************************************************/

static struct attribute *nand_attributes[] = {
	&dev_attr_new_mtd.attr,
	&dev_attr_ramsize.attr,
#if defined CONFIG_MTD_NAND_LF1000_DEBUG
	&dev_attr_timing.attr,
#endif
#ifdef CONFIG_MTD_NAND_LF1000_PROF
	&dev_attr_write.attr,
#endif
#ifdef CONFIG_MTD_NAND_LF1000_READ_DELAY
	&dev_attr_read_delay.attr,
#endif
#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
	&dev_attr_stress.attr,
#endif

#ifdef CONFIG_MTD_NAND_LF1000_HOTSWAP
	&dev_attr_cart_hotswap.attr,
#endif	
	&dev_attr_nand_accesses.attr,
	&dev_attr_base_nand_internal_ecc_support.attr,
	&dev_attr_base_nand_internal_ecc_enabled.attr,
	&dev_attr_base_nand_ecc_mode.attr,
	NULL
};

static struct attribute_group nand_attr_group = {
	.attrs = nand_attributes
};

/*
 * There are 3 ways to test ready/busy bit:
 * 1) test the RnB bit in NFCONTROL (used here)
 * 2) test the IRQPEND bit in NFCONTROL and then set it to clear the interrupt
 * 3) send a NAND_CMD_STATUS to then NAND chip, test the response against
 *    the mask 0x40
 */
static int lf1000_nand_ready(struct mtd_info *mtd)
{
	u32 ctl = readl((tpIO)(NAND_BASE+NFCONTROL));

	if(IS_SET(ctl,RnB))
		return 1;	/* ready */
	return 0;		/* busy */
}

/*
 * hardware-specific access to control and address lines:
 * The LF1000's NAND controller handles the CLE and ALE signals automatically,
 * data must simply be written to the appropriate register: NFCMD or NFADDR
 * respectively.
 */
static void lf1000_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;

	if(cmd == NAND_CMD_NONE)
		return;

	if(ctrl & NAND_CLE) /* command */ {
		writeb(cmd, chip->IO_ADDR_W + NFCMD);
	}
	else if(ctrl & NAND_ALE) /* address */ {
		writeb(cmd, chip->IO_ADDR_W + NFADDR);
	}
}

static void lf1000_select_chip(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;
	u32 tmp;

	switch(chipnr) {
	case -1:	/* TODO: FIXME: no need to call chip->cmd_ctrl(): 
			 * it does nothing */
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, 0 | NAND_CTRL_CHANGE);
		break;
	case 0:
		tmp = readl((tpIO)(NAND_BASE+NFCONTROL));
		BIT_CLR(tmp, NFBANK);
		writel(tmp, (tpIO)(NAND_BASE+NFCONTROL));
		break;
	default:
		BUG();
	}
}

static void lf1000_select_cart(struct mtd_info *mtd, int chipnr)
{
	struct nand_chip *chip = mtd->priv;
	u32 tmp;

	switch(chipnr) {
	case -1:
		chip->cmd_ctrl(mtd, NAND_CMD_NONE, 0 | NAND_CTRL_CHANGE);
		break;
	case 0:
		tmp = readl((tpIO)(NAND_BASE+NFCONTROL));
		BIT_SET(tmp, NFBANK);
		writel(tmp, (tpIO)(NAND_BASE+NFCONTROL));
		break;
	default:
		BUG();
	}
}

static int lf1000_init_cart(u32 nand_base)
{
	struct nand_chip *cart;
	const char *s;

	lf1000_init_mtd_info(&nand.mtd_cart, nand_base);
	if(!nand.mtd_cart) {
		dev_err(&nand.pdev->dev, "Unable to allocate cart device\n");
		return -ENOMEM;
	}
	cart              = (struct nand_chip *)nand.mtd_cart->priv;
	cart->select_chip = lf1000_select_cart;

	if (! gpio_have_gpio_madrid() )
	{
		gpio_configure_pin( NAND_CART_TYPE_PORT, NAND_CART_TYPE_EMERALD,
					    GPIO_GPIOFN, 0, 1, 0);
		gpio_configure_pin( NAND_CART_TYPE_PORT, NAND_CART_TYPE_HIGH,
					    GPIO_GPIOFN, 0, 1, 0);
		if(gpio_get_val(NAND_CART_TYPE_PORT, NAND_CART_TYPE_HIGH) == 0) {
			dev_info(&nand.pdev->dev, "cartridge type: OTP\n");
			cart->ecc.mode = NAND_ECC_NONE;
		} else {
			dev_info(&nand.pdev->dev, "cartridge type: NAND\n");	
			cart->ecc.mode = NAND_ECC_SOFT;
		}
	}
	else
	{
		/* HACK: On Madrid, we can't check Cart Type pins, assume OTP for the time being */
		cart->ecc.mode = NAND_ECC_NONE;
	}

	/* report the ECC mode that we wound up with for the cartridge */
	switch(cart->ecc.mode) {
	case NAND_ECC_NONE:		s="none"; break;
	case NAND_ECC_SOFT:		s="software"; break;
	case NAND_ECC_HW_SYNDROME:	s="hardware syndrome"; break;
	default:			s="unknown"; break;
	}
	dev_info(&nand.pdev->dev, "cartridge ECC mode: %s\n", s);

	return 0;
}

static int lf1000_cart_remove(void)
{
	/* Release resources, unregister device */
	if(nand.mtd_cart)
		nand_release(nand.mtd_cart);
	
	if(nand.mtd_cart)
		kfree(nand.mtd_cart);
	
	nand.mtd_cart = NULL;
	
	return 0;
}

    /* returns 1 if all 'len' bytes at buf are 0xff;
     *         0 if at least one of the first 'len' bytes at buf is not 0xff
     */
static int all_bytes_ff( const uint8_t * buf, int len)
{
	int rem   = (3 & (unsigned int)buf);
	int allFF = 1;

	if (rem) {
		while ( (len > 0) && (rem < 4)) {
			if (*buf++ != 0xFF) {
				allFF = 0;
				break;
			}
			++rem;
			--len;
		}
	}
	if (allFF) {
		if (0 == (3 & (unsigned int)buf)) {
			u32 * p = (u32 *)buf;

			for ( ; len > 3; len -= 4) {
				if (*p++ != 0xFFFFFFFF) {
					allFF = 0;
					break;
				}
			}
			if (allFF) {
				for (buf = (uint8_t *)p; len > 0; --len) {
					if (*buf++ != 0xFF) {
						allFF = 0;
						break;
					}
				}
			}
		} else {
			dev_info(&nand.pdev->dev, "!@#$ all_bytes_ff()\n");
			for ( ; len > 0; --len) {
				if (*buf++ != 0xFF) {
					allFF = 0;
					break;
				}
			}
		}
	}
	return allFF;
}

/**
 * lf1000_nand_erase - [MTD Interface] erase block(s)
 * @mtd:	MTD device structure
 * @instr:	erase instruction
 *
 * Erase one or more blocks
 *
 * NOTE:    This routine is a merger of nand_base.c's nand_erase() and
 *          nand_erase_nand().  nand_erase() calls nand_erase_nand() with
 *  a 3rd argument (allowbbt) that's 0.  This merged code assumes that
 *  allowbbt is 0.
 *	    This routine exists only to remove the Leapfrog-specific code
 *  from nand_base.c.
 *
 * NOTE: nand_erase_nand() is also called from nand_bbt.c's write_bbt().
 *       Its erasures will not be added to the profile data.
 */
#define BBT_PAGE_MASK	0xffffff3f
static int lf1000_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int page, status, pages_per_block, ret, chipnr;
	struct nand_chip *chip = mtd->priv;
	loff_t rewrite_bbt[NAND_MAX_CHIPS]={0};
	unsigned int bbt_masked_page = 0xffffffff;
	loff_t len;
#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
	int power = 0;
#endif

	DEBUG(MTD_DEBUG_LEVEL3, "nand_erase: start = 0x%012llx, len = %llu\n",
	      (unsigned long long)instr->addr, (unsigned long long)instr->len);

	/* Start address must align on block boundary */
	if (instr->addr & ((1 << chip->phys_erase_shift) - 1)) {
		DEBUG(MTD_DEBUG_LEVEL0, "nand_erase: Unaligned address\n");
		return -EINVAL;
	}

	/* Length must align on block boundary */
	if (instr->len & ((1 << chip->phys_erase_shift) - 1)) {
		DEBUG(MTD_DEBUG_LEVEL0, "nand_erase: "
		      "Length not block aligned\n");
		return -EINVAL;
	}

	/* Do not allow erase past end of device */
	if ((instr->len + instr->addr) > mtd->size) {
		DEBUG(MTD_DEBUG_LEVEL0, "nand_erase: "
		      "Erase past end of device\n");
		return -EINVAL;
	}

	instr->fail_addr = MTD_FAIL_ADDR_UNKNOWN;

	/* Grab the lock and see if the device is available */
	nand_get_device(chip, mtd, FL_ERASING);

	/* Shift to get first page */
	page = (int)(instr->addr >> chip->page_shift);
	chipnr = (int)(instr->addr >> chip->chip_shift);

	/* Calculate pages in each block */
	pages_per_block = 1 << (chip->phys_erase_shift - chip->page_shift);

	/* Select the NAND device */
	chip->select_chip(mtd, chipnr);

	/* Check, if it is write protected */
	if (nand_check_wp(mtd)) {
		DEBUG(MTD_DEBUG_LEVEL0, "nand_erase: "
		      "Device is write protected!!!\n");
		instr->state = MTD_ERASE_FAILED;
		goto erase_exit;
	}

	/*
	 * If BBT requires refresh, set the BBT page mask to see if the BBT
	 * should be rewritten. Otherwise the mask is set to 0xffffffff which
	 * can not be matched. This is also done when the bbt is actually
	 * erased to avoid recusrsive updates
	 */
	/* next line has the original nand_erase_nand() line from nand_base.c: 
	 * if (chip->options & BBT_AUTO_REFRESH && !allowbbt)
	 */
	if (chip->options & BBT_AUTO_REFRESH)
		bbt_masked_page = chip->bbt_td->pages[chipnr] & BBT_PAGE_MASK;

	/* Loop through the pages */
	len = instr->len;

	instr->state = MTD_ERASING;

	while (len) {
		u32 eb_index;
		/*
		 * heck if we have a bad block, we do not erase bad blocks !
		 */
		/* here's the original nand_erase_nand() code from nand_base.c 
		 * if (nand_block_checkbad(mtd, ((loff_t) page) <<
		 *			chip->page_shift, 0, allowbbt)) 
		 */
		if (nand_block_checkbad(mtd, ((loff_t)page) << chip->page_shift, 
					0, 0)) 
	        {
			dev_warn(&nand.pdev->dev, "nand_erase: attempt to erase"
			       " a bad block at page 0x%08x\n", page);
			instr->state = MTD_ERASE_FAILED;
			goto erase_exit;
		}
		/*
		 * Invalidate the page cache, if we erase the block which
		 * contains the current cached page
		 */
		if (page <= chip->pagebuf 
		    && chip->pagebuf < (page + pages_per_block))
			chip->pagebuf = -1;

#ifdef CONFIG_MTD_NAND_LF1000_PROF
		nand_stats_accum (NS_ERASE, 1);
#endif

#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
		power = IS_SET(nand_stress, POWER_ERASE);
		if(power) {
			dev_emerg(&nand.pdev->dev, "nand_erase: cutting power"
			      " while erasing block %d (page 0x%08x)\n",
			      (page & chip->pagemask) / pages_per_block,
			      page & chip->pagemask );

			stress_config_power();
		}
#endif
		chip->erase_cmd(mtd, page & chip->pagemask);
		eb_index = (page & chip->pagemask) / pages_per_block;
		if (eb_index < MAX_NUM_ERASE_BLOCKS) {
			block_erase_counts[ eb_index ] += 1;
			total_erases++;
		}

#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
		if(power) {
			stress_cut_power();
		}
#endif
		status = chip->waitfunc(mtd, chip);
#ifdef CONFIG_MTD_NAND_LF1000_PROF
		nand_stats_accum (NS_ERASE, 0);
#endif
		/*
		 * See if operation failed and additional status checks are
		 * available
		 */
		if ((status & NAND_STATUS_FAIL) && (chip->errstat))
			status = chip->errstat(mtd, chip, FL_ERASING,
					       status, page);

		/* See if block erase succeeded */
		if (status & NAND_STATUS_FAIL) {
			DEBUG(MTD_DEBUG_LEVEL0, "nand_erase: "
			      "Failed erase, page 0x%08x\n", page);
			instr->state = MTD_ERASE_FAILED;
			instr->fail_addr =
				((loff_t)page << chip->page_shift);
			goto erase_exit;
		}

		/*
		 * If BBT requires refresh, set the BBT rewrite flag to the
		 * page being erased
		 */
		if (bbt_masked_page != 0xffffffff 
		    && (page & BBT_PAGE_MASK) == bbt_masked_page)
			rewrite_bbt[chipnr] =
					((loff_t)page << chip->page_shift);

		/* Increment page address and decrement length */
		len -= (1 << chip->phys_erase_shift);
		page += pages_per_block;

		/* Check, if we cross a chip boundary */
		if (len && !(page & chip->pagemask)) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
			/*
			 * If BBT requires refresh and BBT-PERCHIP, set the BBT
			 * page mask to see if this BBT should be rewritten
			 */
			if (bbt_masked_page != 0xffffffff 
			    && (chip->bbt_td->options & NAND_BBT_PERCHIP))
				bbt_masked_page = chip->bbt_td->pages[chipnr] 
						  & BBT_PAGE_MASK;
		}
	}
	instr->state = MTD_ERASE_DONE;

erase_exit:
	ret = instr->state == MTD_ERASE_DONE ? 0 : -EIO;

	/* Deselect and wake up anyone waiting on the device */
	nand_release_device(mtd);

	/* Do call back function */
	if (!ret)
		mtd_erase_callback(instr);
	/*
	 * If BBT requires refresh and erase was successful, rewrite any
	 * selected bad block tables
	 */
	if (bbt_masked_page == 0xffffffff || ret)
		return ret;

	for (chipnr = 0; chipnr < chip->numchips; chipnr++) {
		if (!rewrite_bbt[chipnr])
			continue;
		/* update the BBT for chip */
		DEBUG(MTD_DEBUG_LEVEL0, "nand_erase_nand: nand_update_bbt "
		      "(%d:0x%0llx 0x%0x)\n", chipnr, rewrite_bbt[chipnr],
		      chip->bbt_td->pages[chipnr]);
		nand_update_bbt(mtd, rewrite_bbt[chipnr]);
	}

	/* Return more or less happy */
	return ret;
}

static inline int num_zero_bits_u8(uint8_t val) {
	int count;
    
	if (val == 0) {
		count = 8;
	} else {
		for (val = ~val, count = 0; val; val >>= 1) {
			if (val & 1) {
				++count;
			}
		}        
	}
	return count;
}

static inline int num_zero_bits_u32(uint32_t val) {
	int count;
    
	if (val == 0) {
		count = 32;
	} else {
		for (val = ~val, count = 0; val; val >>= 1) {
			if (val & 1) {
				++count;
			}
		}        
	}
	return count;
}


/* 
 * NAND_SUBPAGE_READ(x) is defined this way
 * #define NAND_SUBPAGE_READ(chip) ((chip->ecc.mode == NAND_ECC_SOFT) \
 *                                       && (chip->page_shift > 9))
 * in linux-2.6/include/linux/mtd/nand.h.
 *
 * Because of this definition, NAND_SUBPAGE_READ(chip) is always false
 * when 'chip' refers to an MLC NAND flash.
 *
 * Since we store 7 BCH ECC bytes for each 512-byte subpage of an MLC
 * NAND flash, we can read subpages (in chunks that are multiples of
 * 512 bytes).
 *
 * Here we change the definition of NAND_SUBPAGE_READ(x) so it's
 * true for an MLC NAND flash. 
 * When the ecc mode is HW_SYNDROME (i.e., when the NAND is MLC) we also 
 * initialize ecc.read_subpage to point to lf1000_nand_read_subpage_BCH().
 * We also change the definition so NAND_SUBPAGE_READ(x) is true for
 * a Micron chip with internal ECC enabled.  
 */
#ifdef NAND_SUBPAGE_READ
#undef NAND_SUBPAGE_READ
#define NAND_SUBPAGE_READ(chip) \
                     (   (   (chip->ecc.mode == NAND_ECC_SOFT) \
                          && (chip->page_shift > 9)) \
                      || (   (chip->ecc.mode == NAND_ECC_HW_SYNDROME) \
                          && (chip->cellinfo & NAND_CI_CELLTYPE_MSK  )) \
		      ||     (chip->ecc.mode == NAND_ECC_INTERNAL))
#endif

#include "lf1000_MLC_BCH.c"
#include "lf1000_internal_ECC.c"

/**
 * lf1000_nand_do_read_ops - Leapfrog replacement for nand_do_read_ops()
 *
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @ops:	oob ops structure
 *
 * Internal function. Called with chip held.
 *
 * Differences from nand_do_read_ops():
 *   code for profiling
 *   code for counting and timing read operations
 *   use of CONFIG_MTD_NAND_LF1000_MLC_SCRUB_THRESHOLD
 *   calls to dev_info() in order to report errors.
 */
static int lf1000_nand_do_read_ops(struct mtd_info *mtd, loff_t from,
			           struct mtd_oob_ops *ops)
{
	int chipnr, page, realpage, col, bytes, aligned;
	struct nand_chip *chip = mtd->priv;
	struct mtd_ecc_stats stats;
	int blkcheck = (1 << (chip->phys_erase_shift - chip->page_shift)) - 1;
	int sndcmd = 1;
	int ret = 0;
	uint32_t readlen = ops->len;
	uint32_t oobreadlen = ops->ooblen;
	uint8_t *bufpoi, *oob, *buf;
	uint32_t numCorrected;

	stats	     = mtd->ecc_stats;
	numCorrected = stats.corrected;

	chipnr = (int)(from >> chip->chip_shift);
	chip->select_chip(mtd, chipnr);

	realpage = (int)(from >> chip->page_shift);
	page	 = realpage & chip->pagemask;

	col = (int)(from & (mtd->writesize - 1));

	buf = ops->datbuf;
	oob = ops->oobbuf;

	while(1) {
		u32 eb_index;

		bytes   = min(mtd->writesize - col, readlen);
		aligned = (bytes == mtd->writesize);

		/* Is the current page in the buffer ? */
		if (realpage != chip->pagebuf || oob) {
			bufpoi = aligned ? buf : chip->buffers->databuf;

#ifdef CONFIG_MTD_NAND_LF1000_PROF
			nand_stats_accum (NS_READ, 1);
#endif
			if (likely(sndcmd)) {
				/* if the nand has internal ECC and if we
				 * don't want to start reading at the beginning
				 * of a page, specify the starting offset.
				 * Specifying the offset here makes it 
				 * unnecessary for the read_subpage function
				 * to specify the offset.
				 */
				if (   (col > 0)
			 	    && (chip->ecc.mode == NAND_ECC_INTERNAL))
					chip->cmdfunc(mtd, NAND_CMD_READ0, 
						      col, page);
				else /* start at beginning of page */
					chip->cmdfunc(mtd, NAND_CMD_READ0, 
						      0x00, page);
				sndcmd = 0;
			}
			/* Now read the page into the buffer */
			if (unlikely(ops->mode == MTD_OOB_RAW))
				ret = chip->ecc.read_page_raw(mtd, chip, bufpoi, page);
			else if (!aligned && NAND_SUBPAGE_READ(chip) && !oob) {
				/*
				 * if the nand has internal ECC, read directly
				 * into the destination buffer.
				 */
				if (chip->ecc.mode == NAND_ECC_INTERNAL)
					bufpoi = buf;

				ret = chip->ecc.read_subpage(mtd, chip, col, 
							     bytes, bufpoi);
			} else {
#ifdef TIME_NAND_READ_ENTIRE
				++nand_num_reads;
				timer_start();
#endif
				ret = chip->ecc.read_page(mtd, chip, bufpoi, page);
#ifdef TIME_NAND_READ_ENTIRE
		                nand_read_time += timer_stop();
				/* enable this 'if' to reduce output
		                 * if (0 == (nand_num_reads % 100))
				 */
				dev_info(&nand.pdev->dev, "NandRead: %d, %d\n", 
				        nand_num_reads, nand_read_time);
#endif
			}
			eb_index = realpage / (blkcheck + 1);
			if (eb_index < MAX_NUM_ERASE_BLOCKS) {
				block_read_counts[ eb_index ] += 1;
				total_reads++;
			}
			if (ret < 0) {
                		dev_info(&nand.pdev->dev, 
					"Error reading %d bytes from page %d\n", 
		                        bytes, realpage);
				break;
			}
			else if ( mtd->ecc_stats.corrected != numCorrected ) {
			        numCorrected = mtd->ecc_stats.corrected;
			        dev_info(&nand.pdev->dev, 
					"Corrected error while reading %d bytes "
		                        "at offset %d from page %d\n",
		               		bytes, col, realpage);
		    	}

			/* Transfer not aligned data */
			if (!aligned) {
				if (!NAND_SUBPAGE_READ(chip) && !oob)
					chip->pagebuf = realpage;
				/* if the nand doesn't have internal ECC
				 * (so we didn't read directly into the
				 *  destination buffer), copy the requested
				 *  bytes to the destination buffer.
				 */
				if (chip->ecc.mode != NAND_ECC_INTERNAL)
					memcpy(buf, chip->buffers->databuf + col,
					       bytes);
			}
			buf += bytes;

			if (unlikely(oob)) {
				/* Raw mode does data:oob:data:oob */
				if (ops->mode != MTD_OOB_RAW) {
					int toread = min(oobreadlen,
						    chip->ecc.layout->oobavail);
					if (toread) {
						oob = nand_transfer_oob(chip,
							oob, ops, toread);
						oobreadlen -= toread;
					}
				} else
					buf = nand_transfer_oob(chip,
						buf, ops, mtd->oobsize);
			}

			if (!(chip->options & NAND_NO_READRDY)) {
				/* Apply delay or wait for ready/busy pin. Do
				 * this before the AUTOINCR check, so no
				 * problems arise if a chip which does auto
				 * increment is marked as NOAUTOINCR by the
				 * board driver.
				 */
				if (!chip->dev_ready)
					udelay(chip->chip_delay);
				else
					nand_wait_ready(mtd);
			}
#ifdef CONFIG_MTD_NAND_LF1000_PROF
			nand_stats_accum (NS_READ, 0);
#endif
		} else {
			memcpy(buf, chip->buffers->databuf + col, bytes);
			buf += bytes;
		}
		readlen -= bytes;
		if (!readlen)
			break;

		/* For subsequent reads align to page boundary. */
		col = 0;
		realpage++;

		page = realpage & chip->pagemask;
		/* Check if we cross a chip boundary */
		if (!page) {
			chipnr++;
			chip->select_chip(mtd, -1);
			chip->select_chip(mtd, chipnr);
		}

		/* Check if the chip supports auto page increment
		 * or if we have hit a block boundary.
		 */
		if (!NAND_CANAUTOINCR(chip) || !(page & blkcheck))
			sndcmd = 1;
	}

	ops->retlen = ops->len - (size_t) readlen;
	if (oob)
		ops->oobretlen = ops->ooblen - oobreadlen;

	if (ret)
		return ret;

	if (mtd->ecc_stats.failed - stats.failed) {
        	dev_info(&nand.pdev->dev, "nand_do_read_ops: ECC error: "
                         "from %08x %08x; realpage %x; page %x; readlen %d\n",	
                	(uint32_t)(from >> 32), (uint32_t)from, 
                	realpage, page, ops->len);
		return -EBADMSG;
    	}
#ifndef CONFIG_MTD_NAND_LF1000_MLC_SCRUB_THRESHOLD
	return  mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN : 0;
#else
            /* if this device is an MLC NAND, check for at least
             * CONFIG_MTD_NAND_LF1000_MLC_SCRUB_THRESHOLD bit-flip errors
             * on the current page; if found, return -EUCLEAN.
             * (When UBI sees that value returned, it schedules the page's
             * erase block for scrubbing (copying to another block and then
             * erasing)
             */
	if (chip->cellinfo & NAND_CI_CELLTYPE_MSK) {
        	return  (mtd->ecc_stats.corrected - stats.corrected
                                >= CONFIG_MTD_NAND_LF1000_MLC_SCRUB_THRESHOLD)
                	? -EUCLEAN
                	: 0;
    	}
	else {
        	return  mtd->ecc_stats.corrected - stats.corrected ? -EUCLEAN 
								   : 0;
    	}
#endif
}

/**
 * lf1000_nand_read - Leapfrog's replacement for nand_read()
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @len:	number of bytes to read
 * @retlen:	pointer to variable to store the number of read bytes
 * @buf:	the databuffer to put data
 *
 * Get hold of the chip and call nand_do_read
 *
 * Leapfrog has a special version of this function in order to call
 * lf1000_nand_do_read_ops() instead of the standard nand_do_read_ops().
 * Except for that call, this function is identical to nand_do_read_ops().
 */
static int lf1000_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
		                    size_t *retlen, uint8_t *buf)
{
	struct nand_chip *chip = mtd->priv;
	int ret;

	/* Do not allow reads past end of device */
	if ((from + len) > mtd->size)
		return -EINVAL;
	if (!len)
		return 0;

	nand_get_device(chip, mtd, FL_READING);

	chip->ops.len = len;
	chip->ops.datbuf = buf;
	chip->ops.oobbuf = NULL;

	ret = lf1000_nand_do_read_ops(mtd, from, &chip->ops);

	*retlen = chip->ops.retlen;

	nand_release_device(mtd);

	return ret;
}

/**
 * lf1000_nand_read_oob - Leapfrog replacement for nand_read_oob().
 * @mtd:	MTD device structure
 * @from:	offset to read from
 * @ops:	oob operation description structure
 *
 * NAND read data and/or out-of-band data
 *
 * This function differs from nand_read_oob() only by calling 
 * lf1000_nand_do_read_ops() instead of nand_do_read_ops().
 */
static int lf1000_nand_read_oob(struct mtd_info *mtd, loff_t from,
		            	 struct mtd_oob_ops *ops)
{
	struct nand_chip *chip = mtd->priv;
	int ret = -ENOTSUPP;

	ops->retlen = 0;

	/* Do not allow reads past end of device */
	if (ops->datbuf && (from + ops->len) > mtd->size) {
		DEBUG(MTD_DEBUG_LEVEL0, "nand_read_oob: "
		      "Attempt read beyond end of device\n");
		return -EINVAL;
	}
	nand_get_device(chip, mtd, FL_READING);

	switch(ops->mode) {
	case MTD_OOB_PLACE:
	case MTD_OOB_AUTO:
	case MTD_OOB_RAW:
		break;
	default:
		goto out;
	}
	if (!ops->datbuf)
		ret = nand_do_read_oob(mtd, from, ops);
	else
		ret = lf1000_nand_do_read_ops(mtd, from, ops);
out:
	nand_release_device(mtd);
	return ret;
}


/**
 * lf1000_nand_write_page - write one page; replaces nand_write_page()
 * @mtd:	MTD device structure
 * @chip:	NAND chip descriptor
 * @buf:	the data to write
 * @page:	page number to write
 * @cached:	cached programming
 * @raw:	use _raw version of write_page
 *
 * We use a special version that checks for all FF before trying to write
 * to the NAND.  This is necessary when using UBIFS on MLC NAND.
 */
static int lf1000_nand_write_page(struct mtd_info  *mtd, 
                                  struct nand_chip *chip,
	                          const uint8_t    *buf, 
                                  int page, 
                                  int cached, 
                                  int raw)
{
	u32 eb_index;
	int status;
#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
	int power = 0;
#endif

	/* First check if the page is all FF; if it is, don't bother writing
	 * to the NAND.  Just verify that the nand page is all FF
	 */
	if ( !all_bytes_ff( buf, mtd->writesize) ) {
#ifdef CONFIG_MTD_NAND_LF1000_PROF
	    	nand_stats_accum (NS_WRITE, 1);
#endif
#ifdef TIME_NAND_WRITE_ENTIRE
		++nand_num_writes;
		timer_start();
#endif
		eb_index = (page & chip->pagemask) / 
			                    (1 << (chip->phys_erase_shift 
						   - chip->page_shift));
		if (eb_index < MAX_NUM_ERASE_BLOCKS) {
			block_write_counts[ eb_index ] += 1;
		        total_writes++;
		}
	    	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0x00, page);

    		if (unlikely(raw))
    			chip->ecc.write_page_raw(mtd, chip, buf);
    		else
    			chip->ecc.write_page(mtd, chip, buf);

		/*
		 * Cached progamming disabled for now, Not sure if its worth the
		 * trouble. The speed gain is not very impressive. (2.3->2.6Mib/s)
		 */
	    	cached = 0;

	    	if (!cached || !(chip->options & NAND_CACHEPRG)) {

#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
	    		power = IS_SET(nand_stress, POWER_WRITE);
	    		if(power) {
	    			int pages_per_block = 
						1 << (chip->phys_erase_shift - 
							chip->page_shift);
	    			dev_emerg(&nand.pdev->dev, 
					 "nand_write: cutting power while"
	    			         " writing page 0x%08x (block %d)\n",
	    				  page & chip->pagemask,
	    				 (page & chip->pagemask) 
						/ pages_per_block );
	    			stress_config_power();
	    		}
#endif
	    		chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
	    		if(power) 
	    			stress_cut_power();
#endif
			status = chip->waitfunc(mtd, chip);
#ifdef TIME_NAND_WRITE_ENTIRE
			nand_write_time += timer_stop();
#endif
#ifdef CONFIG_MTD_NAND_LF1000_PROF
    			nand_stats_accum (NS_WRITE, 0);
#endif
			/* See if operation failed and additional status checks 
			 * are available
			 */
			if ((status & NAND_STATUS_FAIL) && (chip->errstat))
				status = chip->errstat(mtd, chip, FL_WRITING,
							status, page);
			if (status & NAND_STATUS_FAIL)
				return -EIO;
    		} 
        	else { /* cached */
			chip->cmdfunc(mtd, NAND_CMD_CACHEDPROG, -1, -1);
    			status = chip->waitfunc(mtd, chip);
#ifdef TIME_NAND_WRITE_ENTIRE
            		nand_write_time += timer_stop();

#endif
#ifdef CONFIG_MTD_NAND_LF1000_PROF
    			nand_stats_accum (NS_WRITE, 0);
#endif
	    	}
	}   /* !all_bytes_ff */
#ifdef CONFIG_MTD_NAND_VERIFY_WRITE
	/* Send command to read back the data */
	chip->cmdfunc(mtd, NAND_CMD_READ0, 0, page);
        eb_index = (page & chip->pagemask) / 
                        (1 << (chip->phys_erase_shift - chip->page_shift));
        if (eb_index < MAX_NUM_ERASE_BLOCKS) {
            block_read_counts[ eb_index ] += 1;
            total_reads++;
        }
	if (chip->verify_buf(mtd, buf, mtd->writesize))
		return -EIO;
#endif
	return 0;
}



/**
 * lf1000_nand_write_page_swecc - [REPLACABLE] software ecc based page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 *
 * NOTE: This routine is identical to nand_write_page_swecc() EXCEPT for the
 *       code that's been added to count and time write operations.
 */
static void lf1000_nand_write_page_swecc(struct mtd_info  *mtd, 
                                         struct nand_chip *chip,
				         const uint8_t    *buf)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	const uint8_t *p = buf;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

#ifdef TIME_NAND_WRITE_PARTS
	++nand_num_writes;
	timer_start();
#endif
	/* Software ecc calculation */
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
#ifdef TIME_NAND_WRITE_PARTS
	nand_write_calc_time += timer_stop();
#endif

	for (i = 0; i < chip->ecc.total; i++)
		chip->oob_poi[eccpos[i]] = ecc_calc[i];

#ifdef TIME_NAND_WRITE_PARTS
	timer_start();
#endif

	chip->ecc.write_page_raw(mtd, chip, buf);
#ifdef TIME_NAND_WRITE_PARTS
	nand_write_time += timer_stop();

	/* enable this 'if' to reduce the amount of output
	 * if (0 == (nand_num_writes % 100))
	 */
	dev_info(&nand.pdev->dev, "NandWrite: %d, %d, %d\n", 
	         nand_num_writes, nand_write_time, nand_write_calc_time);
#endif
}

/**
 * lf1000_nand_read_page_swecc - replaces nand_read_page_swecc() when
 *                               timing of nand reads is enabled.
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 *
 * NOTE: This routine is identical to nand_read_page_swecc() EXCEPT for the
 *       code that's been added to count and time read operations.
 */
static int lf1000_nand_read_page_swecc( struct mtd_info *mtd, 
                                        struct nand_chip *chip,
                        				uint8_t *buf, int page)
{
	int i, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	uint8_t *ecc_code = chip->buffers->ecccode;
	uint32_t *eccpos = chip->ecc.layout->eccpos;

#ifdef TIME_NAND_READ
	++nand_num_reads;
	timer_start();
#endif
	chip->ecc.read_page_raw(mtd, chip, buf, 0);
#ifdef TIME_NAND_READ
	nand_read_time += timer_stop();
	timer_start();
#endif
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		chip->ecc.calculate(mtd, p, &ecc_calc[i]);
#ifdef TIME_NAND_READ
	nand_read_calc_time += timer_stop();
	timer_start();
#endif
	for (i = 0; i < chip->ecc.total; i++)
		ecc_code[i] = chip->oob_poi[eccpos[i]];

	eccsteps = chip->ecc.steps;
	p	 = buf;

	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		stat = chip->ecc.correct(mtd, p, &ecc_code[i], &ecc_calc[i]);
		if (stat < 0)
			mtd->ecc_stats.failed++;
		else
			mtd->ecc_stats.corrected += stat;
	}
#ifdef TIME_NAND_READ
	nand_read_check_time += timer_stop();

	/* enable this 'if' to reduce the amount of output
	 * if (0 == (nand_num_reads % 100))
	 */
	dev_info(&nand.pdev->dev, "NandRead: %d, %d, %d, %d\n", 
	          nand_num_reads, nand_read_time, 
        	  nand_read_calc_time, nand_read_check_time);
#endif
	return 0;
}


/**
 * lf1000_nand_read_buf - read chip data into buffer
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 * Default read function for 8bit buswith
 * Differs from nand_read_buf() by using 32-bit access to the LF1000's
 * NAND Flash Data register when possible.
 */
#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
extern void lf1000_select_cart(struct mtd_info *mtd, int chipnr);
#endif

static void lf1000_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	uint8_t *b   = buf;
	int      rem = (3 & (unsigned int)b);
	struct nand_chip    *chip = mtd->priv;
	tpIO A = chip->IO_ADDR_R;

#ifdef CONFIG_MTD_NAND_LF1000_STRESS_TEST
	if( IS_SET(nand_stress, CART_READ) &&
	    (chip->select_chip == lf1000_select_cart) )	{
		stress_cut_cart(1);
	}
#endif
	if (rem) {
	        while ( (len > 0) && (rem < 4)) {
			*b++ = readb(A);
            		++rem;
            		--len;
        	}
	}
	if (0 == (3 & (unsigned int)b)) {
	        u32 * p = (u32 *)b;
		for (i = 0; i < len - 3; i += 4) {
			*p++ = readl(A);
        	}
        	b = (uint8_t *)p;
	    	while (i++ < len) {
			*b++ = readb(A);
        	}
	}
	else {
	        dev_info(&nand.pdev->dev, 
			 "!@#$ Unexpected path through nand_read_buf()\n");
		for (i = 0; i < len; i++)
			*b++ = readb(A);
	}
}



/**
 * lf1000_nand_write_buf - write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 * Default write function for 8bit buswith
 * Differs from nand_write_buf() by using 32-bit access to the LF1000's
 * NAND Flash Data register when possible.
 */
static void lf1000_nand_write_buf(struct mtd_info * mtd, 
                                  const uint8_t   * buf, 
                                  int               len)
{
	int i;
	struct nand_chip *chip = mtd->priv;
	int rem = (3 & (unsigned int)buf);

	/* NOTE:   This function is called many times  
	 *          (Currently 8 times per page: 4 for 512-byte subpages and
	 *           4 for 7-byte ECC codes.
	 */
	if ((rem == 0) && (0 == (3 & len))) {
		/* we'll use 32-bit access if buffer is on a 4-byte boundary
		 * and len is a multiple of 4
		 */
		u32 * p = (u32 *)buf;
		for (i = 0; i < len; i += 4) {
			writel(*p++, chip->IO_ADDR_W);
        	}
	}
	else {
	        for (i = 0; i < len; i++)
		        writeb(buf[i], chip->IO_ADDR_W);
	}
}


/**
 * lf1000_nand_wait - [DEFAULT]  wait until the command is done
 * @mtd:	MTD device structure
 * @chip:	NAND chip structure
 *
 * Wait for command done. This applies to erase and program only
 * Erase can take up to 400ms and program up to 20ms according to
 * general NAND and SmartMedia specs.
 *
 * This function differs from nand_wait() by calling ndelay(2000) instead of
 * ndelay(100).  The additional delay seemed to make 32-bit writes to the
 * LF1000's NAND Flash Data register work ok.
 */
static int lf1000_nand_wait(struct mtd_info *mtd, struct nand_chip *chip)
{

	unsigned long timeo = jiffies;
	int status, state = chip->state;

	if (state == FL_ERASING)
		timeo += (HZ * 400) / 1000;
	else
		timeo += (HZ * 20) / 1000;

	led_trigger_event(nand_led_trigger, LED_FULL);

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(2000);

	if ((state == FL_ERASING) && (chip->options & NAND_IS_AND))
		chip->cmdfunc(mtd, NAND_CMD_STATUS_MULTI, -1, -1);
	else
		chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);

	while (time_before(jiffies, timeo)) {
		if (chip->dev_ready) {
			if (chip->dev_ready(mtd))
				break;
		} else {
			if (chip->read_byte(mtd) & NAND_STATUS_READY)
				break;
		}
		cond_resched();
	}
	led_trigger_event(nand_led_trigger, LED_OFF);

	status = (int)chip->read_byte(mtd);
	return status;
}



/* If this routine fails to allocate mtd_info and nand_chip structures, 
 * on return from the routine *ppmtd_info will be NULL.
 */
static void lf1000_init_mtd_info(struct mtd_info **ppmtd_info, u32 nand_base)
{
	struct nand_chip *pchip;

	*ppmtd_info = kmalloc(sizeof(struct mtd_info) +
			                sizeof(struct nand_chip), GFP_KERNEL);
	if(!*ppmtd_info)
		return;

	memset(*ppmtd_info, 0, sizeof(struct mtd_info)+ sizeof(struct nand_chip));

	(*ppmtd_info)->owner = THIS_MODULE;

	/* Get pointer to private data */
	pchip = (struct nand_chip *) (&((*ppmtd_info)[1]));

	/* Link the private data with the MTD structure */
	(*ppmtd_info)->priv = pchip;

	pchip->controller = &nand.controller;

	/* Set address of NAND IO lines */
	pchip->IO_ADDR_R   = (void __iomem *)(IO_ADDRESS(nand_base+NFDATA));
	pchip->IO_ADDR_W   = (void __iomem *)(IO_ADDRESS(nand_base+NFDATA));

	pchip->dev_ready   = lf1000_nand_ready;
	pchip->options     = 0; /* 8 bit bus width */
	pchip->cmd_ctrl    = lf1000_hwcontrol; /* hardware access for cmd, addr */
	pchip->write_page  = lf1000_nand_write_page;
	pchip->read_buf    = lf1000_nand_read_buf;
	pchip->write_buf   = lf1000_nand_write_buf;
	pchip->waitfunc    = lf1000_nand_wait;

	/* 25 us command delay time */
	pchip->chip_delay  = 25;
}



static int lf1000_verify_SLC_page(struct mtd_info *mtd, 
                                  const uint8_t *buf, 
                                  int len)
{
	/* When using MLC, we must verify writes.  
	 * When using SLC, we don't need to, but the kernel configuration 
	 * parameter causes us to do so.  
	 * Because SLC allows subpage writes, and because the default
	 * nand_verify_buf() doesn't work correctly in that case, we
	 * just skip the verification and say it was ok.
	 */
	return 0;
}


static void lf1000_set_soft_ecc(struct nand_chip *chip)
{
	chip->ecc.calculate      = nand_calculate_ecc;
	chip->ecc.correct        = nand_correct_data;
	chip->ecc.read_page      = nand_read_page_swecc;
	chip->ecc.read_subpage   = nand_read_subpage;
	chip->ecc.write_page     = lf1000_nand_write_page_swecc;
	chip->ecc.read_page_raw  = nand_read_page_raw;
	chip->ecc.write_page_raw = nand_write_page_raw;
	chip->ecc.read_oob       = nand_read_oob_std;
	chip->ecc.write_oob      = nand_write_oob_std;
	chip->ecc.size           = 256;
	chip->ecc.bytes          = 3;
}

static void lf1000_set_no_ecc(struct mtd_info *mtd, struct nand_chip *chip)
{
	dev_warn(&nand.pdev->dev, "NAND_ECC_NONE selected by board driver. "
	        	        "This is not recommended !!\n");
	chip->ecc.read_page      = nand_read_page_raw;
	chip->ecc.read_page_raw  = nand_read_page_raw;
	chip->ecc.write_page     = nand_write_page_raw;
	chip->ecc.write_page_raw = nand_write_page_raw;
	chip->ecc.read_oob       = nand_read_oob_std;
	chip->ecc.write_oob      = nand_write_oob_std;
	chip->ecc.size           = mtd->writesize;
	chip->ecc.bytes          = 0;
}

/**
 * lf1000_nand_switch_ecc_mode - [NAND Interface] Scan for the NAND device
 * @mtd:	    MTD device structure
 * @maxchips:	    Number of chips to scan for
 *
 * Change ECC mode on master MTD device
 */
static int lf1000_nand_switch_ecc_mode(struct mtd_info *mtd, int ecc_mode)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	chip->ecc.mode = ecc_mode;
	switch (chip->ecc.mode) {

	case NAND_ECC_HW_SYNDROME:
        	lf1000_set_hw_syndrome_ecc(chip);
		if (mtd->writesize >= chip->ecc.size)
			break;
		dev_warn(&nand.pdev->dev, "%d byte HW ECC not possible on "
		       "%d byte page size, fallback to SW ECC\n",
		       chip->ecc.size, mtd->writesize);
		chip->ecc.mode = NAND_ECC_SOFT;
		/* drop through here */
	case NAND_ECC_SOFT:
        	lf1000_set_soft_ecc(chip);
		break;

	case NAND_ECC_NONE:
        	lf1000_set_no_ecc(mtd, chip);
		break;

	case NAND_ECC_INTERNAL:
	        lf1000_prepare_for_InternalEcc(chip);
	        break;

	default:
		dev_warn(&nand.pdev->dev, "Invalid NAND_ECC_MODE %d\n",
		       chip->ecc.mode);
		BUG();
	}
	/* The number of bytes available for a client to place data into
	 * the out of band area
	 */
	chip->ecc.layout->oobavail = 0;
	for (i = 0; chip->ecc.layout->oobfree[i].length; i++)
		chip->ecc.layout->oobavail += 
					chip->ecc.layout->oobfree[i].length;
	mtd->oobavail = chip->ecc.layout->oobavail;

	/* Set the number of read / write steps for one page depending on ECC
	 * mode
	 */
	chip->ecc.steps = mtd->writesize / chip->ecc.size;
	if(chip->ecc.steps * chip->ecc.size != mtd->writesize) {
		dev_warn(&nand.pdev->dev, "Invalid ecc parameters\n");
		BUG();
	}
	chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;

	/* Allow subpage writes up to ecc.steps. Not possible for MLC
	 * FLASH.
	 */
	if (   !(chip->options & NAND_NO_SUBPAGE_WRITE) 
	    && !(chip->cellinfo & NAND_CI_CELLTYPE_MSK)) {
		switch(chip->ecc.steps) {
		case 1: mtd->subpage_sft = 0;
			break;
		case 2: mtd->subpage_sft = 1;
			break;
		case 4:
		case 8: mtd->subpage_sft = 2;
			break;
		}
	}
	chip->subpagesize = mtd->writesize >> mtd->subpage_sft;

	/* Invalidate the pagebuffer reference */
	chip->pagebuf = -1;

	/* propagate ecc.layout to mtd_info */
	mtd->ecclayout = chip->ecc.layout;

	return 0;
}


/**
 * lf1000_nand_scan_tail - [NAND Interface] Scan for the NAND device
 * @mtd:	    MTD device structure
 *
 * Leapfrog's version of nand_scan_tail().
 * We have our own version in order to use code in common with 
 * lf1000_nand_switch_ecc_mode().
 */
static int lf1000_nand_scan_tail(struct mtd_info *mtd)
{
	int i;
	struct nand_chip *chip = mtd->priv;

	if (!(chip->options & NAND_OWN_BUFFERS))
		chip->buffers = kmalloc(sizeof(*chip->buffers), GFP_KERNEL);
	if (!chip->buffers)
		return -ENOMEM;

	/* Set the internal oob buffer location, just after the page data */
	chip->oob_poi = chip->buffers->databuf + mtd->writesize;

	/*
	 * If no default placement scheme is given, select an appropriate one
	 */
	if (!chip->ecc.layout) {
		switch (mtd->oobsize) {
		case 8: chip->ecc.layout = &nand_oob_8;
			break;
		case 16: chip->ecc.layout = &nand_oob_16;
			break;
		case 64: chip->ecc.layout = &nand_oob_64;
			break;
		case 128: chip->ecc.layout = &nand_oob_128;
			break;
		default: dev_warn(&nand.pdev->dev, "No oob scheme defined for "
			       "oobsize %d\n", mtd->oobsize);
			BUG();
		}
	}

	if (!chip->write_page)
		chip->write_page = lf1000_nand_write_page;

	/* check ECC mode, default to software if 3byte/512byte hardware ECC is
	 * selected and we have 256 byte pagesize fallback to software ECC
	 */
	switch (chip->ecc.mode) {

	case NAND_ECC_HW_SYNDROME:
	        lf1000_set_hw_syndrome_ecc(chip);
		if (mtd->writesize >= chip->ecc.size)
			break;
		dev_warn(&nand.pdev->dev, "%d byte HW ECC not possible on "
		       "%d byte page size, fallback to SW ECC\n",
		       chip->ecc.size, mtd->writesize);
		chip->ecc.mode = NAND_ECC_SOFT;
		/* drop through here */
	case NAND_ECC_SOFT:
	        lf1000_set_soft_ecc(chip);
		break;

	case NAND_ECC_NONE:
	        lf1000_set_no_ecc(mtd, chip);
		break;

	case NAND_ECC_INTERNAL:
	        lf1000_prepare_for_InternalEcc(chip);
	        break;

	default:
		dev_warn(&nand.pdev->dev, "Invalid NAND_ECC_MODE %d\n",
		       chip->ecc.mode);
		BUG();
	}

	/* The number of bytes available for a client to place data into
	 * the out of band area
	 */
	chip->ecc.layout->oobavail = 0;
	for (i = 0; chip->ecc.layout->oobfree[i].length
		    && i < ARRAY_SIZE(chip->ecc.layout->oobfree); i++)
		chip->ecc.layout->oobavail +=
					chip->ecc.layout->oobfree[i].length;
	mtd->oobavail = chip->ecc.layout->oobavail;

	/* Set the number of read / write steps for one page depending on ECC
	 * mode
	 */
	chip->ecc.steps = mtd->writesize / chip->ecc.size;
	if(chip->ecc.steps * chip->ecc.size != mtd->writesize) {
		dev_warn(&nand.pdev->dev, "Invalid ecc parameters\n");
		BUG();
	}
	chip->ecc.total = chip->ecc.steps * chip->ecc.bytes;

	/* Allow subpage writes up to ecc.steps. Not possible for MLC
	 * FLASH.
	 */
	if (   !(chip->options & NAND_NO_SUBPAGE_WRITE) 
	    && !(chip->cellinfo & NAND_CI_CELLTYPE_MSK)) {
		switch(chip->ecc.steps) {
		case 1: mtd->subpage_sft = 0;
			break;
		case 2: mtd->subpage_sft = 1;
			break;
		case 4:
		case 8:
		case 16: mtd->subpage_sft = 2;
			break;
		}
	}
	dev_err(&nand.pdev->dev, "I: chip->options & NAND_NO_SUBPAGE_WRITE = %d\n", 
            chip->options & NAND_NO_SUBPAGE_WRITE);
	dev_err(&nand.pdev->dev, "I: chip->options & NAND_CI_CELLTYPE_MSK = %d\n", 
            chip->options & NAND_CI_CELLTYPE_MSK);
	dev_err(&nand.pdev->dev, "I: ecc.steps = %d\n", chip->ecc.steps);
	dev_err(&nand.pdev->dev, "I: mtd=%p mtd->subpage_sft = %x\n", 
            mtd, mtd->subpage_sft);
	chip->subpagesize = mtd->writesize >> mtd->subpage_sft;

	chip->state = FL_READY;

	chip->select_chip(mtd, -1); /* De-select the device */
	chip->pagebuf = -1;	    /* Invalidate the pagebuffer reference */

	/* Fill in remaining MTD driver data */
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	mtd->erase = lf1000_nand_erase;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = lf1000_nand_read;
	mtd->write = nand_write;
	mtd->read_oob = lf1000_nand_read_oob;
	mtd->write_oob = nand_write_oob;
	mtd->sync = nand_sync;
	mtd->lock = NULL;
	mtd->unlock = NULL;
	mtd->suspend = nand_suspend;
	mtd->resume = nand_resume;
	mtd->block_isbad = nand_block_isbad;
	mtd->block_markbad = nand_block_markbad;

	/* propagate ecc.layout to mtd_info */
	mtd->ecclayout = chip->ecc.layout;

	/* Check, if we should skip the bad block table scan */
	if (chip->options & NAND_SKIP_BBTSCAN)
		return 0;

	/* Build bad block table */
	return chip->scan_bbt(mtd);
}




static void lf1000_init_for_SLC_nand(struct mtd_info *mtd, 
                                     struct nand_chip *chip)
{
    chip->ecc.mode    = NAND_ECC_SOFT;
    chip->verify_buf  = lf1000_verify_SLC_page;
    dev_info(&nand.pdev->dev, "Using 1-bit ECC\n");
}



/* Leapfrog-specific version of standard nand_scan() function.
 * This function calls the standard nand_scan_ident() and nand_scan_tail(),
 * but between those calls it performs initialization that's appropriate for
 * the types of NAND flash parts that are found in Leapfrog products that use
 * this software.
 *
 * cart_nand: 0 == base nand, nonzero == cart nand
 */
static int lf1000_nand_scan(struct mtd_info * mtd, 
			    int		      maxchips, 
			    u32		    * p_nand_props, 
			    int		      cart_nand)
{
	int ret;

	ret = nand_scan_ident(mtd, maxchips, NULL);

	if (!ret) {
		struct nand_chip *chip = mtd->priv;

			/* if cartridge has OTP, use standard code */
		if (cart_nand && (chip->ecc.mode == NAND_ECC_NONE)) {
			ret = nand_scan_tail(mtd);
		}
		else {
			if (chip_is_MT29F4G08ABADA(mtd, chip, p_nand_props, 
							cart_nand))
			{
				lf1000_init_for_MT29F4G08ABADA( mtd, chip, 
								p_nand_props);
			} else if (chip->cellinfo & NAND_CI_CELLTYPE_MSK) {
				lf1000_init_for_MLC_nand( mtd, chip );
			} else {
				lf1000_init_for_SLC_nand( mtd, chip );
			}
			dev_info(&nand.pdev->dev, " subpage_sft %d, "
						  "subpagesize %d\n",
			       mtd->subpage_sft, chip->subpagesize);

			dev_info(&nand.pdev->dev, 
				 "  IDB3 0x%02x, page_shift %d, pagemask x%x\n"
				 "  bbt_erase_shift %d, chip_shift %d,"
				 " badblkpos %d\n",
				 chip->cellinfo, chip->page_shift, chip->pagemask, 
				 chip->bbt_erase_shift, chip->chip_shift, 
				 chip->badblockpos);
			ret = lf1000_nand_scan_tail(mtd);
		}
	}
	return ret;
}


static int lf1000_nand_probe(struct platform_device *pdev)
{
	int                   ret;
	struct resource      *res;
	struct nand_chip     *this;
	const char           *part_type     = NULL;
	int                   base_parts_nb = 0;
	int                   cart_parts_nb = 0;
	struct mtd_partition *base_parts    = NULL;
	struct mtd_partition *cart_parts    = NULL;

	nand.pdev = pdev;

	/* check if a cartridge is inserted */
	gpio_configure_pin(NAND_CART_DETECT_PORT, NAND_CART_DETECT_PIN,
			   GPIO_GPIOFN, 0, 1, 0);
        
	/* initialization of a cartridge nand is initiated by
         * set_cart_hotswap_state().
         */
	mutex_init(&nand.sem_hotswap);
	
	/* Reserve resources for driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res) {
		dev_err(&pdev->dev, "failed to get resource!\n");
		return -ENXIO;
	}
	if(!request_mem_region(res->start, (res->end - res->start)+1,
				"lf1000-nand")) {
		dev_err(&pdev->dev, "failed to map memory region.");
		return -EBUSY;
	}
	nand.mem = ioremap(res->start, (res->end - res->start)+1);
	if(nand.mem == NULL) {
		dev_err(&pdev->dev, "failed to ioremap\n");
		ret = -ENOMEM;
		goto fail_remap;
	}
	
	/* initialize the controller */
	spin_lock_init(&nand.controller.lock);
	init_waitqueue_head(&nand.controller.wq);
	
	printk(KERN_INFO "MTD: Controller Initialized\n");

	/* Allocate memory for onboard NAND MTD device structure and private data */
	lf1000_init_mtd_info(&nand.mtd_onboard, res->start);
	if(!nand.mtd_onboard) {
		dev_err(&pdev->dev, "Unable to allocate MTD device\n");
		ret = -ENOMEM;
		goto fail_mem_onboard;
	}
	this              = (struct nand_chip *)nand.mtd_onboard->priv;
	this->select_chip = lf1000_select_chip;
	
	/* Use our version of nand_scan(), with special processing for SLC/MLC*/
	/* find the onboard NAND Flash*/
	if (lf1000_nand_scan(nand.mtd_onboard, 1, &nand.base_nand_props, 0)) {
		dev_err(&pdev->dev, "Failed to find on board NAND\n");
		kfree(nand.mtd_onboard);
		nand.mtd_onboard = NULL;
	}
	
	/* Add the base partitions */
#ifdef CONFIG_MTD_PARTITIONS
	if(nand.mtd_onboard != NULL)
	{
		nand.mtd_onboard->name = "lf1000-base";
		base_parts_nb = parse_mtd_partitions(nand.mtd_onboard,
						     part_probes, &base_parts, 0);
		if (base_parts_nb > 0)
			part_type = "command line";
		else
			base_parts_nb = 0;
	}

	if(nand.mtd_cart != NULL) {
		nand.mtd_cart->name = "lf1000-cart";
		cart_parts_nb = parse_mtd_partitions(nand.mtd_cart,
						     part_probes,
						     &cart_parts, 0);
		if (cart_parts_nb > 0)
			part_type = "command line";
		else
			cart_parts_nb = 0;
	}
#endif
	if (base_parts_nb == 0) {
		if (gpio_get_boot_source_config() == SCRATCH_BOOT_SOURCE_USB) {
			base_parts = partition_info_recovery;
			base_parts_nb = ARRAY_SIZE(partition_info_recovery);
		} else {
			base_parts = partition_info;
			base_parts_nb = ARRAY_SIZE(partition_info);
		}
		part_type = "static";
	}

	if (cart_parts_nb == 0) {
		cart_parts = partition_info_cart;
		cart_parts_nb = ARRAY_SIZE(partition_info_cart);
		part_type = "static";
	}

	/* if madrid (LeapPad), defer to CartManager, otherwise register partitions */
	
	/* Register the onboard partitions */
	if ( nand.mtd_onboard != NULL)
		add_mtd_partitions(nand.mtd_onboard, base_parts, base_parts_nb);

	/* if madrid (LeapPad), let CartManager request partitions */
	/* Register the cartridge partitions, if it exists */
	if ( !gpio_have_gpio_madrid() && nand.mtd_cart != NULL) {
		nand.cart_ready = 1;		
		add_mtd_partitions(nand.mtd_cart, cart_parts, cart_parts_nb);
	}

	/* enable NAND_WP pin as an output, enable write & erase */
	gpio_configure_pin(NAND_WP_PORT, NAND_WP_PIN, GPIO_GPIOFN, 1, 0, 1);

	sysfs_create_group(&pdev->dev.kobj, &nand_attr_group);
	return 0;

fail_mem_onboard:
	iounmap(nand.mem);
fail_remap:
	release_mem_region(res->start, (res->end - res->start) + 1);
	return ret;
}

static int lf1000_nand_remove(struct platform_device *pdev)
{
	struct resource *res  = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	sysfs_remove_group(&pdev->dev.kobj, &nand_attr_group);

	/* Release resources, unregister device */
	nand_release(nand.mtd_onboard);
	if(nand.mtd_cart)
		nand_release(nand.mtd_cart);

	/* Free the MTD device structure */
	kfree(nand.mtd_onboard);
	if(nand.mtd_cart)
		kfree(nand.mtd_cart);

	if(nand.mem != NULL)
		iounmap(nand.mem);

	release_mem_region(res->start, (res->end - res->start) + 1);
	return 0;
}

static struct platform_driver lf1000_nand_driver = {
	.probe		= lf1000_nand_probe,
	.remove		= lf1000_nand_remove,
	.driver		= {
		.name	= "lf1000-nand",
		.owner	= THIS_MODULE,
	},
};

static int __init lf1000_init(void)
{
	return platform_driver_register(&lf1000_nand_driver);
}
module_init(lf1000_init);

static void __exit lf1000_cleanup(void)
{
	platform_driver_unregister(&lf1000_nand_driver);
}
module_exit(lf1000_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leapfrog.com");
MODULE_DESCRIPTION("NAND Flash Controller for the LF1000");

#endif  /* CPU_LF1000 */

