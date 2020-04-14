/*
 *  drivers/mtd/nand/lf1000_internal_ECC.c
 *
 * Copyright 2007-2010 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <ayurovsky@leapfrog.com>
 * Scott Esters <sesters@leapfrog.com>
 * Robert T. Dowling <rdowling@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This file contains routines and data for dealing with NAND flash devices
 * that internally calculate, store, check, and correct ECC.  The Micron
 * MT29F4G08ABADA is an example.
 */


static void lf1000_prepare_for_InternalEcc( struct nand_chip * pchip);


static struct nand_ecclayout micron_nand_oob_internal_ecc = {
	.eccbytes = 32,
	.eccpos = {0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
		   0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
		   0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
           	   0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f },
	.oobfree = {
		{.offset = 0x02,
		 .length = 4},
		{.offset = 0x12,
		 .length = 4},
		{.offset = 0x22,
		 .length = 4},
		{.offset = 0x32,
		 .length = 4}}
};

#define MICRON_SET_FEATURES_CMD     0xEF
#define MICRON_GET_FEATURES_CMD     0xEE
#define MICRON_ARRAY_OPN_MODE_ADDR  0x90
#define ARRAY_OPN_MODE_ENABLE_ECC   0x08
#define ARRAY_OPN_MODE_DISABLE_ECC  0x00

/* Enable or Disable Micron Nand's internal ecc mode
 * the 'mode' argument indicates enable or disable:
 *    ARRAY_OPN_MODE_ENABLE_ECC 
 *    ARRAY_OPN_MODE_DISABLE_ECC 
 *
 * Select the nand device
 * send the Set Features command for the Array Operations
 * send the mode byte and 3 zeroes
 * wait for the device to be ready
 * send the Get Features command for the Array Operations
 * read and return a byte, which indicates whether or not internal ecc
 *	is enabled.
 */
static unsigned char SetInternalEccMode( struct mtd_info *mtd, 
		 	      		 struct nand_chip *chip,
					 unsigned char mode )
{
	unsigned char bvalue;

	/* Select the device */
	chip->select_chip(mtd, 0);

	// enable internal ecc
	chip->cmdfunc(mtd, MICRON_SET_FEATURES_CMD, 
		      MICRON_ARRAY_OPN_MODE_ADDR, -1);
	/* delay briefly */
	ndelay(100);

	writeb( mode, chip->IO_ADDR_W);
	writeb(0, chip->IO_ADDR_W);
	writeb(0, chip->IO_ADDR_W);
	writeb(0, chip->IO_ADDR_W);

	nand_wait_ready(mtd);

	/* now read back the array operations mode 
	 * values and check for internal ecc enabled
	 */
	chip->cmdfunc(mtd, MICRON_GET_FEATURES_CMD, 
		      MICRON_ARRAY_OPN_MODE_ADDR, -1);
	/* delay briefly */
	ndelay(100);

	bvalue = readb(chip->IO_ADDR_R);
	return bvalue;
}

static int DisableInternalECC(struct mtd_info *mtd, 
		 	      struct nand_chip *chip,
			      u32 * p_nand_props,
			      int cart_nand) 
{
	unsigned char bvalue;

	bvalue = SetInternalEccMode(mtd, chip, ARRAY_OPN_MODE_DISABLE_ECC);
	printk(KERN_CRIT "Trying to disable cart intECC; read back %d\n",
		bvalue);
	if (bvalue == ARRAY_OPN_MODE_DISABLE_ECC) {
		*p_nand_props &= ~NAND_INTERNAL_ECC_ENABLED;
		printk(KERN_CRIT " intECC now disabled\n");
	}
	else {
		printk(KERN_CRIT " intECC still enabled\n");
	}
	return (bvalue == ARRAY_OPN_MODE_DISABLE_ECC);
}

static int EnableInternalECC(struct mtd_info *mtd, 
			     struct nand_chip *chip,
			     u32 * p_nand_props,
			     int cart_nand) 
{
	unsigned char bvalue;

	bvalue = SetInternalEccMode(mtd, chip, ARRAY_OPN_MODE_ENABLE_ECC);
	printk(KERN_CRIT "Trying to enable cart intECC; read back %d\n",bvalue);
	if (bvalue == ARRAY_OPN_MODE_ENABLE_ECC) {
		*p_nand_props |= NAND_INTERNAL_ECC_ENABLED;
		printk(KERN_CRIT " intECC now enabled\n");
	}
	else {
		printk(KERN_CRIT " intECC still disabled\n");
	}
	return (bvalue == ARRAY_OPN_MODE_ENABLE_ECC);
}


/* Routines that support the Micron MT29F4G08ABADA SLC NAND flash,
 * which can take care of ECC internally.
 *
 * cart_nand: nonzero ==> cartridge nand; if MT29F4G08ABADA, enable internal ecc
 */
static int chip_is_MT29F4G08ABADA(struct mtd_info *mtd, 
				  struct nand_chip *chip,
				  u32 * p_nand_props,
				  int cart_nand) 
{
	int dev_id, maf_id, three, four, five;

	/* Select the device */
	chip->select_chip(mtd, 0);

	/* wait a little bit after reset to see if this helps ID read problem*/
	mdelay(2);

	/* Send the command for reading device ID */
	chip->cmdfunc(mtd, NAND_CMD_READID, 0x00, -1);

	/* Read manufacturer and device IDs */
	maf_id  = chip->read_byte(mtd);
	dev_id  = chip->read_byte(mtd);
	three   = chip->read_byte(mtd);
	four    = chip->read_byte(mtd);
	five    = chip->read_byte(mtd);
	dev_crit(&nand.pdev->dev, 
		"MT29F... read from %s: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
       		 (cart_nand ? "cart" : "base"), 
		 maf_id, dev_id, three, four, five);

	if (   (maf_id == NAND_MFR_MICRON)
	    && (dev_id == 0xDC)
            && (three  == 0x90)
            && (four   == 0x95)
            && ((five  == 0xD6) || (five == 0x56))) {

	        int os1, os2, os3, os4;

		/* Send the command for reading ONFI signature device ID @ 0x20 */
		chip->cmdfunc(mtd, NAND_CMD_READID, 0x20, -1);
		os1 = chip->read_byte(mtd);
		os2 = chip->read_byte(mtd);
		os3 = chip->read_byte(mtd);
		os4 = chip->read_byte(mtd);
	        if (   ('O' == os1)
	            && ('N' == os2)
	            && ('F' == os3)
	            && ('I' == os4) )
	        {
			dev_crit(&nand.pdev->dev, "MT29F... found ONFI\n");
			*p_nand_props = NAND_SUPPORTS_INTERNAL_ECC
					| NAND_SUPPORTS_ONFI;
			if (five == 0xD6) 
				*p_nand_props |= NAND_INTERNAL_ECC_ENABLED;
			else if (cart_nand) {
				unsigned char bvalue;

				bvalue = SetInternalEccMode(mtd, chip,
						     ARRAY_OPN_MODE_ENABLE_ECC);
				printk(KERN_CRIT "Trying to enable cart intECC;"
						 " read back %d\n", bvalue);
				if (bvalue == ARRAY_OPN_MODE_ENABLE_ECC) {
					*p_nand_props |= NAND_INTERNAL_ECC_ENABLED;
					printk(KERN_CRIT " intECC now enabled\n");
				}
				else {
					printk(KERN_CRIT " intECC still disabled\n");
				}
			}
			return 1;
        	}
	        dev_crit(&nand.pdev->dev, "MT29F... found 0x%x, 0x%x, 0x%x, 0x%x\n",
        	        os1, os2, os3, os4);
	}
	return 0;
}


/**
 * lf1000_nand_read_page_internalEcc - Micron NAND page read function, using
 *                                     the NAND's internal ECC capability.
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 */
static int lf1000_nand_read_page_internalEcc(struct mtd_info  * mtd, 
                                      struct nand_chip * chip,
            	                      uint8_t          * buf)
{ 
	tpIO A = chip->IO_ADDR_R;
	uint8_t *b;
	int      len;
	int      rem;
	char     status;

	/* Read status byte, waiting for the NAND to read the page into its
	 * internal buffer. */
	do {
	        chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	        status = readb(A);
	} while (!(status & NAND_STATUS_READY));

	/* If FAILed, increment mtd->ecc_stats.failed and return -1 */
	if (status & NAND_STATUS_FAIL) {
	        mtd->ecc_stats.failed++;
	        return -1;
	}
#define MT29F_REWRITE_RECOMMENDED   0x08
	if (status & MT29F_REWRITE_RECOMMENDED) {
	        mtd->ecc_stats.corrected++;
	}
	/* Now reissue the Read Mode command to return to Read Mode */
	chip->cmd_ctrl(mtd, NAND_CMD_READ0,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	b   = buf;
	len = mtd->writesize;
	rem = (3 & (unsigned int)b);
	if (rem) {
	        while ( (len > 0) && (rem < 4)) {
		        *b++ = readb(A);
		        ++rem;
		        --len;
	        }
	}
	if (0 == (3 & (unsigned int)b)) {
	        u32 * p = (u32 *)b;

	        for (; len > 63; len -= 64) {
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
	        }
	        for (; len > 3; len -= 4) {
		        *p++ = readl(A);
	        }
	        b = (uint8_t *)p;
		while (len-- > 0) {
		        *b++ = readb(A);
	        }
	}
	else {
	        dev_info(&nand.pdev->dev, 
			"!@#$ lf1000_nand_read_page_internalECC()\n");
	        for ( ; len > 0; --len) {
		        *b++ = readb(A);
	        }
	}
	return 0;
}



/**
 * lf1000_nand_write_page_internalECC - write, using NAND's internal ECC
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 *
 * Writes the buffer's data to the page in one operation.
 *
 * @Details:	nand_write_page(), which calls this routine, has already
 *		supervised processing that prepares for writing a page by
 * doing the following:
 *		send a Write command (0x80)
 *		send the start-of-page address
 *
 * This routine does not send a Start Programming command to the device.  It
 * depends on its caller to do that and to check if the write was completed ok.
 */

static void lf1000_nand_write_page_internalEcc( struct mtd_info  * mtd, 
                                         	struct nand_chip * chip,
		                                const uint8_t    * buf)
{
	chip->write_buf(mtd, buf, mtd->writesize);
}

/**
 * lf1000_nand_read_subpage_internalEcc - sub-page read function
 *		for use with a NAND flash that has internal ECC
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @data_offs:	offset of requested data within the page
 * @readlen:	data length
 * @bufpoi:	buffer to store read data
 */
static int lf1000_nand_read_subpage_internalEcc(struct mtd_info  *mtd, 
					        struct nand_chip *chip, 
					        uint32_t	  data_offs, 
					        uint32_t	  readlen, 
					        uint8_t		 *buf)
{
	tpIO A = chip->IO_ADDR_R;
	uint8_t *b;
	int      len;
	int      rem;
	char     status;

	/* Read status byte, waiting for the NAND to read the page into its
	 * internal buffer. */
	do {
	        chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	        status = readb(A);
	} while (!(status & NAND_STATUS_READY));

	/* If FAILed, increment mtd->ecc_stats.failed and return -1 */
	if (status & NAND_STATUS_FAIL) {
	        mtd->ecc_stats.failed++;
	        return -1;
	}
#define MT29F_REWRITE_RECOMMENDED   0x08
	if (status & MT29F_REWRITE_RECOMMENDED) {
	        mtd->ecc_stats.corrected++;
	}
	/* Now reissue the Read Mode command to return to Read Mode */
	chip->cmd_ctrl(mtd, NAND_CMD_READ0,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	chip->cmd_ctrl(mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/* and position to the intra-page offset (if necessary) */
	/* TODO: add the code */

	b   = buf;
	len = readlen;
	rem = (3 & (unsigned int)b);
	if (rem) {
	        while ( (len > 0) && (rem < 4)) {
		        *b++ = readb(A);
		        ++rem;
		        --len;
	        }
	}
	if (0 == (3 & (unsigned int)b)) {
	        u32 * p = (u32 *)b;

	        for (; len > 63; len -= 64) {
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
		        *p++ = readl(A);
	        }
	        for (; len > 3; len -= 4) {
		        *p++ = readl(A);
	        }
	        b = (uint8_t *)p;
		while (len-- > 0) {
		        *b++ = readb(A);
	        }
	}
	else {
	        dev_info(&nand.pdev->dev, 
			"!@#$ lf1000_nand_read_subpage_internalEcc()\n");
	        for ( ; len > 0; --len) {
		        *b++ = readb(A);
	        }
	}
	return 0;
}


static void lf1000_prepare_for_InternalEcc( struct nand_chip * pchip) {
/* TODO: FIXME: add call to set feature (internal ECC) */
/* (For now, depend on emerald-boot to do that */

	pchip->ecc.mode           = NAND_ECC_INTERNAL;
	pchip->ecc.size           = 512;
	pchip->ecc.bytes          = 8;
          /* When ecc.mode is NAND_ECC_HW_SYNDROME, these 3 must be non-null */
	pchip->ecc.hwctl          = NULL;
	pchip->ecc.calculate      = NULL;
	pchip->ecc.correct        = NULL;

	pchip->ecc.read_page      = lf1000_nand_read_page_internalEcc;
	pchip->ecc.write_page     = lf1000_nand_write_page_internalEcc;
	pchip->ecc.read_subpage   = lf1000_nand_read_subpage_internalEcc;
	pchip->ecc.read_page_raw  = nand_read_page_raw;
	pchip->ecc.write_page_raw = nand_write_page_raw;
	pchip->ecc.read_oob       = nand_read_oob_std;
	pchip->ecc.write_oob      = nand_write_oob_std;
	pchip->ecc.layout         = &micron_nand_oob_internal_ecc;
}

/* Enable this #define if you don't want to use the internal ECC capability
 * of the MT29F4G08ABADA nand flash.  
 * NOTE: You must also change emerald-boot's nand_utils.c so it won't enable
 * the device's internal ECC mode.
 * #define DONT_USE_MT29F4G08ABADA_INTERNAL_ECC 1
 *
 * NOTE: it would be better to put the mode selection code in
 *	 chip_is_MT29F4G08ABADA().  Have it set a variable that indicates
 * whether internal or external ECC mode is enabled.
 */
static void lf1000_init_for_MT29F4G08ABADA(struct mtd_info *mtd, 
                                           struct nand_chip *chip,
					   u32 * p_nand_props)
{
	if (*p_nand_props & NAND_INTERNAL_ECC_ENABLED) {
		/* Prepare to use internally calculated & checked 4-bit ECC */
		lf1000_prepare_for_InternalEcc( chip );
		dev_info(&nand.pdev->dev, "Using Internal ECC\n");
		mtd->subpage_sft  = 2;
		chip->verify_buf  = lf1000_verify_SLC_page; 
		chip->subpagesize = mtd->writesize >> mtd->subpage_sft;
	}
	else {
		lf1000_prepare_for_4BitEcc( chip );
		dev_info(&nand.pdev->dev, "Using External 4-bit ECC\n");
		chip->verify_buf  = lf1000_verify_MLC_page;
		/* TODO: figure out what we ought to do in this case */
		chip->options    |= NAND_NO_SUBPAGE_WRITE;
		mtd->subpage_sft  = 0;
		chip->subpagesize = mtd->writesize >> mtd->subpage_sft;
	}
}




