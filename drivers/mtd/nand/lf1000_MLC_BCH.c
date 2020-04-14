/*
 *  drivers/mtd/nand/lf1000_MLC_BCH.c
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
 * This file contains routines and data for dealing with MLC NAND flash and/or
 * BCH ECC.
 */

static struct nand_ecclayout nand_oob_64_BCH = {
	.eccbytes = 28,
	.eccpos = {36, 37, 38, 39, 40, 41, 42, 
	           43, 44, 45, 46, 47, 48, 49,
        	   50, 51, 52, 53, 54, 55, 56, 
	           57, 58, 59, 60, 61, 62, 63 },
	.oobfree = {
		{.offset = 2,
		 .length = 34}}
};

static struct nand_ecclayout nand_oob_128_BCH = {
	.eccbytes = 56,
	.eccpos = { 72,  73,  74,  75,  76,  77,  78, 
	            79,  80,  81,  82,  83,  84,  85, 
	            86,  87,  88,  89,  90,  91,  92, 
	            93,  94,  95,  96,  97,  98,  99, 
	           100, 101, 102, 103, 104, 105, 106,
	           107, 108, 109, 110, 111, 112, 113,
	           114, 115, 116, 117, 118, 119, 120,
	           121, 122, 123, 124, 125, 126, 127 },
	.oobfree = {
		{.offset = 2,
		 .length = 70}}
};

/**
 * lf1000_nand_read_subpage_BCH  Leapfrog's version for nand_read_subpage()
 *                               for use with BCH ECC
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @data_offs:	offset of requested data within the page
 * @readlen:	data length
 * @bufpoi:	buffer to store read data
 *
 * This function sends a RNDOUT command and read the ECC bytes from the OOB,
 * then send a RNDOUT command for the start of the subpage section, then
 * (repeating as necessary) reset the NAND controller's ECC engine, load ECC
 * bytes into the LF1000's ORGECC registers, read an entire 512-byte section
 * into the buffer, check for errors and correct them if possible.  Continue
 * until an uncorrectable error is detected or until all requested bytes have
 * been read into bufpoi.
 */
static int lf1000_nand_read_subpage_BCH(struct mtd_info  *mtd, 
                                        struct nand_chip *chip, 
                                        uint32_t          data_offs, 
                                        uint32_t          readlen, 
                                        uint8_t          *bufpoi)
{
	int i;
	int start_step;
	int end_step;
	int num_steps;
	int data_col_addr;
	int datafrag_len;
	int eccfrag_len;
	int eccOffset;  /* from start of sectionECC[] to start of ECC bytes
                           for current section */
	int eccbytes = chip->ecc.bytes;
	int eccsize  = chip->ecc.size;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	uint8_t  *buf;
	uint8_t   sectionECC[MAX_ECC_BYTES_PER_PAGE]; 
                                /* buffer for all of the page's ECC bytes */
	uint32_t allFF;
	uint8_t  eccAllFF;
	uint32_t hdweFoundErrors;
	tpIO A = chip->IO_ADDR_R;


	    /* Column address within the page aligned to ECC size */
	start_step = data_offs / eccsize;
	end_step   = (data_offs + readlen - 1) / eccsize;
	num_steps  = end_step - start_step + 1;

	    /* Data size aligned to ECC ecc.size*/
	datafrag_len = num_steps * eccsize;
	eccfrag_len  = num_steps * eccbytes;

        /* first read all of the page's ECC bytes into sectionECC */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, mtd->writesize + *eccpos, -1);
	chip->read_buf(mtd, sectionECC, chip->ecc.layout->eccbytes);
	eccOffset = start_step * eccbytes;

        /* point to the start of the first subpage we'll read */
	data_col_addr = start_step * eccsize;
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, data_col_addr, -1);

	buf = bufpoi + data_col_addr;
	for (i = 0; i < num_steps; 
	     ++i, data_col_addr += eccsize, eccOffset += eccbytes)
	{
       		int      len;
	        uint8_t *b;
		int      rem;
		uint8_t  val8;
		uint32_t val32;
		int	 eccSubIndex;
		uint8_t *pecc;
    		u32 ctl = readl((tpIO)(NAND_BASE+NFCONTROL));
	        writel((ctl & (NFC_NFTYPE_MASK | NFC_NFBANK_MASK)) 
        	        | NFC_ECCRST_MASK, 
        	        (tpIO)(NAND_BASE+NFCONTROL)); 
	        val32 = sectionECC[eccOffset] 
        	        + (sectionECC[eccOffset+1] << 8)
        	        + (sectionECC[eccOffset+2] << 16)
        	        + (sectionECC[eccOffset+3] << 24);
	        writel(val32, (tpIO)(NAND_BASE+NFORGECCL)); 

	        val32 = sectionECC[eccOffset+4] 
        	        + (sectionECC[eccOffset+5] << 8)
        	        + (sectionECC[eccOffset+6] << 16);
        	writel(val32, (tpIO)(NAND_BASE+NFORGECCH)); 

		/* Read 512 bytes into the current section of 'buf'*/
        	len = eccsize;
		b   = buf;
		rem = (3 & (unsigned int)b);

		if (rem) {
			allFF = 0x000000FF;
			while ( (len > 0) && (rem < 4)) {
				val8   = readb((tpIO)A);
				allFF &= val8;
				*b++   = val8;
				++rem;
				--len;
			}
			if (allFF == 0x000000FF) {
				allFF = 0xFFFFFFFF;
			}
		}
		else {
			allFF = 0xFFFFFFFF;
		}
		if (0 == (3 & (unsigned int)b)) {
			u32 * p = (u32 *)b;

			for (; len > 63; len -= 64) {
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
				val32 = readl(A); allFF &= val32; *p++ = val32;
			}
			for (; len > 3; len -= 4) {
				val32 = readl(A); allFF &= val32; *p++ = val32;
			}
			b = (uint8_t *)p;
			while (len-- > 0) {
				val8 = readb(A);
				if (val8 != 0xFF) {
					allFF = 0;
				}
				*b++ = val8;
			}
		}
		else {
			dev_info(&nand.pdev->dev, 
				"!@#$ lf1000_nand_read_subpage_BCH()\n");
			allFF = 0x000000FF;
			for ( ; len > 0; --len) {
				val8   = readb(A);
				allFF &= val8;
				*b++   = val8;
			}
			if (allFF == 0x000000FF) {
				allFF = 0xFFFFFFFF;
			}
		}
		/* Wait until the NFECCDECDONE bit is set in the 
		 * NFECCSTATUS register.
		 */
		/* while we wait, check if all ECC bytes are FF */
		eccAllFF = 0xFF;
		for (eccSubIndex = 0, pecc = &sectionECC[eccOffset]; 
		     eccSubIndex < eccbytes; ++eccSubIndex)
		{
	                eccAllFF &= *pecc++;
		}

		while(IS_CLR(readl((tpIO)(NAND_BASE+NFECCSTATUS)),NFECCDECDONE))
			;
		/* Check the NFCHECKERROR bit in the NFECCSTATUS register.
		 * If there's an error, read the syndrome values from
		 * NFSYNDROME31 and NFSYNDROME75 registers, save them, and use
		 * them to try to correct the error.
		 */
		hdweFoundErrors = IS_SET(readl((tpIO)(NAND_BASE+NFECCSTATUS)),
					 NFCHECKERROR);
		/* Check for eccAllFF and a few non-FF data bytes
		 * if lf1000 hdwe indicates error 
		 *   AND either data or ecc contains non-FF bytes,
		 * then look a little closer.
		 *
		 * Skip this processing if hdwe indicate ok 
		 *                  OR (all data bytes AND all ECC bytes are FF)
		 */
		if (   hdweFoundErrors
		    && ((allFF != 0xFFFFFFFF) || (eccAllFF != 0xFF)))
		{
			/* if all ecc bytes are FF, check if only a few data 
			 * bytes are not FF.  In that case, set all data bytes
			 * to FF and report corrected ECC errors.
			*/
			if (   (eccAllFF == 0xFF)
			    && (num_zero_bits_u32(allFF) <= 4)) 
			{
				uint8_t *b   = buf;
				int      count;
				int      j;
				/* NOTE: this can be speeded up if necessary;
				 *       keeping it simple and short for now.
				 */
				count = 0;
				for (j = 0; (j < eccsize) && (count < 5); j++) {
					count += num_zero_bits_u8(*b++);
				}
				/* Now if count < 5, it's the number of zero
				 * data bits in the buffer.  Treat this as an
				 * erased section in which a few bits have been
				 * flipped.  Add 'count' to the number of
				 * (corrected) errors.
				 */
				if (count < 5) {
					memset( buf, 0xFF, eccsize);
					total_bitflips           += count;
					mtd->ecc_stats.corrected += count;
					allFF = 0xFFFFFFFF;
					dev_info(&nand.pdev->dev, 
						"Found %d 0-bits in mostly-FF"
						" section %d of page;"
						" set all to FF\n",
						count, i);
				}
			}
			/* NOTE: we might find it necessary to check if the
			 *	 total number of zero bits in data and ECC is
			 * < 5 and force all data bytes to FF in that case.
			 */
		}
		if (   hdweFoundErrors && (allFF != 0xFFFFFFFF)) {
			int TryToCorrectBCH_Errors(u8 * pData);
			int numErrors;

			numErrors = TryToCorrectBCH_Errors( buf );
			if (numErrors < 0) { /* uncorrectable errors */
				mtd->ecc_stats.failed++;
				dev_info(&nand.pdev->dev, 
					"Found uncorrectable ECC errors (%d) "
					"in section %d of page\n", 
					 numErrors, i);
				dev_info(&nand.pdev->dev, 
					"  Read ECC bytes: %02x %02x "
					"%02x %02x %02x %02x %02x\n",
					sectionECC[eccOffset+0], 
					sectionECC[eccOffset+1], 
					sectionECC[eccOffset+2], 
					sectionECC[eccOffset+3], 
					sectionECC[eccOffset+4], 
					sectionECC[eccOffset+5], 
					sectionECC[eccOffset+6]);
				for (i = 0; i < eccsize; i += 16) {
					dev_info(&nand.pdev->dev,
						"%02x %02x %02x %02x "
						"%02x %02x %02x %02x "
						"%02x %02x %02x %02x "
						"%02x %02x %02x %02x\n",
						buf[i], buf[i+1], buf[i+2], 
						buf[i+3], buf[i+4], buf[i+5], 
						buf[i+6], buf[i+7], buf[i+8], 
						buf[i+9], buf[i+10], buf[i+11],
						buf[i+12], buf[i+13], buf[i+14], 
						buf[i+15]);
				}
				return numErrors;
			}
			else if (numErrors > 0) {
				total_bitflips           += numErrors;
				mtd->ecc_stats.corrected += numErrors;
				dev_info(&nand.pdev->dev, 
					"Corrected %d ECC errors "
					"in section %d of page\n", 
					numErrors, i);
			}
		}
		buf += eccsize;
	}
	return 0;
}

/**
 * lf1000_nand_read_page_BCH - LF1000 BCH-ecc-based page read function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	buffer to store read data
 */
static int lf1000_nand_read_page_BCH( struct mtd_info  * mtd, 
	                              struct nand_chip * chip,
	    	                      uint8_t          * buf)
{   
	int i;
	int dataOffset; /* from start of page to start of current section */
	int eccOffset;  /* from start of page to start of current ECC section */
	int eccsize  = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	tpIO A = chip->IO_ADDR_R;
	int j;
	uint32_t *eccpos = chip->ecc.layout->eccpos;
	uint8_t sectionECC[MAX_ECC_BYTES_PER_PAGE]; 
                                /* buffer for all of the page's ECC bytes */
	uint32_t allFF;
	uint8_t  eccAllFF;
	uint32_t hdweFoundErrors;

	dataOffset = 0;
	eccOffset  = mtd->writesize + *eccpos;

	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, eccOffset, -1);
	chip->read_buf(mtd, sectionECC, chip->ecc.layout->eccbytes);
	eccOffset  = 0; /* now eccOffset is offset into sectionECC[] */

	    /* Prepare to read the section's data */
	chip->cmdfunc(mtd, NAND_CMD_RNDOUT, dataOffset, -1);
	for (i = 0; i < eccsteps; ++i, dataOffset += eccsize, 
					eccOffset += eccbytes) {
		unsigned int val32;
	    	u32 ctl = readl((tpIO)(NAND_BASE+NFCONTROL));
		writel((ctl & (NFC_NFTYPE_MASK | NFC_NFBANK_MASK)) 
			| NFC_ECCRST_MASK, 
		        (tpIO)(NAND_BASE+NFCONTROL)); 
		        /* set ECCRST, keeping NFTYPE and NFBANK */
		val32 = sectionECC[eccOffset] 
		        + (sectionECC[eccOffset+1] << 8)
		        + (sectionECC[eccOffset+2] << 16)
		        + (sectionECC[eccOffset+3] << 24);
		writel(val32, (tpIO)(NAND_BASE+NFORGECCL)); 

		val32 = sectionECC[eccOffset+4] 
		        + (sectionECC[eccOffset+5] << 8)
		        + (sectionECC[eccOffset+6] << 16);
		writel(val32, (tpIO)(NAND_BASE+NFORGECCH)); 

		    /* Read 512 bytes and store them into the current section 
		     * of 'buf'*/
	        {
		int      len = eccsize;
	        uint8_t *b   = buf;
		int      rem = (3 & (unsigned int)b);
		uint8_t  val8;
		uint32_t val32;

		if (rem) {
			allFF = 0x000000FF;
                	while ( (len > 0) && (rem < 4)) {
                		val8   = readb(A);
		                allFF &= val8;
				*b++   = val8;
		                ++rem;
		                --len;
	                }
	                if (allFF == 0x000000FF)
	                    allFF = 0xFFFFFFFF;
		}
	        else allFF = 0xFFFFFFFF;

		if (0 == (3 & (unsigned int)b)) {
	                u32 * p = (u32 *)b;

		        for (; len > 63; len -= 64) {
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        	val32 = readl(A); allFF &= val32; *p++ = val32;
		        }
			for (; len > 3; len -= 4) {
		            val32 = readl(A); allFF &= val32; *p++ = val32;
		        }
		        b = (uint8_t *)p;
			while (len-- > 0) {
		        	val8 = readb(A);
		        	if (val8 != 0xFF) {
		        	        allFF = 0;
		        	}
				*b++ = val8;
		        }
		}
		else { /* unexpected condition (unaligned pointer) */
		        dev_info(&nand.pdev->dev, 
				 "!@#$ lf1000_nand_read_page_BCH()\n");
		        allFF = 0x000000FF;
			for ( ; len > 0; --len) {
				val8   = readb(A);
			        allFF &= val8;
				*b++   = val8;
			}
		        if (allFF == 0x000000FF)
		            allFF = 0xFFFFFFFF;
		}
        	}
		    /* Wait until the NFECCDECDONE bit is set in the 
		     * NFECCSTATUS register.
		     */
        	{   /* while we wait, check if all ECC bytes are FF */
		int eccSubIndex;
		uint8_t * pecc;
            
		eccAllFF = 0xFF;
		for (eccSubIndex = 0, pecc = &sectionECC[eccOffset]; 
                     eccSubIndex < eccbytes; ++eccSubIndex)
		{
	                eccAllFF &= *pecc++;
		}
	        }

	        while(IS_CLR(readl((tpIO)(NAND_BASE+NFECCSTATUS)),NFECCDECDONE))
	            ;
		/* Check the NFCHECKERROR bit in the NFECCSTATUS register.
		 * If there's an error, read the syndrome values from
		 * NFSYNDROME31 and NFSYNDROME75 registers, save them, 
		 * and use them to try to correct the error.
		 */
	        hdweFoundErrors = IS_SET(readl((tpIO)(NAND_BASE+NFECCSTATUS)),
        	                         NFCHECKERROR);
            	/* Check for eccAllFF and a few non-FF data bytes
        	 * if lf1000 hdwe indicates error 
        	 *   AND either data or ecc contains non-FF bytes,
        	 * then look a little closer.
        	 *
        	 * Skip this processing if hdwe indicate ok 
        	 *                 OR (all data bytes AND all ECC bytes are FF)
		 */
	        if (   hdweFoundErrors
        	    && ((allFF != 0xFFFFFFFF) || (eccAllFF != 0xFF)))
        	{
        		/* if all ecc bytes are FF, check if only a few data 
			 * bytes are not FF.  In that case, set all data bytes
			 * to FF and report corrected ECC errors.
			 */
		        if (   (eccAllFF == 0xFF)
			    && (num_zero_bits_u32(allFF) <= 4)) {
	        		uint8_t *b   = buf;
                		int      count;

				/* NOTE: this can be speeded up if necessary;
				 *       keeping it simple and short for now.
				 */
				count = 0;
				for (j = 0; (j < eccsize) && (count < 5); j++) {
				    count += num_zero_bits_u8(*b++);
				}
				/* Now if count < 5, it's the number of zero 
				 * data bits in the buffer.  Treat this as an
				 * erased section in which a few bits have been
				 * flipped.  Add 'count' to the number of
				 * (corrected) errors.
				 */
				if (count < 5) {
					memset( buf, 0xFF, eccsize);
					mtd->ecc_stats.corrected += count;
					allFF = 0xFFFFFFFF;
					dev_info(&nand.pdev->dev, 
						 "Found %d 0-bits in mostly-FF "
						 " section %d of page;"
						 " set all to FF\n",
				           	 count, i);
		                }
            		}
		/* NOTE: we might find it necessary to check if the total number
		 *	 of zero bits in data and ECC is < 5 and force all data
		 *	 bytes to FF in that case.
		 */
	        }
		if (   hdweFoundErrors && (allFF != 0xFFFFFFFF)) {
		        int TryToCorrectBCH_Errors(u8 * pData);
		        int numErrors;

			numErrors = TryToCorrectBCH_Errors( buf );
			if (numErrors < 0) { /* uncorrectable errors */
				mtd->ecc_stats.failed++;
				dev_info(&nand.pdev->dev, 
					 "Found uncorrectable ECC errors (%d) "
			                 "in section %d of page\n", 
					 numErrors, i);
				dev_info(&nand.pdev->dev,
					 "  Read ECC bytes: %02x %02x "
			                 "%02x %02x %02x %02x %02x\n",
			        	 sectionECC[eccOffset+0], 
					 sectionECC[eccOffset+1], 
				         sectionECC[eccOffset+2],
					 sectionECC[eccOffset+3], 
				         sectionECC[eccOffset+4],
					 sectionECC[eccOffset+5], 
				         sectionECC[eccOffset+6]);

				for (i = 0; i < eccsize; i += 16) {
				    dev_info(&nand.pdev->dev,
				      "%02x %02x %02x %02x %02x %02x %02x %02x "
				      "%02x %02x %02x %02x %02x %02x %02x %02x\n",
				       buf[i], buf[i+1], buf[i+2], buf[i+3],
				       buf[i+4], buf[i+5], buf[i+6], buf[i+7],
				       buf[i+8], buf[i+9], buf[i+10], buf[i+11],
				       buf[i+12], buf[i+13], buf[i+14], buf[i+15]);
				}
		        	return numErrors;
        		}
		        else if (numErrors > 0) {
				total_bitflips += numErrors;
				mtd->ecc_stats.corrected += numErrors;
				dev_info(&nand.pdev->dev,
					 "Corrected %d ECC errors "
			                 "in section %d of page\n", 
					 numErrors, i);
			}
        	}
        	buf += eccsize;
	}
	return 0;
}



/**
 * lf1000_nand_write_page_BCH - LF1000 BCH-ecc-based page write function
 * @mtd:	mtd info structure
 * @chip:	nand chip info structure
 * @buf:	data buffer
 *
 * Writes the buffer's data to the page in 512-byte sections.  After each
 * 512-byte section, reads the 7-byte ECC value from the LF1000's ECC registers
 * and writes it to the appropriate place eccbuf[].  After all the data bytes
 * have been written to the page, this routine writes the ECC bytes to the
 * page's spare area (OOB).
 *
 * @Details:	nand_write_page(), which calls this routine, has already
 *				supervised processing that prepares for writing a page by
 * doing the following:
 *		send a Write command (0x80)
 *		send the start-of-page address
 *
 * On each pass through its loop, this routine does what's needed to write a
 * 512-byte section of the page to the NAND device.
 *
 * 		Sets the ECCRST bit in the NFCONTROL register, resetting the ECC
 *      calculation hardware.
 *	
 *		Next the routine calls chip->write_buf() to write the 512 data bytes
 *		to the device.
 *
 *		After the NAND controller completes its calculation of the ECC bytes
 *		for the section's data, this routine reads the values from the 
 *		controller's NFECCL and NFECCH registers and stores them in eccbuf[].
 *
 *      After writing all the data to the page, the routine calls 
 *      chip->cmdfunc() to send a RNDIN command, with the offset of location
 *      in the spare area where ECC bytes ought to be written.  Then it calls
 *		chip->write_buf() to write the page's ECC bytes to the flash.
 *
 * This routine does not send a Start Programming command to the device.  It
 * depends on its caller to do that and to check if the write was completed ok.
 */

/*
 * FIXME: TODO: Do we need a timeout on the wait for 'NFECCENCDONE'?
 */

static void lf1000_nand_write_page_BCH( struct mtd_info  * mtd, 
	                                struct nand_chip * chip,
		                        const uint8_t    * buf)
{
	unsigned int  eccl;
	unsigned int  ecch;
	unsigned char eccbuf[MAX_ECC_BYTES_PER_PAGE];

	int i;
	int dataOffset; /* from start of page to start of current section */
	int eccOffset;  /* from start of page to start of current ECC section */
	int eccsize  = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	int *eccpos  = chip->ecc.layout->eccpos;

	dataOffset = 0;
	eccOffset  = 0;	    /* offset into eccbuf */
	/*  for each section of page */
	for (i = 0; i < eccsteps; ++i, dataOffset += eccsize,
					eccOffset += eccbytes) {
	        int allFF;

    		u32 ctl = readl((tpIO)(NAND_BASE+NFCONTROL));
        	writel( (ctl & (NFC_NFTYPE_MASK | NFC_NFBANK_MASK))
			 | NFC_ECCRST_MASK, 
	                (tpIO)(NAND_BASE+NFCONTROL)); 
		chip->write_buf(mtd, buf + dataOffset, eccsize);

		/* This routine won't be called to write a buffer with all FFs 
		 * to NAND, but subsections of the buffer might contain only 
		 * FFs.  If no subpage writes are allowed, and if a subsection
		 * contains only FFs, go ahead and store the ECC bytes computed
		 * by the LF1000 hardware.
		 * If subpage writes are allowed (e.g., SLC), and if a
		 * subsection contains only FFs, substitute FFs for the 
		 * section's computed ECC bytes (which are 0xE9, 0x1C, 0x9E,
		 * 0xD5, 0xB9, 0x92, 0x0B).  This substitution allows another
		 * write of non-FF values to the same section of the page
		 * without garbling the stored ECC bytes.
		 * For example there might be two writes to a page when we use
		 * 4-bit ECC on SLC and configure UBI to store both EC header
		 * and VID header in the same page.  After a block is erased,
		 * its EC header is written to the first section of the block's
		 * first page.  Later when the physical block is mapped to a
		 * logical block, a VID header is written to the second section
		 * of the block's first page.
		 */
	        allFF = 0;
		if (mtd->subpage_sft != 0)
		        allFF = all_bytes_ff( &buf[dataOffset], eccsize);

	        while(IS_CLR(readl((tpIO)(NAND_BASE+NFECCSTATUS)), NFECCENCDONE))
	        	;

		eccl = readl((tpIO)(NAND_BASE+NFECCL)); 
		ecch = readl((tpIO)(NAND_BASE+NFECCH));

		if (!allFF) { /* store the computed ECC bytes */
			eccbuf[eccOffset+0] =  eccl        & 0x000000FF; 
			eccbuf[eccOffset+1] = (eccl >> 8)  & 0x000000FF; 
			eccbuf[eccOffset+2] = (eccl >> 16) & 0x000000FF; 
			eccbuf[eccOffset+3] = (eccl >> 24) & 0x000000FF; 
			eccbuf[eccOffset+4] =  ecch        & 0x000000FF; 
			eccbuf[eccOffset+5] = (ecch >> 8)  & 0x000000FF; 
			eccbuf[eccOffset+6] = (ecch >> 16) & 0x000000FF; 
		}
		else { /* all FF, so just store FF as ECC */
			eccbuf[eccOffset+0] = 0xFF; 
			eccbuf[eccOffset+1] = 0xFF; 
			eccbuf[eccOffset+2] = 0xFF; 
			eccbuf[eccOffset+3] = 0xFF; 
			eccbuf[eccOffset+4] = 0xFF; 
			eccbuf[eccOffset+5] = 0xFF; 
			eccbuf[eccOffset+6] = 0xFF; 
			allFF = 0;
	        }
	}
        /* Write all ECC bytes at once */
	chip->cmdfunc(mtd, NAND_CMD_RNDIN, mtd->writesize + *eccpos, -1);
	chip->write_buf(mtd, eccbuf, eccOffset);
}

/*
 * The LF1000 hardware ECC is always enabled so no initialization is needed 
 * here.
 */ 
static void lf1000_nand_enable_hwecc_BCH(struct mtd_info *mtd, int mode) {
}


/**
 * lf1000_nand_calculate_BCH - calculate BCH ECC for 512-byte buffer
 *			 block
 * @mtd:	MTD block structure
 * @buf:	input buffer with raw data
 * @code:	output buffer with ECC
 */
static int lf1000_nand_calculate_BCH(struct mtd_info     * mtd, 
                              const unsigned char * buf,
		                      unsigned char       * code) 
{
	/* not actually needed for BCH; just return 0 */
	return 0;
}


/**
 * lf1000_nand_correct_BCH - Detect and correct bit error(s)
 * @mtd:	MTD block structure
 * @buf:	raw data read from the chip
 * @read_ecc:	ECC from the chip
 * @calc_ecc:	the ECC calculated from raw data
 *
 * Detect and correct up to 4 bit errors for 512 byte block
 */
static int lf1000_nand_correct_BCH(struct mtd_info * mtd, 
                            unsigned char   * buf,
		                    unsigned char   * read_ecc, 
                            unsigned char   * calc_ecc)
{
	/* not actually needed for BCH; just return 0 */
	return 0;
}



static void lf1000_prepare_for_4BitEcc( struct nand_chip * pchip) {
	pchip->ecc.mode           = NAND_ECC_HW_SYNDROME;
	pchip->ecc.size           = 512;
	pchip->ecc.bytes          = 7;
          /* When ecc.mode is NAND_ECC_HW_SYNDROME, these 3 must be non-null */
	pchip->ecc.hwctl          = lf1000_nand_enable_hwecc_BCH;
	pchip->ecc.calculate      = lf1000_nand_calculate_BCH;
	pchip->ecc.correct        = lf1000_nand_correct_BCH;

	pchip->ecc.read_page      = lf1000_nand_read_page_BCH;
	pchip->ecc.write_page     = lf1000_nand_write_page_BCH;
	pchip->ecc.read_page_raw  = nand_read_page_raw;
	pchip->ecc.write_page_raw = nand_write_page_raw;
	pchip->ecc.read_oob       = nand_read_oob_std;
	pchip->ecc.write_oob      = nand_write_oob_std;
	pchip->ecc.layout         = &nand_oob_64_BCH;
}


/**
 * MLC_BlockBad - checks if a specified block of a Samsung MLC flash
 *                   is bad.
 *          Used for these chips instead of nand_block_bad
 * @mtd:    MTD device structure
 * @ofs:    offset from device start
 * @getchip:    0, if the chip is already selected
 *
 * Check if the block has been marked bad.  Return nonzero if it has.
 *
 * @Details:	This function calculates the index of the block in which the
 *		specified offset (ofs) lies.  Then it calculates the index
 * of the last page of the block.  Finally it reads the first byte of the spare
 * area of the last page, whose value indicates whether or not the block has 
 * been marked bad (0xFF == good; anything else == bad).
 */
static int MLC_BlockBad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	int page, chipnr, res = 0;
	struct nand_chip *chip = mtd->priv;
	int block;
	int blockSize;
	int pageSize;

	if (getchip) {
		page   = (int)(ofs >> chip->page_shift);
		chipnr = (int)(ofs >> chip->chip_shift);

		nand_get_device(chip, mtd, FL_READING);

		/* Select the NAND device */
		chip->select_chip(mtd, chipnr);
	} else
		page = (int)ofs;

		/* Calculate page as page index of block's last page */
	blockSize = 1 << chip->phys_erase_shift;
	pageSize  = 1 << chip->page_shift;
	block     = ( ((int)ofs) >> chip->phys_erase_shift);
	ofs       = (block << chip->phys_erase_shift);
	ofs      += (blockSize - pageSize);
	page      = (ofs >> chip->page_shift);

	chip->cmdfunc(mtd, NAND_CMD_READOOB, chip->badblockpos,
			      page & chip->pagemask);
	if (chip->read_byte(mtd) != 0xff) {
		res = 1;
	        dev_crit(&nand.pdev->dev, "Block 0x%x (page 0x%x) is BAD\n", 
			 block, page); 
	}
	if (getchip)
		nand_release_device(mtd);

	return res;
}

/**
 * MLC_BlockMarkBad - marks a specified block of a Samsung MLC flash
 *                       bad.
 *              Used for these chips instead of nand_default_block_markbad
 * @mtd:    MTD device structure
 * @ofs:    offset from device start
 *
 * This function marks the flash block bad and updates the bad block table.  
 * It returns zero if it completed its processing without detecting an error.
*/
/* FIXME: TODO: Because the chip doesn't allow subpage writes, do we need  
 *      to erase the block before we can write the bad block indicator?? 
 */
static int MLC_BlockMarkBad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	uint8_t buf[2] = { 0, 0 };
	int block, ret;


	/* Get block number */
	block = ((int)ofs) >> chip->bbt_erase_shift;
	if (chip->bbt)
		chip->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	/* Do we have a flash based bad block table ? */
	if (chip->options & NAND_USE_FLASH_BBT)
		ret = nand_update_bbt(mtd, ofs);
	else {
		/* We write two bytes, so we dont have to mess with 16 bit
		 * access
		 * Calculate offset to start of last page of block
		 */
		int blockSize;
		int pageSize;
		blockSize = 1 << chip->phys_erase_shift;
		pageSize  = 1 << chip->page_shift;
		block     = ((int)ofs) >> chip->phys_erase_shift;
		ofs       = block << chip->phys_erase_shift;
		ofs      += (blockSize - pageSize);

		chip->ops.len     = chip->ops.ooblen = 2;
		chip->ops.datbuf  = NULL;
		chip->ops.oobbuf  = buf;
		chip->ops.ooboffs = chip->badblockpos & ~0x01;

		ret = nand_do_write_oob(mtd, ofs, &chip->ops);
		/* Return 0 if nand_do_write_oob() fails because of
	         * a write error -- it might happen on a truly bad block.
	         * Return 0 so ubi_io_mark_bad() won't return an error indicator
        	 * to erase_worker(), and erase_worker() won't put the entire
        	 * ubi device into read-only mode.
		 */
		if (-EIO == ret) {
			ret = 0;
			dev_crit(&nand.pdev->dev, 
			 "MLC_BlockMarkBad: -EIO writing bb mark in block %d\n",
			         block); 
		}
	}
	if (!ret)
		mtd->ecc_stats.badblocks++;
	return ret;
}


static uint8_t scan_ff_pattern[] = { 0xff, 0xff };
static struct nand_bbt_descr mlc_memorybased = {
	.options = 0,
	.offs = 0,
	.len = 2,
	.pattern = scan_ff_pattern
};
    /* loosely derived from nand_default_bbt()
     * We need to our own version of nand_scan_bbt() etc
     * because the standard versions assume the bad block info
     * is in the OOB of a block's first page.
     */
static int MLC_bbt(struct mtd_info * mtd )
{
	struct nand_chip *this = mtd->priv;
	int len;
	int i, numblocks;
	int startblock;
	loff_t from;

	/* For now, always build the bad block table in ram */
	this->bbt_td = NULL;
	this->bbt_md = NULL;
	if (!this->badblock_pattern) {
		this->badblock_pattern = &mlc_memorybased;
	}
        /* Allocate memory (2bit per block) 
         * and clear the memory bad block table */
	len = mtd->size >> (this->bbt_erase_shift + 2);
	this->bbt = kzalloc(len, GFP_KERNEL);
	if (!this->bbt) {
	        dev_err(&nand.pdev->dev, "MLC_bbt: Out of memory\n");
	        return -ENOMEM;
	}

	/* Note that numblocks is 2 * (real numblocks) here, see i+=2
	 * below as it makes shifting and masking less painful */
	numblocks  = mtd->size >> (this->bbt_erase_shift - 1);
	startblock = 0;
	from = 0;
	for (i = startblock; i < numblocks;) {
        	int ret;
    
        	ret = MLC_BlockBad(mtd, from, 1);
        	if (ret) {  /* if block is bad */
	    		this->bbt[i >> 3] |= 0x03 << (i & 0x6);
        		dev_warn(&nand.pdev->dev, 
				"Bad eraseblock %d at 0x%012llx\n",
	        	         i >> 1, (unsigned long long)from);
		        	 mtd->ecc_stats.badblocks++;
       		}
		i    += 2;
		from += (1 << this->bbt_erase_shift);
	}
	return 0;
}


/** Leapfrog's replacement for nand_verify_buf()
 * lf1000_verify_MLC_page - Verify chip data against buffer
 * @mtd:	MTD device structure
 * @buf:	buffer containing the data to compare
 * @len:	number of bytes to compare
 */
/* TODO: FIXME: Change this to read ECC bytes from OOB, check for errors,
 *              and correct them if possible.  Report a verification error
 *              only if there are uncorrectable errors or if the corrected 
 *		stuff doesn't match the contents of buf.
 * Alternatively count the number of flipped bits in the values that are read
 * back from NAND.  If only a few (< 4?), say it's ok -- assuming the ECC can
 * locate and fix them.
 */
static int lf1000_verify_MLC_page(struct mtd_info *mtd, 
                                  const uint8_t *buf, 
                                  int len)
{
	int rem = (3 & (unsigned int)buf);
	int olen = len; /* original len; needed just for error reporting */
	int numErrors = 0;
	struct nand_chip *chip = mtd->priv;
	tpIO A = chip->IO_ADDR_R;
	const uint8_t *b;

	uint8_t  val8;
	uint32_t val32;

	b = buf;
	if (rem > 0) {
	        for (; rem < 4; ++rem, --len, ++b) {
		        if (*b != (val8 = readb(A))) {
		                if (0 == numErrors++) {
                			dev_info(&nand.pdev->dev,
						 "lf1000_verify_MLC_page;"
		                                 " verify failed @%x:"
						 " %02x v. %02x\n", 
			                          olen - len, *b, val8);
                		}
		        }
	        }
	}
	if (0 == (3 & (unsigned int)b)) { /* now the pointer is 32-bit aligned*/
	        const uint32_t * p = (const uint32_t *)b;

	        for ( ; len >= 4; ++p, len -= 4) {
		        if (*p != (val32 = readl(A))) {
		                if (0 == numErrors++) {
		                	dev_info(&nand.pdev->dev,
						 "lf1000_verify_MLC_page;"
		                                 " verify failed @%x:"
						 " %08x v. %08x\n", 
			                         olen - len, *p, val32);	
                		}
			}
        	}
	        b = (const uint8_t *)p;
	}
	if (len > 0) {
	        for ( ; len > 0; ++b, --len) {
		        if (*b != (val8 = readb(A))) {
		                if (0 == numErrors++) {
                			dev_info(&nand.pdev->dev,
						 "lf1000_verify_MLC_page;"
		                                 " verify failed @%x:"
						 " %02x v. %02x\n", 
			                         olen - len, *b, val8);
                		}
		        }
		}
	}
	if (numErrors > 0)
		return -EFAULT;
	return 0;
}

static void lf1000_init_for_MLC_nand(struct mtd_info *mtd, 
                                     struct nand_chip *chip)
{
        /* For now we use 4-bit ECC on every MLC device
         * and force NO_SUBPAGE_WRITEs and subpagesize == writesize */
	lf1000_prepare_for_4BitEcc( chip );

        /* We use special bad block detector and marker functions
	 * for this chip in order to be straightforward about
         * checking the first byte of the spare area of the last
         * page of the block.
	 *
         * TODO: FIXME: Probably ought to check the mfr/chip IDs and use
         *              that info to determine the bad block functions
         * to use and the ecc.layout.
         */
	chip->block_bad     = MLC_BlockBad;
	chip->block_markbad = MLC_BlockMarkBad;
	chip->scan_bbt      = MLC_bbt;
	chip->verify_buf    = lf1000_verify_MLC_page;
	dev_info(&nand.pdev->dev, "Using 4-bit ECC\n");

	chip->options    |= NAND_NO_SUBPAGE_WRITE;
	mtd->subpage_sft  = 0;
	chip->subpagesize = mtd->writesize >> mtd->subpage_sft;
	if (mtd->writesize == 4096) {
	        chip->ecc.layout = &nand_oob_128_BCH;
	}
}



static void lf1000_set_hw_syndrome_ecc(struct nand_chip *chip)
{
	chip->ecc.read_page  = lf1000_nand_read_page_BCH;
	chip->ecc.write_page = lf1000_nand_write_page_BCH;
	chip->ecc.read_subpage = lf1000_nand_read_subpage_BCH;
	chip->ecc.read_oob   = nand_read_oob_std;
	chip->ecc.write_oob  = nand_write_oob_std;
	chip->ecc.calculate  = lf1000_nand_calculate_BCH;
	chip->ecc.correct    = lf1000_nand_correct_BCH;
	chip->ecc.hwctl      = lf1000_nand_enable_hwecc_BCH;
	chip->ecc.size       = 512;
	chip->ecc.bytes      = 7;
}


