/*
 *  drivers/mtd/nand/lf1000_ecc.c
 *
 *  Andrey Yurovsky <andrey@cozybit.com>
 *  Robert Dowling <rdowling@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <mach/platform.h>
#include <mach/common.h>
#include <mach/nand.h>
#include <asm/io.h>
#include <asm/sizes.h>

#if defined CPU_LF1000 && defined CONFIG_MTD_NAND_LF1000_HWECC

/* control registers */
#define NAND_BASE	IO_ADDRESS(LF1000_MCU_S_BASE)

/******************************************************************************
 * Lifted from Magic Eyes ./vtk/test/NAND_Flash/mes_nand.c
 *
//  Copyright (C) 2007 MagicEyes Digital Co., All Rights Reserved
//  MagicEyes Digital Co. Proprietary & Confidential
//
//	MAGICEYES INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
//  AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
//  FOR A PARTICULAR PURPOSE.
 *
 * MES_NAND_GetErrorLocation( int *pLocation )
 ******************************************************************************/

extern const short BCH_AlphaToTable[8192];
extern const short BCH_IndexOfTable[8192];
extern const unsigned int BCH_TGRTable[52][2];


//------------------------------------------------------------------------------
// Derived from Magic Eyes's
// int MES_NAND_GetErrorLocation( int *pLocation )
//
int lf1000_GetErrorLocation (int *pLocation, u16 s1, u16 s3, u16 s5, u16 s7)
{
	register int i, j, elp_sum ;
	int count;
	int r;			// Iteration steps
	int Delta; 		// Discrepancy value
	int elp[8+1][8+2]; 	// Error locator polynomial (ELP)
	int L[8+1];		// Degree of ELP
	int B[8+1][8+2];	// Scratch polynomial
	//	int root[8+1];	// Roots of ELP
	int reg[8+1];		// Register state
	int	s[8];

	s[0] = s1;
	s[2] = s3;
	s[4] = s5;
	s[6] = s7;

	// Even syndrome = (Odd syndrome) ** 2
	for( i=1,j=0 ; i<8 ; i+=2, j++ )
	{
		if( s[j] == 0 )		s[i] = 0;
		else				s[i] = BCH_AlphaToTable[(2 * BCH_IndexOfTable[s[j]]) % 8191];
	}

	// Initialization of elp, B and register
	for( i=0 ; i<=8 ; i++ )
	{
		L[i] = 0 ;
		for( j=0 ; j<=8 ; j++ )
		{
			elp[i][j] = 0 ;
			B[i][j] = 0 ;
		}
	}

	for( i=0 ; i<=4 ; i++ )
	{
		//		root[i] = 0 ;
		reg[i] = 0 ;
	}

	elp[1][0] = 1 ;
	elp[1][1] = s[0] ;

	L[1] = 1 ;
	if( s[0] != 0 )
		B[1][0] = BCH_AlphaToTable[(8191 - BCH_IndexOfTable[s[0]]) % 8191];
	else
		B[1][0] = 0;

	for( r=3 ; r<=8-1 ; r=r+2 )
	{
		// Compute discrepancy
		Delta = s[r-1] ;
		for( i=1 ; i<=L[r-2] ; i++ )
		{
			if( (s[r-i-1] != 0) && (elp[r-2][i] != 0) )
				Delta ^= BCH_AlphaToTable[(BCH_IndexOfTable[s[r-i-1]] + BCH_IndexOfTable[elp[r-2][i]]) % 8191];
		}

		if( Delta == 0 )
		{
			L[r] = L[r-2] ;
			for( i=0 ; i<=L[r-2] ; i++ )
			{
				elp[r][i] = elp[r-2][i];
				B[r][i+2] = B[r-2][i] ;
			}
		}
		else
		{
			// Form new error locator polynomial
			for( i=0 ; i<=L[r-2] ; i++ )
			{
				elp[r][i] = elp[r-2][i] ;
			}

			for( i=0 ; i<=L[r-2] ; i++ )
			{
				if( B[r-2][i] != 0 )
					elp[r][i+2] ^= BCH_AlphaToTable[(BCH_IndexOfTable[Delta] + BCH_IndexOfTable[B[r-2][i]]) % 8191];
			}

			// Form new scratch polynomial and register length
			if( 2 * L[r-2] >= r )
			{
				L[r] = L[r-2] ;
				for( i=0 ; i<=L[r-2] ; i++ )
				{
					B[r][i+2] = B[r-2][i];
				}
			}
			else
			{
				L[r] = r - L[r-2];
				for( i=0 ; i<=L[r-2] ; i++ )
				{
					if( elp[r-2][i] != 0 )
						B[r][i] = BCH_AlphaToTable[(BCH_IndexOfTable[elp[r-2][i]] + 8191 - BCH_IndexOfTable[Delta]) % 8191];
					else
						B[r][i] = 0;
				}
			}
		}
	}

	if( L[8-1] > 4 )
	{
		//return L[8-1];
		return -1;
	}
	else
	{
		// Chien's search to find roots of the error location polynomial
		// Ref: L&C pp.216, Fig.6.1
		for( i=1 ; i<=L[8-1] ; i++ )
			reg[i] = elp[8-1][i];

		count = 0;
		for( i=1 ; i<=8191 ; i++ )
		{
			elp_sum = 1;
			for( j=1 ; j<=L[8-1] ; j++ )
			{
				if( reg[j] != 0 )
				{
					reg[j] = BCH_AlphaToTable[(BCH_IndexOfTable[reg[j]] + j) % 8191] ;
					elp_sum ^= reg[j] ;
				}
			}

			if( !elp_sum )		// store root and error location number indices
			{
				//				root[count] = i;

				// Convert error location from systematic form to storage form
				pLocation[count] = 8191 - i;

				if (pLocation[count] >= 52)
				{
					// Data Bit Error
					pLocation[count] = pLocation[count] - 52;
					pLocation[count] = 4095 - pLocation[count];
				}
				else
				{
					// ECC Error
					pLocation[count] = pLocation[count] + 4096;
				}

				if( pLocation[count] < 0 )	return -1;
				/*
				  if( i <= 8191 - 52 )	pLocation[count] = 4095 - (8191 - 52 - i);
				  else					pLocation[count] = 4095 - (8191 + 4096 - i);
				*/

				printk (KERN_DEBUG "lf1000_GetErrorLocation: count=%d, location=%d, count++\n", count, pLocation[count]);
				count++;
			}
		}

		if( count == L[8-1] )	// Number of roots = degree of elp hence <= 4 errors
		{
			return L[8-1];
		}
		else	// Number of roots != degree of ELP => >4 errors and cannot solve
		{
			return -1;
		}
	}
}

    /* returns 0 if no errors
     *      N >0 if N errors were corrected
     *        <0 if uncorrectable errors
     */
int TryToCorrectBCH_Errors( u8 * pData ) 
{
    u32 x, s7, s5, s3, s1;
    int errorLocations[4];
    int numErrors;
    int eccsize  = 512;

        /* Pull out the 4 syndrome words */
    x  = readl(NAND_BASE+NFSYNDRONE75);
    s7 = (x >> SYNDROM7) & 0x1fff;
    s5 = (x >> SYNDROM5) & 0x1fff;

    x  = readl(NAND_BASE+NFSYNDRONE31);
    s3 = (x >> SYNDROM3) & 0x1fff;
    s1 = (x >> SYNDROM1) & 0x1fff;

    numErrors = lf1000_GetErrorLocation( &errorLocations[0], s1, s3, s5, s7);
	    /* If there is one or more correctable errors 		 
         * ('numErrors' is the number of correctable errors)
	     */
    if (numErrors > 0)
    {
        int errorOffset;
        int errorMask;
        int j;

        printk( KERN_INFO "Found %d correctable ECC errors\n", numErrors);
        for (j = 0; j < numErrors; ++j)
        {
            errorOffset = errorLocations[j] >> 3;
            errorMask   = 1 << (errorLocations[j] & 7);
		        /* We see errorOffset >= 512 when an ECC bit is in error
		    	 * Don't bother correcting ECC bit errors, because the
				 * bit positions might be beyond the end of the buffer.
				 */
			if (errorOffset < eccsize)
			{
                printk(KERN_INFO "Flip bit %d; offset %d; mask %02x\n",
                        errorLocations[j], errorOffset, errorMask);
               	*(pData + errorOffset) ^= errorMask;
			}
        }
    }
    return numErrors;
}

#endif /* CPU_LF1000 && CONFIG_MTD_NAND_LF1000_HWECC */
