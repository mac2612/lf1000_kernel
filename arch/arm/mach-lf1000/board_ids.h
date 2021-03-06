/*
 * arch/arm/mach-lf1000/board_ids.h
 *
 * LF1000 board definitions used solely by gpio.c
 * Please use/expand 'include/mach/boards.h' etermine board qualities.
 *
 * Scott Esters <sesters@leapfrog.com>
 *
 * This program is free software; you can redistributte it and/or modify
 * it under the terms of the GNU Gneral Public License as published by
 * the Free Software Foundation
 */

#ifndef LF1000_BOARD_IDS_H
#define LF1000_BOARD_IDS_H

/* This is a list of board types that can be detected at runtime to deal with
 * hardware quirks. See gpio_get_board_config() for more information. */


/* LF1000 Development boards and original Didj Form Factor (alpha) board. */
#define LF1000_BOARD_DEV		0x00

/* Original Didj 08 (Legacy Rev A) */
#define LF1000_BOARD_DIDJ		0x03

/* Didj 09, 2GB MLC Flash, 64KB Boot Flash */
#define LF1000_BOARD_DIDJ_09		0x04

/* Acorn, 8GB SLC Flash, 64MB SDRAM, 512KB Boot Flash, TV Out, RTC SuperCap */
#define LF1000_BOARD_ACORN		0x05


/*
 * Emerald / Leapster3 Boards
 */

/* Leapster Explorer POP, 8GB SLC Flash, 64MB SDRAM,
 * 512KB Boot Flash, TV Out, No RTC SuperCap */
#define LF1000_BOARD_EMERALD_POP	0x01

/* Leapster Explorer, 8GB SLC Flash, 64MB SDRAM,
 * 512KB Boot Flash, No TV Out, No RTC SuperCap */
#define LF1000_BOARD_EMERALD_NOTV_NOCAP	0x02

/* Leapster Explorer, 8GB SLC Flash, 64MB SDRAM,
 * 512KB Boot Flash, TV Out, No RTC SuperCap */
#define LF1000_BOARD_EMERALD_TV_NOCAP	0x06

/* Leapster Explorer, 8GB SLC Flash, 64MB SDRAM,
 * 512KB Boot Flash, No TV Out, RTC SuperCap */
#define LF1000_BOARD_EMERALD_NOTV_CAP	0x07

/* Leapster Explorer 8GB SLC Flash, 64MB Samsung
 * SDRAM, 512KB Boot Flash, No TV Out, No RTC SuperCap */
#define LF1000_BOARD_EMERALD_SAMSUNG	0x0A


/*
 * K2 Boards
 */

/* K2 Base, 8GB SLC Flash, 64MB SDRAM, 512KB Boot Flash */
#define	LF1000_BOARD_K2			0x10


/*
 * Madrid Boards
 */

/* Madrid, 5" LCD 2GB MLC Flash + SD Controller, 64MB SDRAM,
 *512KB Boot Flash, No TV Out, No RTC SuperCap, Accelerometer,
 * Vibration motor, USB Camera SDIO WIFI  */
#define LF1000_BOARD_MADRID		0x0B

/* Madrid, 5" LCD 2GB MLC Flash + SD Controller, 64MB SDRAM,
 *512KB Boot Flash, TV Out, No RTC SuperCap, Accelerometer,
 * Vibration motor, USB Camera SDIO WIFI  */
#define LF1000_BOARD_MADRID_POP		0x0C

/* Madrid, 5" LCD 2GB MLC Flash + SD Controller, 64MB SDRAM,
 *512KB Boot Flash, No TV Out, No RTC SuperCap, Accelerometer,
 * Vibration motor, USB Camera SDIO WIFI, LFP100  */
#define LF1000_BOARD_MADRID_LFP100	0x0D

#endif /* LF1000_BOARD_IDS_H */
