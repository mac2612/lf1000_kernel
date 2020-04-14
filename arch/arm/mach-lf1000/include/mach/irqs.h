/*
 *  mach-lf1000/include/mach/irqs.h
 *
 *  Copyright (C) 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <mach/platform.h>

/* 
 *  IRQ interrupts definitions are the same the INT definitions
 *  held within platform.h
 */

#define	IRQ_PDISPLAY		0
#define	IRQ_SDIPLAY		1

#define	IRQ_DMA			3
#define IRQ_TIMER0		4
#define	IRQ_SYSCTRL		5

#define	IRQ_UART0		10
#define	IRQ_TIMER1		11
#define IRQ_SPI0		12
#define	IRQ_GPIO		13
#define	IRQ_SDMMC0		14
#define IRQ_TIMER2		15

#define IRQ_UDC			20
#define IRQ_TIMER3		21

#define IRQ_AUDIOIF		24
#define IRQ_ADC			25
#define IRQ_MCUSTATIC		26
#define IRQ_GRP3D		27
#define IRQ_UHC			28

#define IRQ_RTC			31
#define IRQ_I2C0		32
#define IRQ_I2C1		33
#define IRQ_UART1		34
#define IRQ_UART2		35
#define IRQ_UART3		36

#define IRQ_SPI1		39
#define IRQ_SPI2		40
#define IRQ_CSC			41
#define IRQ_SDMMC1		42
#define IRQ_TIMER4		43

#define NR_HW_IRQS		64

#define NR_GPIO_IRQS		(32 * 4)
#define NR_DMA_IRQS		8
#define NR_ALIVE_IRQS		8
#define NR_IRQS			256

#define dma_to_irq(x)           (x + NR_HW_IRQS + NR_GPIO_IRQS + NR_ALIVE_IRQS)
#define irq_to_dma(x)           (x - NR_HW_IRQS - NR_GPIO_IRQS - NR_ALIVE_IRQS)

