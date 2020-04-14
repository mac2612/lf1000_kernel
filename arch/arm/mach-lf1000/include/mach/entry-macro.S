/*
 * arch/arm/mach-lf1000/include/mach/entry-macro.S
 *
 * Low-level IRQ helper macros for LF1000 platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <mach/hardware.h>
#include <asm/hardware/vic.h>
#include <mach/ic.h>

		.macro	disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		ldr	\base, =IO_ADDRESS(LF1000_IC_BASE)
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

		.macro	get_irqnr_and_base, irqnr, irqpend, base, tmp

		mov	\irqnr, #0x0
		ldr	\irqpend, [\base, #INTPENDL]	@ get masked status
		cmp	\irqpend, #0x0
		bne	1001f				@ low irq pending, calc irq number

		ldr	\irqpend, [\base, #INTPENDH]	@ check if high irq pending
		cmp	\irqpend, #0x0
		beq	1003f				@ oops, there is no irq pending.
		movne	\irqnr, #32

1001:		/* get irq number */
		and	\tmp, \irqpend, #0x1
		cmp	\tmp, #0x1
		addne	\irqnr, \irqnr, #1
		movne	\irqpend, \irqpend, LSR #1
		bne	1001b

1003:
		cmp 	\irqpend, #0x0
		.endm


