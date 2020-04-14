/* arch/arm/plat-mes/irq.c - vectored interrupt controller support for
 * MagicEyes/Leapfrog SoCs.
 *
 * Copyright (c) 2007-2010 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <ayurovsky@leapfrog.com>
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

#include <linux/init.h>
#include <linux/list.h>

#include <asm/io.h>
#include <asm/mach/irq.h>
#include <asm/hardware/vic.h>
#include <mach/platform.h>
#include <mach/ic.h>

/* FIXME: IRQ Base */
#define MES_INT_BASE	0xC0000800
#define INTMASKL	0x10
#define INTMASKH	0x14
#define INTPENDL	0x20
#define INTPENDH	0x24

/* FIXME: DMA Mode and interrupt bit */
#define MES_DMA_BASE	0xC0000000
#define DMAMODE		0x0C
#define DMA_INTENB	(1<<18)
#define DMA_INTPEND	(1<<17)

static __inline void mes_irq_clear(unsigned int irq)
{
	if (irq < 32)
		writel((1 << irq), IO_ADDRESS(MES_INT_BASE + INTPENDL));
	else
		writel((1 << (irq - 32)), IO_ADDRESS(MES_INT_BASE + INTPENDH));
}

static void mes_irq_mask(unsigned int irq)
{
	unsigned long val;
	unsigned int shift =irq & 63;

	mes_irq_clear(irq);

	if (irq < 32) {
		val = readl(IO_ADDRESS(MES_INT_BASE + INTMASKL));
		val |= (1 << shift);
		writel(val, IO_ADDRESS(MES_INT_BASE + INTMASKL));
	} else {
		shift -= 32;
		val = readl(IO_ADDRESS(MES_INT_BASE + INTMASKH));
		val |= (1 << shift);
		writel(val, IO_ADDRESS(MES_INT_BASE + INTMASKH));
	}
}

static void mes_irq_unmask(unsigned int irq)
{
	unsigned long val;
	unsigned int shift = irq & 63;

	mes_irq_clear(irq);

	if (irq < 32) {
		val = readl(IO_ADDRESS(MES_INT_BASE + INTMASKL));
		val &= ~(1 << shift);
		writel(val, IO_ADDRESS(MES_INT_BASE + INTMASKL));
	} else {
		shift -= 32;
		val = readl(IO_ADDRESS(MES_INT_BASE + INTMASKH));
		val &= ~(1 << shift);
		writel(val, IO_ADDRESS(MES_INT_BASE + INTMASKH));
	}
}

static struct irq_chip mes_irq_chip = {
	.name		= "MES IRQ",
	.mask		= mes_irq_mask,
	.mask_ack	= mes_irq_mask,
	.unmask		= mes_irq_unmask,
};

static void mes_dma_irq_mask(unsigned int irq)
{
	unsigned int dma = irq_to_dma(irq);
	unsigned long val;

	mes_irq_clear(IRQ_DMA);

	val = readl(IO_ADDRESS(MES_DMA_BASE + DMAMODE + (0x80 * dma)));
	val &= ~(DMA_INTENB);	/* mask interrupt */
	val |= DMA_INTPEND;	/* clear pending interrupt */
	writel(val, IO_ADDRESS(MES_DMA_BASE + DMAMODE + (0x80 * dma)));
}

static void mes_dma_irq_unmask(unsigned int irq)
{
	unsigned int dma = irq_to_dma(irq);
	unsigned long val;

	mes_irq_clear(IRQ_DMA);
	val = readl(IO_ADDRESS(MES_DMA_BASE + DMAMODE + (0x80 * dma)));
	val |= DMA_INTENB;	/* enable interrupt */
	val |= DMA_INTPEND;	/* clear interrupt */
	writel(val, IO_ADDRESS(MES_DMA_BASE + DMAMODE + (0x80 * dma)));
}

static void mes_dma_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	void __iomem *mode = (void __iomem *)IO_ADDRESS(MES_DMA_BASE + DMAMODE);
	unsigned long val;
	int i;

	for (i = 0; i < NR_DMA_IRQS; ++i) {
		val = readl(mode + (0x80 * i));
		if (val & DMA_INTPEND) {		   /* int pending ? */
			generic_handle_irq(dma_to_irq(i)); /* software int  */
		}
	}
}

static struct irq_chip mes_dma_irq_chip = {
	.name		= "MES DMA IRQ",
	.mask		= mes_dma_irq_mask,
	.mask_ack	= mes_dma_irq_mask,
	.unmask		= mes_dma_irq_unmask,
};

void __init mes_irq_init(void __iomem *base)
{
	int i;

	writel(0, base + INTMODEL);
	writel(0, base + INTMODEH);
	writel(~0, base + INTMASKL);
	writel(~0, base + INTMASKH);
	writel(~0, base + INTPENDL);
	writel(~0, base + INTPENDH);
	writel(0, base + PRIORDER);

	for (i = 0; i < NR_IRQS; i++) {
		set_irq_chip(i, &mes_irq_chip);
		set_irq_chip_data(i, base);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}

	/* set DMA IRQ sources */
	set_irq_chained_handler(IRQ_DMA, mes_dma_irq_handler);
	for (i = dma_to_irq(0); i < dma_to_irq(NR_DMA_IRQS); ++i) {
		set_irq_chip(i, &mes_dma_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	} 
}
