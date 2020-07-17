/*
 *  mach-lf1000/include/mach/memory.h
 *
 *  Copyright (C) 2003 ARM Limited
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
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/*
 * Physical DRAM offset.
 */

/* NAND / UART BOOT */
#define PHYS_OFFSET_SHADOW	UL(0x00000000)
/* NOR BOOT */
#define PHYS_OFFSET_NO_SHADOW	UL(0x80000000)

/* TODO: Figure this out dynamically or via bootloader arg. */
#ifdef CONFIG_LF1000_SHADOWRAM
#define PHYS_OFFSET	PHYS_OFFSET_SHADOW
#else
#define PHYS_OFFSET     PHYS_OFFSET_NO_SHADOW
#endif

/*
 * Virtual view <-> DMA view memory address translations
 * virt_to_bus: Used to translate the virtual address to an
 *              address suitable to be passed to set_dma_addr
 * bus_to_virt: Used to convert an address for DMA operations
 *              to an address that the kernel can use.
 */
#define __phys_to_bus(x)        ((x) + PAGE_OFFSET)
#define __bus_to_phys(x)        ((x) - PAGE_OFFSET)

#define __virt_to_bus(v)        __phys_to_bus(__virt_to_phys(v))
#define __bus_to_virt(b)        __phys_to_virt(__bus_to_phys(b))
#define __pfn_to_bus(p)         __phys_to_bus(__pfn_to_phys(p))
#define __bus_to_pfn(b)         __phys_to_pfn(__bus_to_phys(b))

#endif
