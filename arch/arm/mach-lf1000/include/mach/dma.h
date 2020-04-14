/*
 * LF1000 DMA Engine support
 * adapted from Corelogic CLX7000 DMA Engine support
 *
 * Copyright (C) 2009 Corelogic, Inc. All rights reserved.
 *
 * Author:
 *   Scott Esters <sesters@leapfrog.com>
 *
 * Description:
 *   DMA engine driver for LF1000 DMA controller
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#ifndef __ASM_MACH_DMA_H
#define __ASM_MACH_DMA_H

/* DMA operation mode (for circular DMA) */
enum dma_mem_io {
	DMA_MEM_IO		= 0x00,	// Data I/O with DMA API
	DMA_MEM_MAPPED		= 0x01, // DMA Buffer is mapped and direct access
};

/* DMA Channel  */
enum dma_priority {
	DMA_PRIORITY_LV0	= 0x01,
	DMA_PRIORITY_LV1	= 0x02,
	DMA_PRIORITY_LV2	= 0x04,
	DMA_PRIORITY_LV3	= 0x08,
	DMA_PRIORITY_ANY	= 0x0F,
};

/* interrupt type */
enum dma_interrupt_type {
	DMA_INT_DISABLE		= 0,
	DMA_INT_BLOCK		= 1,
	DMA_INT_EVERY_BLOCK	= 2,
	DMA_INT_LAST_BLOCK	= 3,
};

enum dma_transfer_type {
	DMA_MEM_TO_MEM		= 0,
	DMA_MEM_TO_IO		= 1,
	DMA_IO_TO_MEM		= 2,
};

enum dma_request_id {
	DMA_PERI_UART0TX	= 0,
	DMA_PERI_UART0RX	= 1,
	DMA_PERI_UART1TX	= 2,
	DMA_PERI_UART1RX	= 3,
	DMA_PERI_UART2TX	= 4,
	DMA_PERI_UART2RX	= 5,
	DMA_PERI_UART3TX	= 6,
	DMA_PERI_UART3RX	= 7,

	DMA_PERI_USBEP1		= 12,
	DMA_PERI_USBEP2		= 13,

	DMA_PERI_SD0RW		= 16,

	DMA_PERI_SPI0TX		= 18,
	DMA_PERI_SPI0RX		= 19,
	DMA_PERI_SPI1TX		= 20,
	DMA_PERI_SPI1RX		= 21,
	DMA_PERI_SPI2TX		= 22,
	DMA_PERI_SPI2RX		= 23,
	DMA_PERI_PCMOUT		= 24,

	DMA_PERI_PCMIN		= 26,

	DMA_PERI_SD1RW		= 30,
};

struct dma_control {
	enum dma_transfer_type	transfer;
	enum dma_interrupt_type	interrupt;
	enum dma_request_id	request_id;
	unsigned int		io_addr_inc;	// 1: inc, 0:not inc
	unsigned int		src_width;	// 1, 2, 4 bytes
	unsigned int		dest_width;	// 1, 2, 4 bytes
};

unsigned int dma_get_base_address(int channel);
int dma_request (char *name, enum dma_priority priority, irq_handler_t handler,
	void *objdata, unsigned int *ch);
int dma_release(unsigned int ch);
int dma_start(unsigned int ch);
int dma_stop(unsigned int ch);
int dma_reset(unsigned int ch);
int dma_is_active(unsigned int ch);
int dma_is_enabled(unsigned int ch);
unsigned int dma_get_write_addr(unsigned int ch);
unsigned int dma_get_read_addr(unsigned int ch);

int dma_transfer_init(unsigned int ch, enum dma_mem_io mem_mode);
int dma_transfer_insert(unsigned int ch, unsigned int src, unsigned int dest,
	unsigned int size, struct dma_control *ctrl);

int dma_sg_write(unsigned int ch, struct scatterlist *sg, unsigned int dest,
	int count, struct dma_control *ctrl);
int dma_sg_read(unsigned int ch, unsigned int src, struct scatterlist *sg,
	int count, struct dma_control *ctrl);

int dma_circ_write(unsigned int ch, unsigned int *src_list, unsigned int dest,
	int count, unsigned int block_size, struct dma_control *ctrl);
int dma_circ_read(unsigned int ch, unsigned int src, unsigned int *dest_list,
	int count, unsigned int block_size, struct dma_control *ctrl);

#endif

