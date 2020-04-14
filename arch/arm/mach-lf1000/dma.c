/*
 * Author:
 *   Scott Esters <sesters@leapfrog.com>
 *
 * Description:
 *   DMA engine driver for Magic Eyes LF1000 DMA controller
 *   Adapted from CoreLogic Spica DMA Controller
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/scatterlist.h>
#include <mach/dma.h>
#include <mach/irqs.h>
#include <mach/platform.h>

#define DRIVER_NAME	"lf1000-dma"

#ifdef LF1000_DMA_DEBUG
#define DBG(x...) printk(KERN_ALERT DRIVER_NAME ": " x)
#else
#define DBG(x...)
#endif

#define USE_IOREMAP

#define MAX_DMA_CHANNELS		8

#define DEFAULT_MAX_NODE		32

/* Register offset */
#define DMASRCADDR			0x00
#define DMADSTADDR			0x04
#define DMALENGTH			0x08
#define DMAREQID			0x0A
#define DMAMODE				0x0C
#define DMASRCADDR_WB			0x10
#define DMADSTADDR_WB			0x14
#define DMALENGTH_WB			0x18
#define DMAREQID_WB			0x1A
#define DMAMODE_WB			0x1C
#define DMACMDWAIT			0x20
#define DMACMDSTOP			0x24
#define DMACMDBUSY			0x28
#define DMACMDSPACE			0x2C

/* Operation Mode */
#define MODE_STOP			(1 << 20)
#define MODE_RUN			(1 << 19)
#define MODE_INTENB			(1 << 18)
#define MODE_INTPEND			(1 << 17)
#define MODE_BUSY			(1 << 16)
#define MODE_DSTNOTREQCHK		(1 << 13)
#define MODE_DSTNOTINC			(1 << 12)
#define MODE_DSTIOMODE			(1 << 10)
#define MODE_DSTIOSIZE_MASK		(3 << 8)
#define MODE_SRCNOTREQCHK		(1 << 5)
#define MODE_SRCNOTINC			(1 << 4)
#define MODE_SRCIOMODE			(1 << 2)
#define MODE_SRCIOSIZE_MASK		3

#define set_dst_io_width(size)	((size >> 1) << 8)
#define set_src_io_width(size)	(size >> 1)
#define get_dst_io_width(mode)	(1 << ((mode & MODE_DSTIOSIZE_MASK) >> 8))
#define get_src_io_width(mode)	(1 << (mode & MODE_SRCIOSIZE_MASK))

enum dma_state {
	DMAC_STOP 	= 0,
	DMAC_RUN	= 1,
};

struct item_node {
	struct list_head	link;
	unsigned int 		src_addr;	// source address
	unsigned int 		dst_addr;	// destination address
	unsigned short		size;		// size of transfer - 1
	unsigned short		req_id;
	unsigned int 		op_mode;
	unsigned int		int_flag;
};

struct dmachannel {
	char			*device_id;
	void __iomem		*reg;		// register of dma channel
	enum dma_mem_io		mode;
	enum dma_priority	priority;
	enum dma_state		state;
	irq_handler_t		handler;
	void			*objdata;
	spinlock_t 		lock;
	struct list_head	active_nodes;	// active node list
	struct list_head	free_nodes;	// free node list
	struct item_node	*cur_node;	// indicates item_node of
						// current dma transfer
	int			num_node;	// transfer count
};

struct dma_info {
	void __iomem		*reg;
	unsigned int		num_active_channel;
	struct dmachannel	dmach[MAX_DMA_CHANNELS];
};

static struct dma_info	*dmadev = NULL;

/*******************************************************************************
  * Function Name       : dma_irq_handler
  * Input Parameter(s)  : int irq
  			  void *dev_id
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
static irqreturn_t dma_irq_handler(int irq, void *dev_id)
{
	unsigned int dma;
	irqreturn_t ret;
	struct item_node *node = NULL;
	struct dmachannel *dmach;

	if (!dmadev)
		return IRQ_NONE;

	dma = irq_to_dma(irq);

	if (dma >= MAX_DMA_CHANNELS)
		return IRQ_NONE;

	dmach = &dmadev->dmach[dma];
	if (!dmach->device_id)
		return IRQ_NONE;

#if 0
	// process interrupt
	if (dmach->handler && dmach->cur_node->int_flag)
		ret = dmach->handler((int)dmach, dmach->objdata);

	// check DMA is stopped ( dma_stop() )
	if (dmach->state == DMAC_STOP) {
		return IRQ_HANDLED;
	}

	// transfer next node
	if (list_is_last((struct list_head*)dmach->cur_node, &dmach->active_nodes)) {
		if (dmach->mode == DMA_MEM_IO) {
			// transfer complete.
			dmach->state = DMAC_STOP;
			return IRQ_HANDLED;
		} else {
			// ring buffer..
			node = (struct item_node*)list_first_entry(&dmach->active_nodes, struct item_node, link);
		}
	} else {
		// get next node
		node = (struct item_node*)dmach->cur_node->link.next;
	}

	dmach->cur_node = node;

	writel(node->src_addr, dmach->reg + DMASRCADDR);
	writel(node->dst_addr, dmach->reg + DMADSTADDR);
	writew(node->size, dmach->reg + DMALENGTH);
	writew(node->req_id, dmach->reg + DMAREQID);
	node->op_mode |= MODE_RUN;
	writel(node->op_mode, dmach->reg + DMAMODE);
#else
	// check DMA is stopped ( dma_stop() )
	if (dmach->state == DMAC_STOP) {
		goto irq_exit;
	}

	// transfer next node
	if (list_is_last((struct list_head*)dmach->cur_node,
		&dmach->active_nodes)) {
		if (dmach->mode == DMA_MEM_IO) {
			// transfer complete.
			dmach->state = DMAC_STOP;
			goto irq_exit;
		} else {
			// ring buffer..
			node = (struct item_node*)
				list_first_entry(&dmach->active_nodes,
				struct item_node, link);
		}
	} else {
		// get next node
		node = (struct item_node*)dmach->cur_node->link.next;
	}

	dmach->cur_node = node;

	writel(node->src_addr, dmach->reg + DMASRCADDR);
	writel(node->dst_addr, dmach->reg + DMADSTADDR);
	writew(node->size, dmach->reg + DMALENGTH);
	writew(node->req_id, dmach->reg + DMAREQID);
	node->op_mode |= MODE_RUN;
	writel(node->op_mode, dmach->reg + DMAMODE);

irq_exit:
	// process interrupt
	if (dmach->handler && dmach->cur_node->int_flag)
		ret = dmach->handler((int)dmach, dmach->objdata);

#endif
	return IRQ_HANDLED;
}

/*******************************************************************************
  * Function Name       : dma_get_free_node
  * Input Parameter(s)  : struct dmachannel *dmach
  * Output Parameter(s) : 
  * Return Value        : node address. NULL is error
  * Description         : 
  *****************************************************************************/
static struct item_node *dma_get_free_node(struct dmachannel *dmach)
{
	struct item_node *node = NULL;

	if(list_empty(&dmach->free_nodes)) {
		node = (struct item_node*)kzalloc(sizeof(struct item_node),
			GFP_KERNEL);
		DBG("alloc new item_node - %p\n", node);
	} else {
		node = list_first_entry(&dmach->free_nodes,
			struct item_node, link);
		list_del((struct list_head*)node); // remove node from free list
	}
	
	return node;
}

unsigned int dma_get_base_address(int channel)
{
  return ((LF1000_DMA_BASE) + (0x80 * channel));
}

/*******************************************************************************
  * Function Name       : dma_request
  * Input Parameter(s)  : char *name
			  dma_priority priority
  			  irq_handler_t handler
  			  void *objdata
  * Output Parameter(s) : unsigned int *ch
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_request (char *name, enum dma_priority priority, irq_handler_t handler, void *objdata, unsigned int *ch)
{
	unsigned long flags;
	struct item_node *node;
	int i, cnt;

	if (!dmadev)
		return (-ENODEV);

	local_irq_save(flags);

	for(i = MAX_DMA_CHANNELS - 1; i >= 0 ; i--) {
	//for(i = 0; i < MAX_DMA_CHANNELS ; i++) {
		if(!dmadev->dmach[i].device_id &&
			(dmadev->dmach[i].priority & priority)) {

			dmadev->dmach[i].device_id = kzalloc(strlen(name) + 1,
				GFP_KERNEL);
			if(!dmadev->dmach[i].device_id) {
				local_irq_restore(flags);
				return (-ENOMEM);
			}

			strcpy(dmadev->dmach[i].device_id, (const char*)name);
			dmadev->dmach[i].handler = handler;
			dmadev->dmach[i].objdata = objdata;
			dmadev->dmach[i].mode = DMA_MEM_IO;	// default mode
			dmadev->dmach[i].state = DMAC_STOP;

			INIT_LIST_HEAD(&dmadev->dmach[i].active_nodes);
			INIT_LIST_HEAD(&dmadev->dmach[i].free_nodes);

			for (cnt = 0; cnt < DEFAULT_MAX_NODE; cnt++) {
				node = kzalloc(sizeof(struct item_node),
					GFP_KERNEL);
				if (!node)
					goto error;
				list_add_tail((struct list_head*)node,
					&dmadev->dmach[i].free_nodes);
			}

			*ch = (unsigned int)&dmadev->dmach[i];

			local_irq_restore(flags);

			return 0;
		}
	}
	
	local_irq_restore(flags);

	return (-EBUSY);
error:
	while(!list_empty(&dmadev->dmach[i].free_nodes)) {
		node = (struct item_node*)
			list_first_entry(&dmadev->dmach[i].free_nodes,
			struct item_node, link);
		list_del(&node->link);
		kfree(node);
	}

	if(dmadev->dmach[i].device_id) {
		kfree(dmadev->dmach[i].device_id);
		dmadev->dmach[i].device_id = NULL;
	}

	local_irq_restore(flags);

	return (-ENOMEM);
}
EXPORT_SYMBOL(dma_request);

/*******************************************************************************
  * Function Name       : dma_release
  * Input Parameter(s)  : unsigned int ch
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_release(unsigned int ch)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	struct item_node *node = NULL;
	unsigned long flags;
	int i;

	if (!dmadev)
		return (-ENODEV);

	if (!dmach->device_id)
		return (-EINVAL);

	dma_stop(ch);
	dma_reset(ch);
	
	// check valid
	for(i = 0; i < MAX_DMA_CHANNELS; i++) {
		if (dmadev->dmach[i].device_id) {
			if (!strncmp(dmach->device_id,
				dmadev->dmach[i].device_id, 32))
				goto release;
		}
	}

	DBG("not found opened DMA channel %u", ch);

	return (-EINVAL);

release:
	local_irq_save(flags);

	kfree(dmach->device_id);
	while(!list_empty(&dmach->free_nodes)) {
		node = (struct item_node*)list_first_entry(&dmach->free_nodes,
			struct item_node, link);
		list_del(&node->link);
		kfree(node);
	}

	dmach->device_id = NULL;
	dmach->handler = NULL;
	dmach->objdata = NULL;

	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_release);

/*******************************************************************************
  * Function Name       : dma_start
  * Input Parameter(s)  : unsigned int ch
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_start(unsigned int ch)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	struct item_node *node;
	unsigned long flags;

	DBG("Entered %s - 0x%p\n", __func__, ch);

	if (list_empty(&dmach->active_nodes))
		return -1;

	if (dma_is_enabled(ch))
		return (-EBUSY);
	
	local_irq_save(flags);

	dmach->cur_node = node = list_first_entry(&dmach->active_nodes,
		struct item_node, link);
	
	writel(node->src_addr, dmach->reg + DMASRCADDR);
	writel(node->dst_addr, dmach->reg + DMADSTADDR);
	writew(node->size, dmach->reg + DMALENGTH);
	writew(node->req_id, dmach->reg + DMAREQID);
	node->op_mode |= MODE_RUN;
	writel(node->op_mode, dmach->reg + DMAMODE);
	dmach->state = DMAC_RUN;
	
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_start);

/*******************************************************************************
  * Function Name       : dma_stop
  * Input Parameter(s)  : unsigned int ch
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_stop(unsigned int ch)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
//	struct item_node *node;
	unsigned int regs;
	unsigned long flags;
	unsigned int loop;
	
	DBG("Entered %s - 0x%p\n", __func__, ch);

	if (!dmach->device_id)
		return (-EINVAL);

	local_irq_save(flags);

	// change state to STOP
	dmach->state = DMAC_STOP;
	
	regs = readl(dmach->reg + DMAMODE);
	//regs &= ~MODE_RUN;
	regs |= MODE_STOP;
	writel(regs, dmach->reg + DMAMODE);

	// check RUN bit to decide DMA channel is safety stopped.
	loop = 100;
	while (((regs = readl(dmach->reg + DMAMODE)) & MODE_RUN) &&
		(loop-- > 0)) {
		msleep(1);
	}

	// clear STOP bit
	writel(regs & ~MODE_STOP, dmach->reg + DMAMODE);

/*
	// move active_nodes to free_nodes
	while(!list_empty(&dmach->active_nodes)) {
		node = (struct item_node*)list_first_entry(&dmach->active_nodes, struct item_node, link);
		list_move_tail((struct list_head*)node, &dmach->free_nodes);
	}
*/
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_stop);

/*******************************************************************************
  * Function Name       : dma_reset
  * Input Parameter(s)  : unsigned int ch
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_reset(unsigned int ch)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	struct item_node *node;
	unsigned int regs;
	unsigned long flags;
	//unsigned int loop;
	
	DBG("Entered %s - 0x%p\n", __func__, ch);

	if (!dmach->device_id)
		return (-EINVAL);

	local_irq_save(flags);

	// change state to STOP
	dmach->state = DMAC_STOP;

	// forced stop
	regs = readl(dmach->reg + DMAMODE);
	regs &= ~MODE_RUN;
	//regs |= MODE_STOP;
	writel(regs, dmach->reg + DMAMODE);

/*
	// check RUN bit to decide DMA channel is safety stopped.
	loop = 100;
	while (((regs = readl(dmach->reg + DMAMODE)) & MODE_RUN) && (loop-- > 0)) {
		msleep(1);
	}

	// clear STOP bit
	writel(regs & ~MODE_STOP, dmach->reg + DMAMODE);
*/
	// move active_nodes to free_nodes
	while(!list_empty(&dmach->active_nodes)) {
		node = (struct item_node*)
			list_first_entry(&dmach->active_nodes,
			struct item_node, link);
		list_move_tail((struct list_head*)node, &dmach->free_nodes);
	}

	local_irq_restore(flags);

	return 0;

}
EXPORT_SYMBOL(dma_reset);

/*******************************************************************************
  * Function Name       : dma_is_active
  * Input Parameter(s)  : unsigned int ch
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 if inactive
  * Description         : 
  *****************************************************************************/
int dma_is_active(unsigned int ch)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;

	return (readl(dmach->reg + DMAMODE) & MODE_BUSY);
}
EXPORT_SYMBOL(dma_is_active);

/*******************************************************************************
  * Function Name       : dma_is_enabled
  * Input Parameter(s)  : unsigned int ch
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_is_enabled(unsigned int ch)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	unsigned int regs;

	regs = readl(dmach->reg + DMAMODE);
	if( (regs & MODE_RUN) || (dmach->state == DMAC_RUN))
		return 1;
	else 
		return 0;
}
EXPORT_SYMBOL(dma_is_enabled);

/*******************************************************************************
  * Function Name       : dma_get_write_addr
  * Input Parameter(s)  : unsigned int ch
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
unsigned int dma_get_write_addr(unsigned int ch)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;

	return readl(dmach->reg + DMASRCADDR);
}
EXPORT_SYMBOL(dma_get_write_addr);

/*******************************************************************************
  * Function Name       : dma_get_read_addr
  * Input Parameter(s)  : unsigned int ch
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
unsigned int dma_get_read_addr(unsigned int ch)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;

	return readl(dmach->reg + DMADSTADDR);
}
EXPORT_SYMBOL(dma_get_read_addr);

/*******************************************************************************
  * Function Name       : dma_transfer_init
  * Input Parameter(s)  : unsigned int ch
  			  dma_mem_io mem_mode
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_transfer_init(unsigned int ch, enum dma_mem_io mem_mode)
{
	struct dmachannel *dmach = (struct dmachannel*)ch;
	struct item_node *node;
	unsigned long flags;

	if(!dmach->device_id)
		return (-EINVAL);

	if(dma_is_enabled(ch))
		return (-EBUSY);

	local_irq_save(flags);

	// move node from active list to free list
	while(!list_empty(&dmach->active_nodes)) {
		node = (struct item_node*)list_first_entry(&dmach->active_nodes,
			struct item_node, link);
		list_move_tail((struct list_head*)node, &dmach->free_nodes);
	}

	dmach->mode = mem_mode;
	dmach->num_node = 0;
	dmach->cur_node = NULL;

	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_transfer_init);

/*******************************************************************************
  * Function Name       : dma_transfer_insert
  * Input Parameter(s)  : unsigned int ch
  			  unsigned int src
  			  unsigned int dest
  			  unsigned int size
  			  dma_control ctrl
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_transfer_insert(unsigned int ch, unsigned int src, unsigned int dest,
	unsigned int size, struct dma_control *ctrl)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	struct item_node *node = NULL;
	unsigned long flags;

	if (!dmach->device_id)
		return (-EINVAL);

	node = dma_get_free_node(dmach);
	if(!node) {
		return (-ENOMEM);
	}

	node->src_addr = src;
	node->dst_addr = dest;
	node->req_id = ctrl->request_id;
	node->size = size - 1;

	switch(ctrl->interrupt) {
	case DMA_INT_DISABLE:
		node->int_flag = 0;
		break;
	default:
		node->int_flag = 1;
	}

	switch(ctrl->transfer) {
	case DMA_MEM_TO_MEM:
		node->op_mode = MODE_INTENB | MODE_SRCNOTREQCHK |
			MODE_DSTNOTREQCHK;
		break;
	case DMA_MEM_TO_IO:
		node->op_mode = MODE_INTENB | MODE_SRCNOTREQCHK |
			MODE_DSTIOMODE | MODE_DSTNOTINC |
			set_dst_io_width(ctrl->dest_width);
		break;
	case DMA_IO_TO_MEM:
		node->op_mode = MODE_INTENB | MODE_SRCIOMODE | MODE_SRCNOTINC |
			set_src_io_width(ctrl->src_width) | MODE_DSTNOTREQCHK;
		break;
	default:
		return (-EINVAL);
	}

	local_irq_save(flags);
	list_add_tail((struct list_head *)node, &dmach->active_nodes);
	dmach->num_node++;
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_transfer_insert);

/*******************************************************************************
  * Function Name       : dma_sg_write
  * Input Parameter(s)  : unsigned int ch
  			  struct scatterlist *sg
  			  unsigned int dest
  			  int count
  			  dma_control ctrl
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_sg_write(unsigned int ch, struct scatterlist *sg, unsigned int dest,
	int count, struct dma_control *ctrl)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	struct item_node *node = NULL;
	unsigned int int_flag, i, op_mode;
	unsigned long flags;

	if (!dmach->device_id)
		return (-EINVAL);

	switch(ctrl->interrupt) {
	case DMA_INT_DISABLE:
	case DMA_INT_LAST_BLOCK:
		int_flag = 0;
		break;
	default:
		int_flag = 1;
	}

	switch(ctrl->transfer) {
	case DMA_MEM_TO_MEM:
		op_mode = MODE_INTENB | MODE_SRCNOTREQCHK | MODE_DSTNOTREQCHK;
		break;
	case DMA_MEM_TO_IO:
		op_mode = MODE_INTENB | MODE_SRCNOTREQCHK | MODE_DSTIOMODE |
			MODE_DSTNOTINC | set_dst_io_width(ctrl->dest_width);
		break;
	case DMA_IO_TO_MEM:
		op_mode = MODE_INTENB | MODE_SRCIOMODE | MODE_SRCNOTINC |
			set_src_io_width(ctrl->src_width) | MODE_DSTNOTREQCHK;
		break;
	default:
		return (-EINVAL);
	}

	local_irq_save(flags);
	
	dmach->num_node = 0;
	
	for (i = 0; i < count; i++) {
		node = dma_get_free_node(dmach);
		if(!node) {
			return (-ENOMEM);
		}
		
		node->src_addr = sg_phys(&sg[i]);
		node->dst_addr = dest;
		node->req_id = ctrl->request_id;
		node->size = sg[i].length - 1;
		node->op_mode = op_mode;
		node->int_flag = int_flag;
		
		++dmach->num_node;
		list_add_tail((struct list_head *)node, &dmach->active_nodes);
	}

	if (ctrl->interrupt == DMA_INT_LAST_BLOCK)
		node->int_flag = 1;
	
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_sg_write);

/*******************************************************************************
  * Function Name       : dma_sg_read
  * Input Parameter(s)  : unsigned int ch
  			  unsigned int src
  			  struct scatterlist *sg
  			  int count
  			  dma_control ctrl
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_sg_read(unsigned int ch, unsigned int src, struct scatterlist *sg,
	int count, struct dma_control *ctrl)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	struct item_node *node = NULL;
	unsigned int int_flag, i, op_mode;
	unsigned long flags;

	if (!dmach->device_id)
		return (-EINVAL);

	switch(ctrl->interrupt) {
	case DMA_INT_DISABLE:
	case DMA_INT_LAST_BLOCK:
		int_flag = 0;
		break;
	default:
		int_flag = 1;
	}

	switch(ctrl->transfer) {
	case DMA_MEM_TO_MEM:
		op_mode = MODE_INTENB | MODE_SRCNOTREQCHK | MODE_DSTNOTREQCHK;
		break;
	case DMA_MEM_TO_IO:
		op_mode = MODE_INTENB | MODE_SRCNOTREQCHK | MODE_DSTIOMODE |
			MODE_DSTNOTINC | set_dst_io_width(ctrl->dest_width);
		break;
	case DMA_IO_TO_MEM:
		op_mode = MODE_INTENB | MODE_SRCIOMODE | MODE_SRCNOTINC |
			set_src_io_width(ctrl->src_width) | MODE_DSTNOTREQCHK;
		break;
	default:
		return (-EINVAL);
	}

	local_irq_save(flags);

	dmach->num_node = 0;
	
	for (i = 0; i < count; i++) {
		node = dma_get_free_node(dmach);
		if(!node) {
			return (-ENOMEM);
		}

		node->src_addr = src;
		node->dst_addr = sg_phys(&sg[i]);
		node->req_id = ctrl->request_id;
		node->size = sg[i].length - 1;
		node->op_mode = op_mode;
		node->int_flag = int_flag;
		
		++dmach->num_node;
		list_add_tail((struct list_head *)node, &dmach->active_nodes);
	}

	if (ctrl->interrupt == DMA_INT_LAST_BLOCK)
		node->int_flag = 1;
	
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_sg_read);

/*******************************************************************************
  * Function Name       : dma_circ_write
  * Input Parameter(s)  : struct platform_device *pdev
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_circ_write(unsigned int ch, unsigned int *src_list, unsigned int dest,
	int count, unsigned int block_size, struct dma_control *ctrl)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	struct item_node *node = NULL;
	unsigned int int_flag, i, op_mode;
	unsigned long flags;

	if (!dmach->device_id)
		return (-EINVAL);

	switch(ctrl->interrupt) {
	case DMA_INT_DISABLE:
	case DMA_INT_LAST_BLOCK:
		int_flag = 0;
		break;
	default:
		int_flag = 1;
	}

	switch(ctrl->transfer) {
	case DMA_MEM_TO_MEM:
		op_mode = MODE_INTENB | MODE_SRCNOTREQCHK | MODE_DSTNOTREQCHK;
		break;
	case DMA_MEM_TO_IO:
		op_mode = MODE_INTENB | MODE_SRCNOTREQCHK | MODE_DSTIOMODE |
			MODE_DSTNOTINC | set_dst_io_width(ctrl->dest_width);
		break;
	case DMA_IO_TO_MEM:
		op_mode = MODE_INTENB | MODE_SRCIOMODE | MODE_SRCNOTINC |
			set_src_io_width(ctrl->src_width) | MODE_DSTNOTREQCHK;
		break;
	default:
		return (-EINVAL);
	}

	//DBG("opmode:%x, req_id:%d, size:%d\n", op_mode, ctrl->request_id, block_size);

	local_irq_save(flags);

	// Circular DMA Memory mapped operation..
	dmach->mode = DMA_MEM_MAPPED;
	dmach->num_node = 0;

	for (i = 0; i < count; i++) {
		node = dma_get_free_node(dmach);
		if(!node) {
			return (-ENOMEM);
		}

		node->src_addr = src_list[i];
		node->dst_addr = dest;
		node->req_id = ctrl->request_id;
		node->size = block_size - 1;
		node->op_mode = op_mode;
		node->int_flag = int_flag;
		
		++dmach->num_node;
		list_add_tail((struct list_head *)node, &dmach->active_nodes);
	}

	if (ctrl->interrupt == DMA_INT_LAST_BLOCK)
		node->int_flag = 1;
	
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_circ_write);

/*******************************************************************************
  * Function Name       : dma_circ_read
  * Input Parameter(s)  : unsigned int ch
  			  unsigned int src
  			  unsigned int *dest_list
  			  int count
  			  unsigned int block_size
  			  dma_control ctrl
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
int dma_circ_read(unsigned int ch, unsigned int src, unsigned int *dest_list, int count, unsigned int block_size, struct dma_control *ctrl)
{
	struct dmachannel *dmach = (struct dmachannel *)ch;
	struct item_node *node = NULL;
	unsigned int int_flag, i, op_mode;
	unsigned long flags;

	if (!dmach->device_id)
		return (-EINVAL);

	switch(ctrl->interrupt) {
	case DMA_INT_DISABLE:
	case DMA_INT_LAST_BLOCK:
		int_flag = 0;
		break;
	default:
		int_flag = 1;
	}

	switch(ctrl->transfer) {
	case DMA_MEM_TO_MEM:
		op_mode = MODE_INTENB | MODE_SRCNOTREQCHK | MODE_DSTNOTREQCHK;
		break;
	case DMA_MEM_TO_IO:
		op_mode = MODE_INTENB | MODE_SRCNOTREQCHK | MODE_DSTIOMODE |
			MODE_DSTNOTINC | set_dst_io_width(ctrl->dest_width);
		break;
	case DMA_IO_TO_MEM:
		op_mode = MODE_INTENB | MODE_SRCIOMODE | MODE_SRCNOTINC |
			set_src_io_width(ctrl->src_width) | MODE_DSTNOTREQCHK;
		break;
	default:
		return (-EINVAL);
	}

	local_irq_save(flags);

	// Circular DMA Memory mapped operation..
	dmach->mode = DMA_MEM_MAPPED;
	dmach->num_node = 0;

	for (i = 0; i < count; i++) {
		node = dma_get_free_node(dmach);
		if(!node) {
			return (-ENOMEM);
		}

		node->src_addr = src;
		node->dst_addr = dest_list[i];
		node->req_id = ctrl->request_id;
		node->size = block_size - 1;
		node->op_mode = op_mode;
		node->int_flag = int_flag;
		
		++dmach->num_node;
		list_add_tail((struct list_head *)node, &dmach->active_nodes);
	}

	if (ctrl->interrupt == DMA_INT_LAST_BLOCK)
		node->int_flag = 1;
	
	local_irq_restore(flags);

	return 0;
}
EXPORT_SYMBOL(dma_circ_read);

/*******************************************************************************
  * Function Name       : lf1000_dma_remove
  * Input Parameter(s)  : struct platform_device *pdev
  * Output Parameter(s) : NIL
  * Return Value        : int - 0 success else failure
  * Description         : 
  *****************************************************************************/
static int __devexit lf1000_dma_remove(struct platform_device *pdev)
{
	struct resource *res, *mem;
	struct list_head *node = NULL;
	int i;

	dmadev = dev_get_drvdata(&(pdev->dev));
	if(!dmadev)
		return 0;

	for(i = 0; i < MAX_DMA_CHANNELS; i++) {

		if (dma_is_enabled((unsigned int)&dmadev->dmach[i]))
			dma_stop((unsigned int)&dmadev->dmach[i]);
		
		free_irq(dma_to_irq(i), NULL);
		if(dmadev->dmach[i].device_id){
			kfree(dmadev->dmach[i].device_id);
			while(!list_empty(&dmadev->dmach[i].free_nodes)) {
				node = (struct list_head*)
				list_first_entry(&dmadev->dmach[i].free_nodes,
					struct item_node, link);
				list_del(node);
				kfree(node);
			}
		}
	}
#ifdef USE_IOREMAP
	iounmap(dmadev->reg);
#endif
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res) {
		mem = request_mem_region(res->start, res->end - res->start + 1,
					pdev->name);
		if (mem)
			release_resource(mem);
	}

	kfree(dmadev);
	dmadev = NULL;
	
	return 0;
}
 
 /******************************************************************************
   * Function Name	 : lf1000_dma_probe
   * Input Parameter(s)  : struct platform_device *pdev
   * Output Parameter(s) : NIL
   * Return Value		 : int - 0 success else failure
   * Description		 : 
   ****************************************************************************/
static int __devinit lf1000_dma_probe(struct platform_device *pdev)
{
	struct resource *res, *mem = NULL;
	int ret = 0, i;
	unsigned int offset;

	DBG("Probing LF1000 DMA Controller...\n");

	/* allocate dma controllor information */
	dmadev = kzalloc(sizeof(struct dma_info), __GFP_ZERO);
	if(!dmadev) {
		DBG("DMA kzalloc error!\n");
		return -ENOMEM;
	}

	DBG("dmadev (%p)\n", dmadev);
#ifdef USE_IOREMAP
	/* reserve static register mappings */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		DBG("DMA platform_get_resource error!\n");
		ret = -ENOENT;
		goto err;
	}

	mem = request_mem_region(res->start, res->end - res->start + 1,
		pdev->name);
	if (mem == NULL) {
		DBG("DMA request_mem_region error!\n");
		ret = -ENOENT;
		goto err;
	}

	dmadev->reg = ioremap(res->start, res->end - res->start + 1);
	if (!dmadev->reg) {
		DBG("DMA ioremap error!\n");
		ret = -ENOENT;
		goto err;
	}
#else
	dmadev->reg = (void __iomem *)IO_ADDRESS(LF1000_DMA_BASE);
#endif
	DBG("register mapping (%p)\n", dmadev->reg);

	offset = 0;

	for (i = 0; i < MAX_DMA_CHANNELS; i++, offset+=0x80) {
		dmadev->dmach[i].reg = (void __iomem *)(dmadev->reg + offset);
		dmadev->dmach[i].priority = 1 << ((i & 0xC) >> 2);
		INIT_LIST_HEAD(&dmadev->dmach[i].active_nodes);
		INIT_LIST_HEAD(&dmadev->dmach[i].free_nodes);
		dmadev->dmach[i].state = DMAC_STOP;

		DBG("DMA channel(%d) - base: 0x%p, prio: %d\n", i,
			dmadev->reg + offset, dmadev->dmach[i].priority >> 1);

		ret = request_irq(dma_to_irq(i), dma_irq_handler,
			IRQF_DISABLED, DRIVER_NAME, NULL);
		if (ret) {
			DBG("DMAC : Register IRQ Error!\n");                                                         			ret = -EBUSY;
			goto err;
		}

		spin_lock_init(&dmadev->dmach[i].lock);
	}

	dev_set_drvdata(&(pdev->dev), dmadev);

	return 0;	
err:
	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		free_irq(dma_to_irq(i), NULL);
	}

#ifdef USE_IOREMAP
	if (dmadev->reg)
		iounmap(dmadev->reg);
#endif

	if (mem)
		release_resource(mem);

	kfree(dmadev);
	dmadev = NULL;
	return ret;
}

static struct platform_driver lf1000_dma_driver = {
	.probe 		= lf1000_dma_probe,
	.remove		= lf1000_dma_remove,
	.driver		= {
		.owner  = THIS_MODULE,
		.name	= DRIVER_NAME,
	},
};
 
 /******************************************************************************
   * Function Name		 : lf1000_dma_init
   * Input Parameter(s)  : iNIL
   * Output Parameter(s) : NIL
   * Return Value		 : int - 0 success else failure
   * Description		 : 
   ****************************************************************************/
static int __init lf1000_dma_init (void)
{
	return platform_driver_register(&lf1000_dma_driver);
}

arch_initcall(lf1000_dma_init);
 
MODULE_AUTHOR("Scott Esters / Corelogic");
MODULE_DESCRIPTION("LF1000 DMA Engine Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dma");

