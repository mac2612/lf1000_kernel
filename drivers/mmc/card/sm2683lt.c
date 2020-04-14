/*
 * Programming driver for Silicon Motion SM2683LT
 *
 * Provides one ioctl as an SD-command pass-through, and another to control
 * "card" (SD-controller + attached NAND) power.
 *
 * Copyright (c) 2011 LeapFrog Enterprises, Inc.
 *
 */

#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/scatterlist.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sm2683lt_ioctl.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#define MAX(x, y) (x > y ? x : y)

#define DRIVER_NAME	"sm2683lt"

/*
 * Use dynamic major number if none specified.
 */
static int sm2683lt_major =   0;

module_param(sm2683lt_major, int, S_IRUGO);

struct sm2683lt_dev {
	struct cdev cdev;
	dev_t devno;

	int init_level;

	struct mmc_card *card;
};

static struct sm2683lt_dev sm2683lt_device;


/*
 * Wait for the card to finish the busy state.  Similar to
 * mmc-block:get_card_status() and mmc-test:mmc_test_wait_busy()
 * TODO: use mmc_ops:mmc_send_status() instead
 */
static int sm2683lt_wait_busy(struct mmc_card *card)
{
	int ret;
	unsigned int busy;
	struct mmc_command cmd;

	busy = 0;
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));

		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (ret)
			break;

		if (!(busy++ & 0xF) && !(cmd.resp[0] & R1_READY_FOR_DATA)) {
			printk(KERN_INFO "%s: Warning: %s still busy.\n",
				mmc_hostname(card->host), DRIVER_NAME);
		}
	} while (!(cmd.resp[0] & R1_READY_FOR_DATA));

        return ret;
}

/*
 * Complete a data read or write command.  Similar to
 * mmc-test:mmc_test_buffer_transfer().
 * TODO: sanity checks for data size ( N * 512 bytes ).
 */
static int sm2683lt_data_xfer(struct mmc_card *card, struct mmc_command *cmd, struct sd_command *sd_cmd)
{
	int retval = 0;

	struct mmc_request mrq;
	struct mmc_command stop;
	struct mmc_data data;

	struct scatterlist sg;
	void *buf;

	buf = kmalloc(sd_cmd->data_len, GFP_KERNEL);
	if(buf == NULL)
		return -ENOMEM;

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&data, 0, sizeof(struct mmc_data));
	memset(&stop, 0, sizeof(struct mmc_command));

	mrq.cmd = cmd;
	mrq.data = &data;

	sg_init_one(&sg, buf, sd_cmd->data_len);

	data.blksz = 512;
	data.blocks = sd_cmd->data_len / 512;

	/*
	 * The stop command (CMD12) must be included in the mrq instead of in
	 * the userspace struct sd_command list.  Without the stop command in
	 * the mrq, the _MULTIPLE_ command fails with a timeout because the
	 * sm2683lt_wait_busy() below reads a response[0] of 0x000e0000
	 * (programming state, not ready) instead of 0x00090000 (transfer
	 * state, ready).
	 *
	 * Note: Our host controller driver, mes_sdhc, automatically issues a
	 * STOP command along with any _MULTIPLE_ commands, so this command is
	 * actually ignored on our platform.
	 */

	if( (sd_cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK) ||
	    (sd_cmd->opcode == MMC_READ_MULTIPLE_BLOCK ) )
	{
		mrq.stop = &stop;
		stop.opcode =  MMC_STOP_TRANSMISSION;
		stop.arg = 0;
		stop.flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	if(sd_cmd->opcode == MMC_WRITE_BLOCK || sd_cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK)
	{
		/* fetch input payload */
		retval = __copy_from_user(buf, (void __user *)sd_cmd->data.write_data, sd_cmd->data_len);
		data.flags = MMC_DATA_WRITE;
		if(retval)
			goto out;
	}
	else
	{
		data.flags = MMC_DATA_READ;
	}
	data.sg = &sg;
	data.sg_len = 1;

	mmc_set_data_timeout(mrq.data, card);

	mmc_wait_for_req(card->host, &mrq);

	if (cmd->error)
	{
		retval = cmd->error;
		goto out;
	}
	if (data.error)
	{
		retval = data.error;
		goto out;
	}

	retval = sm2683lt_wait_busy(card);
	if(retval)
		goto out;

	if(data.flags == MMC_DATA_READ)
	{
		/* deliver output results */
		retval = __copy_to_user((void __user *)sd_cmd->data.read_data, buf, sd_cmd->data_len);
	}

out:
	kfree(buf);
	return retval;
}

/*
 * Generic SD-command parser and dispatcher.
 */
static int sd_execute_command(struct mmc_card *card, struct sd_command *sd_cmd)
{
	int i, retval;
	struct mmc_command cmd;

	/* extract user-supplied parameters */
	cmd.opcode	= sd_cmd->opcode;
	cmd.arg		= sd_cmd->arg;
	for(i = 0; i < ARRAY_SIZE(cmd.resp); i++)
	{
		cmd.resp[i]	= sd_cmd->resp[i];
	}
	cmd.flags	= sd_cmd->flags;
	cmd.retries	= sd_cmd->retries;
	cmd.error	= sd_cmd->error;

	/* execute SD command */
	switch(cmd.opcode) {
	case MMC_WRITE_BLOCK:
	case MMC_WRITE_MULTIPLE_BLOCK:
	case MMC_READ_SINGLE_BLOCK:
	case MMC_READ_MULTIPLE_BLOCK:
		retval = sm2683lt_data_xfer(card, &cmd, sd_cmd);
		break;
	default:
		retval = mmc_wait_for_cmd(card->host, &cmd, cmd.retries);
	}
	
	/* propagate status back to user */
	for(i = 0; i < ARRAY_SIZE(sd_cmd->resp); i++)
	{
		sd_cmd->resp[i]	= cmd.resp[i];
	}
	sd_cmd->error	= cmd.error;

	return retval;
}

/*
 * Handle power toggling.  Make sure to drive SD bus pins low
 * before/during/after VCC is removed so chip isn't "IO-powered."
 * We use MMC ios.power_mode to achieve this.  Note that the mes_sdhc host
 * doesn't provide VCC control, but on Madrid we have a separate gpio for
 * switching "card" power.
 */
static int sd_set_power(struct mmc_card *card, const int set)
{
	int retval = 0;
	struct mmc_ios ios;

	memcpy(&ios, &card->host->ios, sizeof(struct mmc_ios));

	if(set)
	{
		/* restore device power / restore bus pins */
		ios.power_mode = MMC_POWER_UP;
		card->host->ops->set_ios(card->host, &ios);

		/* restore clock */
		ios.power_mode = MMC_POWER_ON;
		card->host->ops->set_ios(card->host, &ios);
	}
	else
	{
		ios.power_mode = MMC_POWER_OFF;

		/* stop clock, drive bus low, cut device power */
		card->host->ops->set_ios(card->host, &ios);
	}

	return retval;
}

/* TODO: enforce singleton access pattern? */
static int sm2683lt_open(struct inode *inode, struct file *filp)
{
	struct sm2683lt_dev *dev;
	struct mmc_card *card;

	dev = container_of(inode->i_cdev, struct sm2683lt_dev, cdev);
	filp->private_data = dev; /* for other methods */

	if(!(card = dev->card))
		return -ENXIO;

	if(!card->host)
		return -ENXIO;

	/* maintain exclusive access to host controller */
	mmc_claim_host(card->host);

	return 0;
}

static int sm2683lt_release(struct inode *inode, struct file *filp)
{
	struct sm2683lt_dev *dev;
	struct mmc_card *card;

	dev = filp->private_data;
	card = dev->card;
	
	mmc_release_host(card->host);

	return 0;
}

static int sm2683lt_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	struct sm2683lt_dev *dev;
	struct mmc_card *card;
	struct sd_command sd_cmd;

	dev = filp->private_data;
	card = dev->card;

	if(_IOC_TYPE(cmd) != SM2683LT_IOC_MAGIC)
		return -ENOTTY;

	if(_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if(retval)
		return -EFAULT;

	switch(cmd) {

	  case SM2683LT_SD_COMMAND:
		retval = __copy_from_user(&sd_cmd, (struct sd_command __user *)arg, sizeof(struct sd_command));
		if(!retval)
			retval = sd_execute_command(card, &sd_cmd);
		if(!retval)
			retval = __copy_to_user((struct sd_command __user *)arg, &sd_cmd, sizeof(struct sd_command));
		break;

	  case SM2683LT_SET_POWER:
		retval = sd_set_power(card, arg);
		break;
 
	  default:
		return -ENOTTY;
	}
	return retval;
}

static struct file_operations sm2683lt_fops = {
	.owner =    THIS_MODULE,
	.ioctl =    sm2683lt_ioctl,
	.open =     sm2683lt_open,
	.release =  sm2683lt_release,
};

static int sm2683lt_probe(struct mmc_card *card)
{
	sm2683lt_device.card = card;

	printk(KERN_INFO "%s, %s, %s, %s\n", DRIVER_NAME,
	 mmc_hostname(card->host), mmc_card_id(card), mmc_card_name(card));

	return 0;
}

static void sm2683lt_remove(struct mmc_card *card)
{
	sm2683lt_device.card = NULL;
}

#ifdef CONFIG_PM
static int sm2683lt_suspend(struct mmc_card *card, pm_message_t state)
{
	return 0;
}

static int sm2683lt_resume(struct mmc_card *card)
{
	return 0;
}
#else
#define	sm2683lt_suspend	NULL
#define sm2683lt_resume		NULL
#endif

static struct mmc_driver sm2683lt_driver = {
	.drv		= {
		.name	= DRIVER_NAME,
	},
	.probe		= sm2683lt_probe,
	.remove		= sm2683lt_remove,
	.suspend	= sm2683lt_suspend,
	.resume		= sm2683lt_resume,
};

static void __exit sd_sm2683lt_exit(void)
{
	dev_t devno = MKDEV(sm2683lt_major, 0);

	switch(sm2683lt_device.init_level) {
	case 2: cdev_del(&sm2683lt_device.cdev);
	case 1: mmc_unregister_driver(&sm2683lt_driver);
	case 0: break;
	}

	unregister_chrdev_region(devno, 1);
}

static int __init sd_sm2683lt_init(void)
{
	int result;
	dev_t devno = 0;

	sm2683lt_device.init_level = 0;
	sm2683lt_device.card = NULL;

	/*
	 * Ask for a dynamic major unless directed otherwise at load time.
	 */
	if (sm2683lt_major) {
		devno = MKDEV(sm2683lt_major, 0);
		result = register_chrdev_region(devno, 1, "sm2683lt");
	} else {
		result = alloc_chrdev_region(&devno, 0, 1, "sm2683lt");
		sm2683lt_major = MAJOR(devno);
	}
	if (result < 0) {
		printk(KERN_WARNING "sm2683lt: can't get major %d\n", sm2683lt_major);
		return result;
	}

	/* Register w/mmc bus */
	result = mmc_register_driver(&sm2683lt_driver);
	if (result) {
		printk(KERN_NOTICE "Error %d registering sm2683lt", result);
		goto fail;
	}

	sm2683lt_device.init_level = 1;

        /* Initialize device structure */
	cdev_init(&sm2683lt_device.cdev, &sm2683lt_fops);
	sm2683lt_device.cdev.owner = THIS_MODULE;
	sm2683lt_device.cdev.ops = &sm2683lt_fops;
	result = cdev_add (&sm2683lt_device.cdev, devno, 1);
	if (result) {
		printk(KERN_NOTICE "Error %d adding sm2683lt", result);
		goto fail;
	}

	sm2683lt_device.init_level = 2;

	return 0;

  fail:
	sd_sm2683lt_exit();
	return result;
}

module_init(sd_sm2683lt_init);
module_exit(sd_sm2683lt_exit);

MODULE_DESCRIPTION("SM2683LT SD Controller device driver");
MODULE_ALIAS("sd:" DRIVER_NAME);
MODULE_LICENSE("GPL");
