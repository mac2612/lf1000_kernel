/*
 * LeapFrog LFP100 device driver
 *
 * Written by Scott Esters <sesters@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The LFP100 chip haves codec, LCD backlight, and power functions in a
 * single package.  Chip support is here with addtional functionality
 * in the respective hwmon, sound/soc, and video driver directories.
 *
 * Place shared code here for all I2C access.  Cache the I2C registers, like
 * in the sound driver codecs.  Provide access via private getter/setter calls
 * and sysfs hooks.  Monitor register changes via IRQ, read_reg and write_reg
 * routines.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <mach/gpio.h>
#include <mach/lfp100.h>

struct lfp100_chip {
	wait_queue_head_t wait;
	spinlock_t lock;
	struct work_struct lfp100_work;		/* task			*/
	struct workqueue_struct *lfp100_tasks;	/* workqueue		*/
	bool	have_lfp100;			/* 1 = found chip	*/
	bool	busy;				/* 1 = chip is busy	*/
	u8	reg_cache[LFP100_NUMREGS];	/* saved reg values	*/
	u8	reg_properties[LFP100_NUMREGS];	/* register properties	*/
	struct	i2c_client *client;
	void	(*power_button_callback)(void);
};

#define REG_HAS_PASSWORD 0x01	/* register has password */
#define REG_IS_VOLATILE	 0x02	/* register value is volatile  */

static struct lfp100_chip *local_chip;

/*
 * local LFP100 write reg routine, no range checking
 */
static int lfp100_write_reg_raw(unsigned int reg, unsigned int value)
{
	struct i2c_adapter* i2c;
	struct i2c_msg i2c_messages[2];
	u8 buf[2];

	/* write new register value */
	i2c = i2c_get_adapter(0);
	if (!i2c) {
		printk(KERN_ERR "%s.%d return -EIO\n",
			__FUNCTION__, __LINE__);
		return -EIO;
	}

	buf[0] = 0xff & reg;
	buf[1] = 0xFF & value;

	/* write portion */
	i2c_messages[0].addr = LFP100_ADDR;
	i2c_messages[0].buf = buf;
	i2c_messages[0].len = 2;
	i2c_messages[0].flags = 0;      /* write */

	if (i2c_transfer(i2c, i2c_messages, 1) < 0) {
		i2c_put_adapter(i2c);
		printk(KERN_ERR "%s.%d return -EIO\n",
			__FUNCTION__, __LINE__);
		return -EIO;
	}

	i2c_put_adapter(i2c);
	return 0;
}

/*
 * local i2c read reg routine
 */
static int lfp100_read_reg_raw(unsigned int reg)
{
	struct i2c_adapter* i2c;
	struct i2c_msg i2c_messages[2];
	u8 buf[2];
	int ret;

	i2c = i2c_get_adapter(0);
	if (!i2c)
		return -1;

	buf[0] = 0xFF & reg;           /* read this register */
	buf[1] = 0;

	/* write portion */
	i2c_messages[0].addr = LFP100_ADDR;
	i2c_messages[0].buf = buf;
	i2c_messages[0].len = 1;
	i2c_messages[0].flags = 0;      /* write */

	/* read portion */
	i2c_messages[1].addr = LFP100_ADDR;
	i2c_messages[1].buf = buf;
	i2c_messages[1].len = 2;
	i2c_messages[1].flags = I2C_M_RD;

	ret = i2c_transfer(i2c, i2c_messages, 2);
	i2c_put_adapter(i2c);
	if (ret < 0) {
		printk(KERN_ERR "%s.%d reg=%d, i2c_transfer=%d\n",
			__FUNCTION__, __LINE__, reg, ret);
		return -EIO;
	}
	return 0xFF & buf[1];
}

static int lfp100_available(void)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&local_chip->busy, flags);
	ret = !local_chip->busy;
	spin_unlock_irqrestore(&local_chip->busy, flags);

	return ret;
}

/*
 * local LFP100 write reg routine
 * Handle password protected registers, lock access as needed
 */

int lfp100_write_reg(unsigned int reg, unsigned int value)
{
	u8 *cache = local_chip->reg_cache;
	u8 *properties = local_chip->reg_properties;
	unsigned long flags;
	int ret=0;
	int i;
	
	if ((!local_chip->have_lfp100) || (reg < LFP100_FIRSTREG ||
		reg > LFP100_LASTREG))
		return -EIO;

	/* serialize access to LFP100 and cache */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(lfp100_available())))
			return -ERESTARTSYS;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	/* wait for busy bit to clear */
	for (i=0; i < 20; i++) {
		if(lfp100_read_reg_raw(LFP100_STATUS2) &
			LFP100_STATUS2_ABUSY) {
			msleep(20);
		} else {
			break;
		}
	}

	/*
	 * Only perform an I2C operation if reg is
	 * volatile or the new value is different
	 */
	if ((properties[reg - LFP100_FIRSTREG] == REG_IS_VOLATILE) ||
	    (cache[reg - LFP100_FIRSTREG] != value)) {

		if (properties[reg - LFP100_FIRSTREG] ==
			REG_HAS_PASSWORD) {
			/* unlock reg if needed */
			ret =
			   lfp100_write_reg_raw(LFP100_PASSWORD,
				LFP100_UNLOCK(reg));
			if (ret < 0)
				goto exit;
		}
		ret = lfp100_write_reg_raw(reg, value);

		if (0 <= ret) /* wrote value to hardware, update the cache */
			cache[reg - LFP100_FIRSTREG] = value;
	}

	/* release LFP100 */
exit:
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);
	return ret;
}

EXPORT_SYMBOL(lfp100_write_reg);

/*
 * local i2c read reg routine, use buffer if possible
 * Handle password protected registers, lock access as needed
 */

int lfp100_read_reg(unsigned int reg)
{
	u8 *cache = local_chip->reg_cache;
	u8 *properties = local_chip->reg_properties;
	unsigned long flags;
	int ret;

	if ((!local_chip->have_lfp100) || (reg < LFP100_FIRSTREG ||
		reg > LFP100_LASTREG))
		return -EIO;

	/* serialize access to LFP100 and cache */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(lfp100_available())))
			return -ERESTARTSYS;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	/* return cached value if register is not volatile */
	if (properties[reg - LFP100_FIRSTREG] != REG_IS_VOLATILE) {
		ret = cache[reg - LFP100_FIRSTREG];
	} else { /* register is volatile, read it */
		ret = lfp100_read_reg_raw(reg);
		if (0 <= ret )	/* update the cache */
			cache[reg - LFP100_FIRSTREG] = ret;
		else
			printk(KERN_ERR "%s.%d ret=%d\n",
				__FUNCTION__, __LINE__, ret);
	}

	/* release LFP100 */
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);

	return ret;
}

EXPORT_SYMBOL(lfp100_read_reg);

/*
 * sysfs interface
 */

static ssize_t show_volume(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u32 tmp;

	tmp = lfp100_read_reg(LFP100_VOLUME);
	return sprintf(buf, "VOLUME = %d\n", tmp);
}

static ssize_t set_volume(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;

	if (63 < value)
		return -EINVAL;

	lfp100_write_reg(LFP100_VOLUME, (u8)value);
	return count;
}

static DEVICE_ATTR(volume, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_volume, set_volume);

static ssize_t show_backlight(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	u32 tmp;

	tmp = lfp100_read_reg(LFP100_WLED);
	return sprintf(buf, "BACKLIGHT = %d\n", tmp);
}

static ssize_t set_backlight(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%x", &value) != 1)
		return -EINVAL;

	if (31 < value)
		return -EINVAL;

	lfp100_write_reg(LFP100_WLED, (u8)value);
	return count;
}

static DEVICE_ATTR(backlight, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH,
	show_backlight, set_backlight);

static struct attribute *lfp100_attributes[] = {
	&dev_attr_volume.attr,
	&dev_attr_backlight.attr,
	NULL
};

static struct attribute_group lfp100_attr_group = {
	.attrs = lfp100_attributes
};


static void lfp100_monitor_task(struct work_struct *work)
{
	u8 *cache = local_chip->reg_cache;
	unsigned long flags;
	int reg;

	/* serialize access to LFP100 and cache */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(lfp100_available())))
			return;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	/* read LFP100 registers */
	reg = lfp100_read_reg_raw(LFP100_CONTROL);
	if (0 <= reg)
		cache[LFP100_CONTROL] = reg;

	reg = lfp100_read_reg_raw(LFP100_STATUS1);
	if (0 <= reg)
		cache[LFP100_STATUS1] = reg;

	reg = lfp100_read_reg_raw(LFP100_STATUS2);
	if (0 <= reg)
		cache[LFP100_STATUS2] = reg;

	reg = lfp100_read_reg_raw(LFP100_INT1);
	if (0 <= reg)
		cache[LFP100_INT1] = reg;

	reg = lfp100_read_reg_raw(LFP100_INT2);
	if (0 <= reg)
		cache[LFP100_INT2] = reg;

	reg = lfp100_read_reg_raw(LFP100_INT3);
	if (0 <= reg)
		cache[LFP100_INT3] = reg;

	/* release LFP100 */
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);

	gpio_set_int(lf1000_l2p_port(LFP100_INT),
		     lf1000_l2p_pin(LFP100_INT), 1);	//renable ints

	if (local_chip->power_button_callback)		// call power handler
		local_chip->power_button_callback();
	return;
}

/*
 * unmute HP or SPK, depending on HP switch setting
 */
void lfp100_unmute_hp_sp(void)
{
	int	reg;
	int	ret;
	int	hp;
	int	i;
	int	status2;
	unsigned long flags;
	u8	*cache = local_chip->reg_cache;

	/* serialize access to LFP100 */
	spin_lock_irqsave(&local_chip->lock, flags);
	while (local_chip->busy) {
		spin_unlock_irqrestore(&local_chip->lock, flags);
		if (wait_event_interruptible(local_chip->wait,
			(lfp100_available())))
			return;
		spin_lock_irqsave(&local_chip->lock, flags);
	}
	local_chip->busy = 1;
	spin_unlock_irqrestore(&local_chip->lock, flags);

	/* wait for busy bit to clear */
	for (i=0; i < 20; i++) {
		if (lfp100_read_reg_raw(LFP100_STATUS2) &
				LFP100_STATUS2_ABUSY) {
			msleep(20);
		} else {
			break;
		}
	}

	/* read HP status switch before change */
	status2 = lfp100_read_reg_raw(LFP100_STATUS2);
	if (status2 < 0) {
		printk(KERN_ERR "%s.%d error reading LFP100_STATUS2 (%d)\n",
			__FUNCTION__, __LINE__, status2);
		goto read_err;
	}
	hp = (status2 & LFP100_STATUS2_HP) ? 1 : 0;

	reg = lfp100_read_reg_raw(LFP100_A_CONTROL);
	if (reg < 0) {
		printk(KERN_ERR "%s.%d error reading LFP100_A_CONTROL (%d)\n",
			__FUNCTION__, __LINE__, reg);
		goto read_err;
	}

	/* remove existing HP and SPK enable bits */
	reg &= ~(LFP100_A_CONTROL_HP_EN | LFP100_A_CONTROL_SPK_EN);

	if (hp)
		reg |= LFP100_A_CONTROL_HP_EN;
	else
		reg |= LFP100_A_CONTROL_SPK_EN;

	ret = lfp100_write_reg_raw(LFP100_A_CONTROL, reg);
	if (0 <= ret) /* wrote value to hardware, update the cache */
		cache[LFP100_A_CONTROL - LFP100_FIRSTREG] = reg;

	/* wait for busy bit to clear */
	for (i=0; i < 20; i++) {
		if (lfp100_read_reg_raw(LFP100_STATUS2) &
				LFP100_STATUS2_ABUSY) {
			msleep(20);
		} else {
			break;
		}
	}

read_err:
	/* release LFP100 */
	spin_lock_irqsave(&local_chip->lock, flags);
	local_chip->busy = 0;
	spin_unlock_irqrestore(&local_chip->lock, flags);
	wake_up_interruptible(&local_chip->wait);
}

EXPORT_SYMBOL(lfp100_unmute_hp_sp);

/*
 * got an interrupt.  Let background task process it.
 */
static irqreturn_t lfp100_chip_irq(enum gpio_port port, enum gpio_pin pin,
					void *priv)
{
	struct lfp100_chip *chip = (struct lfp100_chip *)priv;

	if (gpio_get_pend(lf1000_l2p_port(LFP100_INT),
			  lf1000_l2p_pin(LFP100_INT))) {
		/* disable int, clear pending int, schedule task */
		gpio_set_int(lf1000_l2p_port(LFP100_INT),
			     lf1000_l2p_pin(LFP100_INT), 0);
		gpio_clear_pend(lf1000_l2p_port(LFP100_INT),
				lf1000_l2p_pin(LFP100_INT));
		queue_work(chip->lfp100_tasks, &chip->lfp100_work);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

int lfp100_is_battery(void)
{
	int ret = 0;

	ret = ((lfp100_read_reg(LFP100_STATUS1) &
		LFP100_STATUS1_SOURCE) == LFP100_STATUS1_SOURCE_BAT) ? 1 : 0;
	return ret;
}

EXPORT_SYMBOL(lfp100_is_battery);

void lfp100_set_power_button_callback(void (*callback)(void))
{
	local_chip->power_button_callback = callback;
}

EXPORT_SYMBOL(lfp100_set_power_button_callback);

int lfp100_get_power_button(void)
{
	int ret = 0;

	if (lfp100_read_reg(LFP100_STATUS1) &
		LFP100_STATUS1_PB) {
		ret = 1;
	}
	return ret;
}

EXPORT_SYMBOL(lfp100_get_power_button);

void lfp100_set_power_standby(void)
{
	/* go to standby mode */
	lfp100_write_reg(LFP100_CONTROL, LFP100_CONTROL_STANDBY);
}

EXPORT_SYMBOL(lfp100_set_power_standby);

int lfp100_have_lfp100(void)
{
	if (local_chip && local_chip->have_lfp100)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(lfp100_have_lfp100);

static int lfp100_chip_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int i;
	int ret = 0;
	struct lfp100_chip *priv;

	priv = kzalloc(sizeof(struct lfp100_chip), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto fail_alloc;
	}

	i2c_set_clientdata(client, priv);
	priv->client = client;
	local_chip = priv;

	/* mark registers that are password protected or volatile */
	priv->reg_properties[LFP100_PASSWORD]	= REG_IS_VOLATILE;
	priv->reg_properties[LFP100_DCDC1_PW]   = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_DCDC2_PW]   = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SLEW_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_LDO1_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_LDO2_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_LDO3_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_PG_PW]      = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_UVLO_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ1_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ2_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ3_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ4_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_SEQ5_PW]    = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_FORMAT_PW]  = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_A_APOP_PW]  = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_GAINADJ_PW] = REG_HAS_PASSWORD;
	priv->reg_properties[LFP100_VLIMIT_PW]  = REG_HAS_PASSWORD;
	init_waitqueue_head(&priv->wait);
	priv->lock = SPIN_LOCK_UNLOCKED;

	/* initialize LFP100 cache by reading registers */

	for (i = 0; i < LFP100_NUMREGS; i++) {
		ret = lfp100_read_reg_raw(i);
		if (ret < 0) {
			printk(KERN_ERR "%s.%d No LFP100\n",
				__FUNCTION__, __LINE__);
			ret = -ENXIO;
			goto no_lfp100;
		}
		priv->reg_cache[i] = ret;
	}
	priv->have_lfp100 = 1;		/* read registers */
	ret = sysfs_create_group(&client->dev.kobj, &lfp100_attr_group);
	if (ret)
		goto sysfs_fail;

	ret = gpio_request_irq(lf1000_l2p_port(LFP100_INT),
			       lf1000_l2p_pin(LFP100_INT),
		lfp100_chip_irq, priv);
	if (ret) {
		printk(KERN_ERR "%s.%d: failed to get LFP100 IRQ\n",
			__FUNCTION__, __LINE__);
		goto sysfs_fail;
	}

	/* initialize worker thread which processes irq */
	priv->lfp100_tasks = create_singlethread_workqueue("lfp100 tasks");
	INIT_WORK(&priv->lfp100_work, lfp100_monitor_task);

	/* turn on LFP100 backlight */
	lfp100_write_reg(LFP100_P_ENABLE,
		lfp100_read_reg(LFP100_P_ENABLE) | LFP100_P_ENABLE_WLED_EN);
	/* setup LFP100 IRQ for USB, AC, power button, or headphone changes */
	lfp100_write_reg(LFP100_MASK1, 0xFF);
	lfp100_write_reg(LFP100_MASK2,
		~(LFP100_MASK2_USBM | LFP100_MASK2_ACM | LFP100_MASK2_PBM));
	lfp100_write_reg(LFP100_MASK3, ~(LFP100_MASK3_HP));

	/* clear any pending interrupts */
	lfp100_read_reg(LFP100_INT1);
	lfp100_read_reg(LFP100_INT2);
	lfp100_read_reg(LFP100_INT3);

	/* setup IRQ */
	gpio_set_fn(lf1000_l2p_port(LFP100_INT),
		    lf1000_l2p_pin(LFP100_INT), GPIO_GPIOFN);
	gpio_set_int_mode(lf1000_l2p_port(LFP100_INT),
			  lf1000_l2p_pin(LFP100_INT),
			  GPIO_IMODE_LOW_LEVEL);
	gpio_clear_pend(lf1000_l2p_port(LFP100_INT),
			lf1000_l2p_pin(LFP100_INT));
	gpio_set_int(lf1000_l2p_port(LFP100_INT),
		     lf1000_l2p_pin(LFP100_INT), 1);

	ret = 0;
no_lfp100:
	return ret;

sysfs_fail:
	sysfs_remove_group(&client->dev.kobj, &lfp100_attr_group);
	kfree(i2c_get_clientdata(client));
fail_alloc:
	return ret;
}


static int lfp100_chip_remove(struct i2c_client *client)
{
	gpio_set_int(lf1000_l2p_port(LFP100_INT),
		     lf1000_l2p_pin(LFP100_INT), 0); /* disable int   */
	gpio_clear_pend(lf1000_l2p_port(LFP100_INT),
			lf1000_l2p_pin(LFP100_INT)); /* clear pending */
	gpio_free_irq(lf1000_l2p_port(LFP100_INT),
		      lf1000_l2p_pin(LFP100_INT), lfp100_chip_irq);

	destroy_workqueue(local_chip->lfp100_tasks);

	sysfs_remove_group(&client->dev.kobj, &lfp100_attr_group);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id lfp100_chip_id[] = {
	{ LFP100_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, lfp100_chip_id);

static struct i2c_driver lfp100_chip_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= LFP100_NAME,
	},
	.probe		= lfp100_chip_probe,
	.remove		= lfp100_chip_remove,
	.id_table	= lfp100_chip_id,
};

/*
 * module stuff
 */

static int __init lfp100_chip_init(void)
{
	return i2c_add_driver(&lfp100_chip_driver);
}

static void lfp100_chip_exit(void)
{
	i2c_del_driver(&lfp100_chip_driver);
}

MODULE_AUTHOR("Scott Esters");
MODULE_DESCRIPTION("LFP100 support");
MODULE_LICENSE("GPL");

module_init(lfp100_chip_init);
module_exit(lfp100_chip_exit);
