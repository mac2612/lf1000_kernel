/*
 * drivers/input/misc/lf1000_aclmtr.c
 *
 * Accelerometer driver for the LF1000 platform.
 * Generically named to fit input driver framework.
 * Supports Bosch BMA150, BMA220 devices via I2C.
 *
 * Copyright 2010 LeapFrog Enterprises Inc.
 *
 * Dave Milici <dmilici@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include <mach/platform.h>
#include <mach/gpio.h>

#include <linux/sysfs.h>

/*
 * device
 */

#define INPUT_SAMPLING_HZ		10
#define INPUT_SAMPLING_JIFFIES	(HZ / INPUT_SAMPLING_HZ)

#define BMA150_ADDR			0x70
#define BMA220_ADDR			0x16

#define MIN_XYZ				-(0x001F+1)
#define MAX_XYZ				0x001F

#define MIN_PHI				0x00
#define MAX_PHI				0x07

struct lf1000_aclmtr {
	struct input_dev *input;
	struct timer_list input_timer;

	struct	workqueue_struct *input_tasks;
	struct	work_struct       input_work;

	struct i2c_client  *control_data;
	struct i2c_adapter *i2c_dev;
	unsigned int		i2c_bus;
	unsigned int		i2c_addr;

	unsigned int do_enable;
	unsigned int do_orient;
	unsigned int do_tick;
	unsigned int rate;
	unsigned int rate_jiffies;
	unsigned int average;
	int biasx, biasy, biasz;
	int	x, y, z, phi;
};

static int bma_write_reg(struct lf1000_aclmtr *dev, unsigned int reg,
		unsigned int value)
{
	struct i2c_adapter *adapter = i2c_get_adapter(dev->i2c_bus);
	struct i2c_msg msg;
	char buf[2];
	int ret;

	/* BMA220 left-justified index */
	if (BMA220_ADDR == dev->i2c_addr)
		reg <<= 1;

	buf[0] = reg & 0xFF;
	buf[1] = value & 0xFF;

	msg.addr = dev->i2c_addr;
	msg.buf = buf;
	msg.len = 2;
	msg.flags = 0; /* write */

	ret = i2c_transfer(adapter, &msg, 1);

	i2c_put_adapter(adapter);

	if (ret < 0)
		return -EIO;

	return 0;
}

static unsigned int bma_read_reg(struct lf1000_aclmtr *dev,
		unsigned int reg)
{
	struct i2c_adapter *adapter = i2c_get_adapter(dev->i2c_bus);
	struct i2c_msg msg[2];
	char buf[2];
	int ret;

	/* BMA220 left-justified index */
	if (BMA220_ADDR == dev->i2c_addr)
		reg <<= 1;

	buf[0] = reg & 0xFF;
	buf[1] = 0;

	msg[0].addr = dev->i2c_addr;
	msg[0].buf = buf;
	msg[0].len = 1;
	msg[0].flags = 0; /* write */

	msg[1].addr = dev->i2c_addr;
	msg[1].buf = buf;
	msg[1].len = 2;
	msg[1].flags = I2C_M_RD;

	ret = i2c_transfer(adapter, msg, 2);

	i2c_put_adapter(adapter);

	return (ret < 0) ? ret : buf[1];
}

static int bma_detect(struct lf1000_aclmtr* dev)
{
	int id[2];
	int n = 0;

	/* Madrid, Emerald CIP */
	if (gpio_have_gpio_madrid() || n++ || gpio_have_gpio_emerald()) {
		dev->i2c_bus = n;
		dev->i2c_addr = BMA220_ADDR;
		id[0] = bma_read_reg(dev, 0x00);
		id[1] = bma_read_reg(dev, 0x01);

		if (0xDD == id[0] && 0x00 == id[1]) {
			printk(KERN_INFO "%s: BMA220 device found\n", __FUNCTION__);
			bma_write_reg(dev, 0x0D, 0xC0);	/* data mode enable */
			bma_write_reg(dev, 0x0F, 0x07);	/* x,y,z axis enable */
			return 1;
		}
	}

	/* Acorn */
	if (gpio_have_gpio_acorn()) {
		dev->i2c_bus = 0;
		dev->i2c_addr = BMA150_ADDR;
		id[0] = bma_read_reg(dev, 0x00);
		id[1] = bma_read_reg(dev, 0x01);

		if (0x02 == id[0] && 0x11 == id[1]) {
			printk(KERN_INFO "%s: BMA150 device found\n", __FUNCTION__);
			return 1;
		}
	}

	printk(KERN_INFO "%s: device not found\n", __FUNCTION__);
	return 0;
}

static void bma150_get_xyz(struct lf1000_aclmtr* dev, int* x, int* y, int* z)
{
	*x = (bma_read_reg(dev, 0x02) >> 6) | (bma_read_reg(dev, 0x03) << 2);
	*y = (bma_read_reg(dev, 0x04) >> 6) | (bma_read_reg(dev, 0x05) << 2);
	*z = (bma_read_reg(dev, 0x06) >> 6) | (bma_read_reg(dev, 0x07) << 2);
	if (*x > 0x01FF)
		*x -= 0x3FF+1;
	if (*y > 0x01FF)
		*y -= 0x3FF+1;
	if (*z > 0x01FF)
		*z -= 0x3FF+1;
	dev->x = *x;
	dev->y = *y;
	dev->z = *z;
}

static void bma220_get_xyz(struct lf1000_aclmtr* dev, int* x, int* y, int* z)
{
	*x = bma_read_reg(dev, 0x02) >> 2;
	*y = bma_read_reg(dev, 0x03) >> 2;
	*z = bma_read_reg(dev, 0x04) >> 2;
	if (*x > 0x001F)
		*x -= 0x03F+1;
	if (*y > 0x001F)
		*y -= 0x03F+1;
	if (*z > 0x001F)
		*z -= 0x03F+1;
	dev->x = *x;
	dev->y = *y;
	dev->z = *z;
}

static void get_orient(struct lf1000_aclmtr* dev, int* orient)
{
	if (BMA220_ADDR == dev->i2c_addr) {
		*orient = bma_read_reg(dev, 0x0B) >> 4;
		*orient &= MAX_PHI;
		dev->phi = *orient;
		return;
	}
	*orient = 0;
}

static void get_xyz(struct lf1000_aclmtr* dev, int* x, int* y, int* z)
{
	switch (dev->i2c_addr) {
	case BMA150_ADDR:
		return bma150_get_xyz(dev, x, y, z);
	case BMA220_ADDR:
		return bma220_get_xyz(dev, x, y, z);
	}
	*x = *y = *z = 0;
}

static struct lf1000_aclmtr* g_dev = NULL;	/* not cached in work_struct */
static void input_work_task(struct	work_struct *work)
{
	struct lf1000_aclmtr *i_dev = g_dev;
	int x, y, z;
	int orient = 0;
	static int tick=0;
	static int acount=0;
	static int sx=0, sy=0, sz=0;

	/* get x,y,z from device */
	get_xyz(i_dev, &x, &y, &z);

	/* get orientation from device */
	if (i_dev->do_orient)
		get_orient(i_dev, &orient);

	if (i_dev->average > 1)
	{
		sx += x;
		sy += y;
		sz += z;
		if (++acount >= i_dev->average)
		{
			int a=i_dev->average; /* Make signed */
			x = (sx-i_dev->biasx)/a;
			y = (sy-i_dev->biasy)/a;
			z = (sz-i_dev->biasz)/a;
			/* Clear accumulators */
			acount = 0;
			sx = sy = sz = 0;
		}
		else
			return; /* No report */
	}
	else
	{
		x -= i_dev->biasx;
		y -= i_dev->biasy;
		z -= i_dev->biasz;
	}
		
	/* report input data */
	input_report_abs(i_dev->input, ABS_X, x);
	input_report_abs(i_dev->input, ABS_Y, y);
	input_report_abs(i_dev->input, ABS_Z, z);
	/* orientation is optional */
	if (i_dev->do_orient)
		input_report_abs(i_dev->input, ABS_MISC, orient);
	/* Force at least one changing value to get a new event every sample */
	if (i_dev->do_tick)
		input_report_abs(i_dev->input, ABS_WHEEL, tick++);
	input_sync(i_dev->input);
}

static void input_monitor_task(unsigned long data)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)data;

	/* defer input sampling to work queue */
	if (i_dev->do_enable)
		queue_work(i_dev->input_tasks, &i_dev->input_work);

	/* reset task timer */
	i_dev->input_timer.expires += i_dev->rate_jiffies;
	i_dev->input_timer.function = input_monitor_task;
	i_dev->input_timer.data = data;
	add_timer(&i_dev->input_timer);
}

/*
 * sysfs Interface
 */

static ssize_t show_tick(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->do_tick);
}
static ssize_t set_tick(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->do_tick = temp;
	return(count);
}

static DEVICE_ATTR(tick, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_tick, set_tick);

static ssize_t show_rate(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->rate);
}
static ssize_t set_rate(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp <= 0 || temp > HZ)
		return -EINVAL;
	i_dev->rate = temp;
	i_dev->rate_jiffies = HZ / temp;
	return(count);
}

static DEVICE_ATTR(rate, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_rate, set_rate);

static ssize_t show_average(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->average);
}
static ssize_t set_average(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->average = temp;
	return(count);
}

static DEVICE_ATTR(average, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_average, set_average);

static ssize_t show_bias(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d %d %d\n", 
		       i_dev->biasx, i_dev->biasy, i_dev->biasz);
}
static ssize_t set_bias(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int tx, ty, tz;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%d %d %d", &tx, &ty, &tz) != 3)
		return -EINVAL;
	i_dev->biasx = tx;
	i_dev->biasy = ty;
	i_dev->biasz = tz;
	return(count);
}

static DEVICE_ATTR(bias, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_bias, set_bias);

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->do_enable);
}
static ssize_t set_enable(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->do_enable = temp;
	return(count);
}

static DEVICE_ATTR(enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_enable, set_enable);

static ssize_t show_orient(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->do_orient);
}
static ssize_t set_orient(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	i_dev->do_orient = temp;
	return(count);
}

static DEVICE_ATTR(orient, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_orient, set_orient);

static ssize_t show_raw_xyz(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d %d %d\n", i_dev->x, i_dev->y, i_dev->z);
}

static DEVICE_ATTR(raw_xyz, S_IRUSR|S_IRGRP|S_IROTH, show_raw_xyz, NULL);

static ssize_t show_raw_phi(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct lf1000_aclmtr *i_dev = (struct lf1000_aclmtr *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", i_dev->phi);
}

static DEVICE_ATTR(raw_phi, S_IRUSR|S_IRGRP|S_IROTH, show_raw_phi, NULL);

static struct attribute *aclmtr_attributes[] = {
	&dev_attr_tick.attr,
	&dev_attr_rate.attr,
	&dev_attr_average.attr,
	&dev_attr_bias.attr,
	&dev_attr_enable.attr,
	&dev_attr_orient.attr,
	&dev_attr_raw_xyz.attr,
	&dev_attr_raw_phi.attr,
	NULL
};

static struct attribute_group aclmtr_attr_group = {
	.attrs = aclmtr_attributes
};


/*
 * platform device
 */

static int lf1000_aclmtr_probe(struct platform_device *pdev)
{
	struct lf1000_aclmtr *lf1000_aclmtr_dev;
	struct input_dev *input_dev;
	int ret;

	lf1000_aclmtr_dev = kzalloc(sizeof(struct lf1000_aclmtr), GFP_KERNEL);
	if (!lf1000_aclmtr_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, lf1000_aclmtr_dev);

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		goto fail_input;
	}

	input_dev->name = "LF1000 Accelerometer";
	input_dev->phys = "lf1000/aclmtr";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	lf1000_aclmtr_dev->input = input_dev;

	lf1000_aclmtr_dev->do_enable = 0;
	lf1000_aclmtr_dev->do_orient = 0;
	lf1000_aclmtr_dev->do_tick = 0;
	lf1000_aclmtr_dev->rate = INPUT_SAMPLING_HZ;
	lf1000_aclmtr_dev->rate_jiffies = INPUT_SAMPLING_JIFFIES;
	lf1000_aclmtr_dev->average = 1;
	lf1000_aclmtr_dev->biasx = 0;
	lf1000_aclmtr_dev->biasy = 0;
	lf1000_aclmtr_dev->biasz = 0;
	
	/* event types that we support */
	input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	input_dev->absbit[0] = BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) |
		BIT_MASK(ABS_Z) | BIT_MASK(ABS_WHEEL) | BIT_MASK(ABS_MISC);
	input_set_abs_params(input_dev, ABS_X, MIN_XYZ, MAX_XYZ, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, MIN_XYZ, MAX_XYZ, 0, 0);
	input_set_abs_params(input_dev, ABS_Z, MIN_XYZ, MAX_XYZ, 0, 0);
	input_set_abs_params(input_dev, ABS_WHEEL, 0, 0x7FFFFFFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MISC, MIN_PHI, MAX_PHI, 0, 0);

	ret = input_register_device(lf1000_aclmtr_dev->input);
	if (ret)
		goto fail_register;

	/* query for accelerometer device */
	ret = bma_detect(lf1000_aclmtr_dev);
	if (!ret) {
		ret = -ENODEV;
		goto fail_detect;
	}

	/* create work queue to defer input sampling */
	lf1000_aclmtr_dev->input_tasks = create_singlethread_workqueue("accelerometer-tasks");
	INIT_WORK(&lf1000_aclmtr_dev->input_work, input_work_task);
	g_dev = lf1000_aclmtr_dev;

	/* create periodic input task */
	setup_timer(&lf1000_aclmtr_dev->input_timer, input_monitor_task, (unsigned long)lf1000_aclmtr_dev);
	lf1000_aclmtr_dev->input_timer.expires = get_jiffies_64() + 
		lf1000_aclmtr_dev->rate_jiffies;
	lf1000_aclmtr_dev->input_timer.function = input_monitor_task;
	lf1000_aclmtr_dev->input_timer.data = (unsigned long)lf1000_aclmtr_dev;
	add_timer(&lf1000_aclmtr_dev->input_timer);


	sysfs_create_group(&pdev->dev.kobj, &aclmtr_attr_group);

	return 0;

fail_detect:
	input_unregister_device(input_dev);
fail_register:
	input_free_device(input_dev);
fail_input:
	kfree(lf1000_aclmtr_dev);
	return ret;
}

static int lf1000_aclmtr_remove(struct platform_device *pdev)
{
	struct lf1000_aclmtr *lf1000_aclmtr_dev = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &aclmtr_attr_group);
	del_timer_sync(&lf1000_aclmtr_dev->input_timer);
	destroy_workqueue(lf1000_aclmtr_dev->input_tasks);
	input_unregister_device(lf1000_aclmtr_dev->input);
	kfree(lf1000_aclmtr_dev);

	return 0;
}

static struct platform_driver lf1000_aclmtr_driver = {
	.probe		= lf1000_aclmtr_probe,
	.remove		= lf1000_aclmtr_remove,
	.driver		= {
		.name		= "lf1000-aclmtr",
	},
};

/*
 * module stuff
 */

static int __devinit lf1000_aclmtr_init(void)
{
	return platform_driver_register(&lf1000_aclmtr_driver);
}

static void __exit lf1000_aclmtr_exit(void)
{
	platform_driver_unregister(&lf1000_aclmtr_driver);
}

module_init(lf1000_aclmtr_init);
module_exit(lf1000_aclmtr_exit);

MODULE_AUTHOR("Dave Milici <dmilici@leapfrog.com>");
MODULE_DESCRIPTION("LF1000 Accelerometer driver");
MODULE_LICENSE("GPL");
