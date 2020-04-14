/* LF1000 clock utilities
 * 
 * Andrey Yurovsky <ayurovsky@leapfrog.com>
 *
 * Copyright 2010 LeapFrog Enterprises Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/sysfs.h>

#include <mach/core.h>
#include <mach/clkpwr.h>

#define RESSIZE(res) (((res)->end - (res)->start)+1)

struct lf1000_clk {
	void __iomem *mem;
	struct resource *res;
};

static ssize_t show_cpu_freq_in_hz(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u32 tmp;

	tmp = get_cpu_freq();
	return sprintf(buf,"%u\n", tmp);
}

static ssize_t set_cpu_freq_in_hz(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;

	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

	set_cpu_freq(value);

	return count;
}
static DEVICE_ATTR(cpu_freq_in_hz, S_IRUGO|S_IWUGO, show_cpu_freq_in_hz,
		set_cpu_freq_in_hz);

static int set_pll(struct lf1000_clk *clkdev, const char *buf, u8 pll)
{
	u32 val;
	u32 p, m, s;
	u8 reg;

	if (pll > 1)
		return -EINVAL;
	reg = pll ? PLLSETREG1 : PLLSETREG0;

	if (sscanf(buf, "%d,%d,%d", &p, &m, &s) != 3)
		return -EINVAL;

	val = ((p & 0x1F)<<PDIV_0) | ((m & 0x3FF)<<MDIV_0) | s;

	/* don't do anything if there's no change */
	if (readl(clkdev->mem + reg) == val)
		return 0;

	/* update the PLL settings and wait for it to re-lock */
	writel(val, clkdev->mem + reg);
	val = readl(clkdev->mem + PWRMODE);
	writel(val | (1<<CHGPLL), clkdev->mem + PWRMODE);
	while (readl(clkdev->mem + PWRMODE) & (1<<CHGPLL));

	return 0;
}

static ssize_t show_pll1(struct device *pdev, struct device_attribute *attr,
		char *buf)
{
	u32 reg;
	struct lf1000_clk *clkdev = dev_get_drvdata(pdev);

	reg = readl(clkdev->mem + PLLSETREG1);
	return sprintf(buf, "%d,%d,%d\n", (reg>>PDIV_0) & 0x1F,
			(reg>>MDIV_0) & 0x3FF, reg & 0xFF);
}

static ssize_t set_pll1(struct device *pdev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	struct lf1000_clk *clkdev = dev_get_drvdata(pdev);

	ret = set_pll(clkdev, buf, 1);
	if (ret)
		return ret;

	/* update linux system tick counter when freq changed */
	lf1000_pll1_clock_changed();

	return count;
}

static DEVICE_ATTR(pll1, S_IRUGO|S_IWUGO, show_pll1, set_pll1);

static ssize_t show_pll0(struct device *pdev, struct device_attribute *attr,
		char *buf)
{
	u32 reg;
	struct lf1000_clk *clkdev = dev_get_drvdata(pdev);

	reg = readl(clkdev->mem + PLLSETREG0);
	return sprintf(buf, "%d,%d,%d\n", (reg>>PDIV_0) & 0x1F,
			(reg>>MDIV_0) & 0x3FF, reg & 0xFF);
}

static ssize_t set_pll0(struct device *pdev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	struct lf1000_clk *clkdev = dev_get_drvdata(pdev);

	ret = set_pll(clkdev, buf, 0);
	if (ret)
		return ret;
	return count;
}

static DEVICE_ATTR(pll0, S_IRUGO|S_IWUGO, show_pll0, set_pll0);

static struct attribute *clock_attributes[] = {
	&dev_attr_pll0.attr,
	&dev_attr_pll1.attr,
	&dev_attr_cpu_freq_in_hz.attr,
	NULL
};

static struct attribute_group clock_attr_group = {
	.attrs = clock_attributes,
};

static int lf1000_clock_probe(struct platform_device *pdev)
{
	int ret;
	struct lf1000_clk *clkdev;

	clkdev = kzalloc(sizeof(struct lf1000_clk), GFP_KERNEL);
	if (!clkdev)
		return -ENOMEM;

	clkdev->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!clkdev->res) {
		dev_err(&pdev->dev, "failed to get resource\n");
		ret = -ENXIO;
		goto out_res;
	}

	if (!request_mem_region(clkdev->res->start, RESSIZE(clkdev->res),
				"lf1000-clock")) {
		dev_err(&pdev->dev, "failed to get memory region\n");
		ret = -EBUSY;
		goto out_res;
	}

	clkdev->mem = ioremap_nocache(clkdev->res->start,
			RESSIZE(clkdev->res));
	if (!clkdev->mem) {
		dev_err(&pdev->dev, "failed to ioremap\n");
		ret = -ENOMEM;
		goto out_map;
	}

	platform_set_drvdata(pdev, clkdev);

	sysfs_create_group(&pdev->dev.kobj, &clock_attr_group);

	return 0;

out_map:
	release_mem_region(clkdev->res->start, RESSIZE(clkdev->res));
out_res:
	kfree(clkdev);
	
	return ret;

}

static int lf1000_clock_remove(struct platform_device* pdev)
{
	struct lf1000_clk *clkdev = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &clock_attr_group);
	iounmap(clkdev->mem);
	release_mem_region(clkdev->res->start, RESSIZE(clkdev->res));
	kfree(clkdev);
	
	return 0;
}

static struct platform_driver lf1000_clock_drv = {
	.probe		= lf1000_clock_probe,
	.remove		= lf1000_clock_remove,
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= "lf1000-clock",
		.owner	= THIS_MODULE,
	},
};

int lf1000_clock_dev_init(void)
{
	return platform_driver_register(&lf1000_clock_drv);
}
EXPORT_SYMBOL_GPL(lf1000_clock_dev_init);

void lf1000_clock_dev_exit(void)
{
	platform_driver_unregister(&lf1000_clock_drv);
}
EXPORT_SYMBOL_GPL(lf1000_clock_dev_exit);

module_init(lf1000_clock_dev_init);
module_exit(lf1000_clock_dev_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:clock");
