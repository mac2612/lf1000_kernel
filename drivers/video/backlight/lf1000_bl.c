/*
 * drivers/video/backlight/lf1000_bl.c
 *
 * PWM backlight support for the LF1000 LeapFrog boards.
 *
 * Copyright 2010 LeapFrog Enterprises Inc.
 *
 * Andrey Yurovsky <ayurovsky@leapfrog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/backlight.h>

#include <mach/platform.h>
#include <mach/pwm.h>
#include <mach/gpio.h>

#define LF1000_INITIAL_BRIGHTNESS	318	// nominal second brightest
#define LF1000_MAX_BRIGHTNESS		511

struct lf1000_bl {
	struct platform_device	*pdev;
	struct backlight_device *bl;
	u32			pwmds;
	u32			pwm_channel;
};

/* convert 9 bit intensity range to 5 bit WLED range */
#define LFP100_WLED_ENTRIES	32

static int lf1000_bl_get_brightness(struct backlight_device *bd)
{
	struct lf1000_bl *priv = bl_get_data(bd);

	return priv->pwmds;
}

static int lf1000_bl_set_brightness(struct backlight_device *bd)
{
	struct lf1000_bl *priv = bl_get_data(bd);
	int intensity = bd->props.brightness;

#if 0
	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;
#endif

	if (pwm_set_duty_cycle(priv->pwm_channel, intensity))
			return -EINVAL;

	priv->pwmds = intensity;

	return 0;
}

static struct backlight_ops lf1000_bl_ops = {
	.get_brightness	= lf1000_bl_get_brightness,
	.update_status	= lf1000_bl_set_brightness,
};

static int lf1000_bl_probe(struct platform_device *pdev)
{
        struct backlight_properties props;
	int ret;
	u8 polarity;
	struct lf1000_bl *priv;

	priv = kzalloc(sizeof(struct lf1000_bl), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "can't allocate priv data\n");
		return -ENOMEM;
	}
	priv->pdev = pdev;
        memset(&props, 0, sizeof(struct backlight_properties));
        //TODO: set default properties here.
	priv->bl = backlight_device_register("lf1000-pwm-bl",
			&pdev->dev, priv, &lf1000_bl_ops, &props);
	if (IS_ERR(priv->bl)) {
		ret = PTR_ERR(priv->bl);
		dev_err(&pdev->dev, "failed to register backlight: %d\n", ret);
		kfree(priv);
	}

	platform_set_drvdata(pdev, priv);

	priv->bl->props.power = FB_BLANK_UNBLANK;
	priv->bl->props.max_brightness = LF1000_MAX_BRIGHTNESS;
	priv->bl->props.brightness = LF1000_INITIAL_BRIGHTNESS;

	if (gpio_have_gpio_madrid()) {
		priv->pwm_channel = PWM_CHAN2;
		polarity = POL_INV;	/* inverted PWM polarity */
	} else {
		priv->pwm_channel = PWM_CHAN1;
		//On Explorer, pin A30 (PWM0) is LED_ENA and should be on
		//The bootloader does this, but can't hurt to do it here to be safe.
		polarity = POL_BYP;	/* normal PWM polarity */
		gpio_set_out_en(lf1000_l2p_port(LED_ENA),
			lf1000_l2p_pin(LED_ENA), 1);
		gpio_set_cur(lf1000_l2p_port(LED_ENA),
			lf1000_l2p_pin(LED_ENA), GPIO_CURRENT_8MA);
	}
	dev_info(&pdev->dev, "Using PWM Channel %d for backlight\n", priv->pwm_channel);
	
	ret = pwm_get_clock_rate();
	if (ret < 1) {
		dev_err(&pdev->dev, "can't get PWM rate\n");
		priv->pwmds = 0;
	} else {
		dev_info(&pdev->dev, "PWM rate is %d\n", ret);
		pwm_configure_pin(priv->pwm_channel);
		pwm_set_prescale(priv->pwm_channel, 1);
		pwm_set_period(priv->pwm_channel, 511);
		pwm_set_polarity(priv->pwm_channel, polarity);
	}

	lf1000_bl_set_brightness(priv->bl);

	return 0;
}

static int __exit lf1000_bl_remove(struct platform_device *pdev)
{
	struct lf1000_bl *priv = platform_get_drvdata(pdev);

	backlight_device_unregister(priv->bl);
	platform_set_drvdata(pdev, NULL);
	kfree(priv);

	return 0;
}

static struct platform_driver lf1000_bl_driver = {
	.probe	= lf1000_bl_probe,
	.remove	= __exit_p(lf1000_bl_remove),
	.driver = {
		.name	= "lf1000-bl",
		.owner	= THIS_MODULE,
	},
};

static int __init lf1000_bl_init(void)
{
	return platform_driver_register(&lf1000_bl_driver);
}

static void __exit lf1000_bl_exit(void)
{
	platform_driver_unregister(&lf1000_bl_driver);
}

module_init(lf1000_bl_init);
module_exit(lf1000_bl_exit);

MODULE_AUTHOR("Daniel Lazzari");
MODULE_DESCRIPTION("LF1000 backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lf1000-bl");
