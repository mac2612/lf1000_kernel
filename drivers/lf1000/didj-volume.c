
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#include <mach/gpio.h>
#include <mach/adc.h>


#define ADC_VARIATION	3
#define VOLUME_SAMPLING_J		HZ / 10
/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
define device
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

struct volume {
	struct input_dev	*i_dev;

	struct	timer_list        volume_timer;
	struct	workqueue_struct *volume_tasks;
	struct	work_struct       volume_work;	// check touchscreen

	int	stop_timer;	          // non-zero = stop timer reload
	int	sample_rate_in_jiffies;	  // sample rate
	int adc_reading_last;
	int adc_reading;
	int adc_variation;

} volume_dev;

struct volume * v_dev = &volume_dev;


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
trigger event
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
static void get_volume(struct work_struct *work)
{
	int reading_diff;
	v_dev->adc_reading = adc_GetReading(LF1000_ADC_VOLUMESENSE);
	reading_diff = abs(v_dev->adc_reading - v_dev->adc_reading_last);

	if (reading_diff > v_dev->adc_variation){
		v_dev->adc_reading_last = v_dev->adc_reading;
		input_report_abs(v_dev->i_dev, ABS_X, v_dev->adc_reading);
		input_sync(v_dev->i_dev);
	}
}

void volume_monitor_task(unsigned long data)
{
	struct volume *v_dev = (struct volume *)data;

	if (!v_dev->stop_timer) {	// not stopping, reload timer
		queue_work(v_dev->volume_tasks, &v_dev->volume_work);
		v_dev->volume_timer.expires += v_dev->sample_rate_in_jiffies;
		v_dev->volume_timer.function = volume_monitor_task;
		v_dev->volume_timer.data = data;
		add_timer(&v_dev->volume_timer);
	}
}


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
/sys/ attributes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

static ssize_t show_name(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "Name is %s\n", __FILE__);
}
static DEVICE_ATTR(name, S_IRUSR|S_IRGRP|S_IROTH, show_name, NULL);


static ssize_t show_adc_reading(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", adc_GetReading(LF1000_ADC_VOLUMESENSE));
}
static DEVICE_ATTR(adc_reading, S_IRUSR|S_IRGRP|S_IROTH, show_adc_reading, NULL);


static ssize_t show_adc_reading_last(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", v_dev->adc_reading_last);
}
static DEVICE_ATTR(adc_reading_last, S_IRUSR|S_IRGRP|S_IROTH, show_adc_reading_last, NULL);


static ssize_t show_adc_variation(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", v_dev->adc_variation);
}
static DEVICE_ATTR(adc_variation, S_IRUSR|S_IRGRP|S_IROTH, show_adc_variation, NULL);



static struct attribute *volume_attributes[] = {
	&dev_attr_name.attr,
	&dev_attr_adc_reading.attr,
	&dev_attr_adc_reading_last.attr,
	&dev_attr_adc_variation.attr,
	NULL
};

static struct attribute_group volume_attr_group = {
	.attrs = volume_attributes
};


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
module init exit probe etc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

static int openlf_didj_vol_probe(struct platform_device *pdev)
{
	struct input_dev *i_dev;
	int error;

	i_dev = input_allocate_device();

	if (!i_dev)  {
		error = -ENOMEM;
		goto err_free_devs;
	}

	i_dev->name		= "OpenLF Didj volume interface";
	i_dev->phys		= "openlf/volume";
	i_dev->id.bustype	= BUS_HOST;
	i_dev->id.vendor	= 0x0001;
	i_dev->id.product	= 0x0001;
	i_dev->id.version	= 0x0001;
	v_dev->i_dev		= i_dev;

	v_dev->adc_reading_last = 0;
	v_dev->adc_reading = adc_GetReading(LF1000_ADC_VOLUMESENSE);
	v_dev->adc_variation = ADC_VARIATION;
	v_dev->sample_rate_in_jiffies = VOLUME_SAMPLING_J;
	sysfs_create_group(&pdev->dev.kobj, &volume_attr_group);

	/* event types that we support */	
	i_dev->evbit[0]			   = BIT(EV_ABS);

	platform_set_drvdata(pdev, v_dev);
	input_set_abs_params(i_dev, ABS_X, 0, 1023, 0, 0);
	error = input_register_device(i_dev);
	if(error)
		goto err_free_devs;

	v_dev->volume_tasks = create_singlethread_workqueue("didj-volume tasks");
	INIT_WORK(&v_dev->volume_work, get_volume);

	setup_timer(&v_dev->volume_timer, volume_monitor_task, (unsigned long)v_dev);
	v_dev->volume_timer.expires = get_jiffies_64() + v_dev->sample_rate_in_jiffies;

	add_timer(&v_dev->volume_timer);	/*run*/

	dev_info( &v_dev->i_dev->dev, "openlf-didj-volume: probe done ok\n");

	return 0;

err_free_devs:
	if (&v_dev->volume_timer != NULL) {
		v_dev->stop_timer = 1;		// don't reload timer
		del_timer_sync(&v_dev->volume_timer);
	}
	sysfs_remove_group(&pdev->dev.kobj, &volume_attr_group);
	input_free_device(v_dev->i_dev);
	return error;
}


static int openlf_didj_vol_remove(struct platform_device *pdev)
{
	struct volume *v_dev = platform_get_drvdata(pdev);

	if (&v_dev->volume_timer != NULL) {
		v_dev->stop_timer = 1;		// don't reload timer
		del_timer_sync(&v_dev->volume_timer);
	}

	destroy_workqueue(v_dev->volume_tasks);

	sysfs_remove_group(&pdev->dev.kobj, &volume_attr_group);

	input_unregister_device(v_dev->i_dev);
	return 0;
}
void openlf_didj_vol_release(struct device *dev)
{
}

static struct platform_device openlf_didj_vol_device = {
	.name			= "openlf-didj-volume",
	.id				= -1,
	.num_resources	= 0,
	.dev			= {
		.release = openlf_didj_vol_release,
	}
};

static struct platform_driver openlf_didj_vol_driver = {
	.probe		= openlf_didj_vol_probe,
	.remove		= openlf_didj_vol_remove,
	.driver		= {
		.name	= "openlf-didj-volume",
		.owner	= THIS_MODULE,
		},
};

static int __init openlf_didj_vol_init(void)
{
	int ret;
	ret = platform_device_register(&openlf_didj_vol_device);
	ret = platform_driver_register(&openlf_didj_vol_driver);
	return(ret);
}

static void __exit openlf_didj_vol_exit(void)
{
	platform_driver_unregister(&openlf_didj_vol_driver);
	platform_device_unregister(&openlf_didj_vol_device);
}

module_init(openlf_didj_vol_init);
module_exit(openlf_didj_vol_exit);

MODULE_AUTHOR("Jason Pruitt");
MODULE_DESCRIPTION("Monitor Didj Volume Slider");
MODULE_LICENSE("GPL");
