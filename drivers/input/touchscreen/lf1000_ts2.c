/*
 *  LF1000 touchscreen driver
 *
 *  Author:	Scott Esters, <sesters@leapfrog.com>
 *  Created:	May 27, 2008
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This code is heavily based on ucb1x00-*.c copyrighted by Russell King
 */

#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <mach/gpio.h>
#include <mach/adc.h>

#include <linux/sysfs.h>

#define LED1_ON gpio_configure_pin(GPIO_PORT_B, GPIO_PIN0, GPIO_GPIOFN, 1, 0, 0)
#define LED2_ON gpio_configure_pin(GPIO_PORT_B, GPIO_PIN1, GPIO_GPIOFN, 1, 0, 0)

#define LED1_OFF gpio_configure_pin(GPIO_PORT_B, GPIO_PIN0, GPIO_GPIOFN, 1, 0, 1)
#define LED2_OFF gpio_configure_pin(GPIO_PORT_B, GPIO_PIN1, GPIO_GPIOFN, 1, 0, 1)

#define TOUCHSCREEN_SAMPLING_J	HZ / 100  // sample touchscreen every 10 ms

#define TOUCHSCREEN_MAJOR	247

#define RAW_ABS_PARAMS	 11	// How many input event codes raw sends

#define TS_DEBOUNCE_DOWN 1	// default debounce down, min samples for down
#define TS_DEBOUNCE_UP	 1	// default debounce up, min samples for up
/* NOTE:
 *  Using 1 as the debounce threshold is equivalent to no debouncing.
 *  This allows greatest responsiveness.  It also allows some bouncing --
 *  when the stylus hits the touch screen, bounces off, then touches the
 *  screen again.
 *  We'll need to check if this is the way we want it.
 */

// input event touch defines. X=0 and Y=0 is reserved for no touch.
#define TS_MIN_X	1	    // min X resolution value
#define TS_MAX_X	1023	// max X resolution value
#define	TS_FUZZ_X	2	    // gaussian filter window size, 0=none

#define TS_MIN_Y	1	    // min Y resolution value
#define TS_MAX_Y	1023	// max Y resolution value
#define	TS_FUZZ_Y	2	    // gaussian filter window size, 0=none

// Pressure
#define	TS_FUZZ_P	5	    // gaussian filter window size, 0=none

/* Rob's calculations indicated the driver was delaying longer than necessary for
 * the voltages to settle before reading the ADC.
 * Experiments indicated that a 10-microsecond delay is long enough for the
 * x, y, and Z readings but not for the TNT readings.  Though 15 microseconds
 * might be a long enough delay for the TNT reading, 18 seemed a little better:
 * the tnt2 reading is a bit closer to the tnt1 reading.
 */

#define TS_DELAY_IN_US	 1
#define TS_Y_DELAY_IN_US 1
#define TNT_DELAY_IN_US  1


#define	N_PCURVE_PTS	9   // With nine control points, we get 10 bins

// Default rotation, translation, and shear values.  System should set these
// at startup.  Note order in /etc/pointercal is A1 A2 A0 A4 A5 A3 A6
#define TS_A0  -12097127
#define TS_A1	   52942
#define TS_A2	     149
#define TS_A3  -12950733	
#define TS_A4	      78 
#define TS_A5	   42024
#define TS_A6	   65536

/* These are the states of a touch screen state machine managed by get_touch()
 */
#define TSTATE_DOWN         0
#define TSTATE_GOING_DOWN   1
#define TSTATE_GOING_UP     2
#define TSTATE_UP           3

static int abs_x[3] = {TS_MIN_X, TS_MAX_X, TS_FUZZ_X};
module_param_array(abs_x, int, NULL, 0);
MODULE_PARM_DESC(abs_x, "Touchscreen absolute X min, max, fuzz");

static int abs_y[3] = {TS_MIN_Y, TS_MAX_Y, TS_FUZZ_Y};
module_param_array(abs_y, int, NULL, 0);
MODULE_PARM_DESC(abs_y, "Touchscreen absolute Y min, max, fuzz");

static int abs_p[3] = {TS_MIN_X, TS_MAX_X, TS_FUZZ_P};
module_param_array(abs_p, int, NULL, 0);
MODULE_PARM_DESC(abs_p, "Touchscreen absolute P min, max, fuzz");

#define SCAN_DELAY_LUT_SIZE 8
static int scan_delay_lut[SCAN_DELAY_LUT_SIZE] = { 5, 10, 15, 20, 30, 40, 60, 80 };

/* Enable the following #define if you want to collect nand access timing data*/
// #define TS_TIMING 1
int timing = 0;

struct touch {
	struct input_dev	*i_dev, *iraw_dev;

	struct task_struct	*ts_task;

	struct	timer_list        touchscreen_timer;
	struct	workqueue_struct *touchscreen_tasks;
	struct	work_struct       touchscreen_work;	// check touchscreen

	int	stop_timer;	          // non-zero = stop timer reload
	int	sample_rate_in_jiffies;	  // screen sample rate
	int	debounce_in_samples_down; // samples before stylus down declared
	int	debounce_in_samples_up;   // samples before stylus up declared

	/*
	 * report_events, controls sending data up to Linux
	 * as well as debugging messages.  Also has bitfield definitions
	 *   bit 0 ==0 no events reported to Linux
	 *         ==1 report events to Linux
	 *   bit 1 ==1 show ADC debugging messages
	 *   bit 2 ==0 no raw events reported to Linux
	 *         ==1 report raw events to Linux
	 *   bit 3 ==x Control debugging messages
	 *   bit 4-31  Count of debugging messages
	 */
#define REBIT_REPORT    0
#define REBIT_ADC_DBG   1
#define REBIT_RAW_REPORT    2
#define REBIT_ADC_DBG_MODE   3

#define RESHIFT_COUNT	4
#define REMASK_REPORT   (1 << REBIT_REPORT)
#define REMASK_ADC_DBG  (1 << REBIT_ADC_DBG)
#define REMASK_RAW_REPORT   (1 << REBIT_RAW_REPORT)
#define REMASK_ADC_DBG_MODE  (1 << REBIT_ADC_DBG_MODE)
	int	report_events;

	int	delay_in_us;		// delay between GPIO setup and reading
	int	y_delay_in_us;		// delay between GPIO setup and reading
	int	tnt_delay_in_us;	// delay between GPIO setup and reading
					// for tnt

	int	first_adc;		// 0=didj-ts, 4=Leapster 3

	int	adc_x;			// raw adc for x measurements
	int	adc_x2;			// raw adc for x measurements

	int	adc_y;			// raw adc for y measurements
	int	adc_y2;			// raw adc for y measurements

	int	averaging;		// Number of samples to average
	int	tails;			// Tails mode, in honor of a silly bug
	int	scanning;		// Enable/Disable scanning mode
	int	scan_ctr;		// Scan counter
	int	tnt_mode;		// Which TNT to read from
	int	tnt_plane[3];		// TNT planar offset model params

	int	adc_p1;		        // raw adc for p1 measurements
	int	adc_p2;		        // raw adc for p2 measurements
	int	adc_pressure;
	int	adc_tnt1;		// raw adc for first tnt measurement
	int	adc_tnt2;		// raw adc for second tnt measurement

	int	adc_max_tnt_down;   // tnt readings must be <= this for touch
	int	adc_min_tnt_up;     // either tnt reading > this ==> Up
	int	adc_max_delta_tnt;  // for Down, both tnt readings must be 
				    // less than or equal to adc_max_tnt_down
	                            // AND must differ by no more than this.

	int	pressure_curve[N_PCURVE_PTS];	// Pressure transfer function
	// This is a peice-wise approximation to an inverse of the pressure
	// curve, so we can relate computed pressure to something more linear
	// for the user.

	int	ts_state;

	int	touch_x, touch_y;	// raw ADC touchscreen x and y values
	int	touch_pressure;		// Linearized pressure

	int	touch_button;		// screen touched
	int	screen_x, screen_y;	// transformed screen x and y values

	int	screen_last_x, screen_last_y;  // debugging values

	int	stylus_down_count;	// count consecutive stylus down samples
	int	stylus_up_count;	// count consecutive stylus up samples

	int	a[7];			// /etc/pointercal values

#ifdef TS_TIMING
	u32	min_us, max_us, N_us, sum_us;
#endif

	struct cdev *cdev;		// char device
	int	dev;
	int	major;
} touch_dev;

struct touch * t_dev = &touch_dev;

#ifdef TS_TIMING
#include <mach/platform.h>
#include <mach/common.h>
#include <mach/timer.h>


#define TIMER32(r)	REG32(IO_ADDRESS(TIMER_BASE+r))

#define TIMER_BASE	LF1000_TIMER3_BASE
/* Timer Clock Rate: 4.5Mhz */

/* start the stopwatch */
void timer_start(void)
{
	/* make sure the timer is stopped */
	BIT_CLR(TIMER32(TMRCONTROL), RUN);

	/* zero out the timer */
	TIMER32(TMRCOUNT) = 0;

	/* run the timer */
	BIT_SET(TIMER32(TMRCONTROL), RUN);
}

/* stop the stopwatch, and return the time in us*/
u32 timer_stop(void)
{
	BIT_CLR(TIMER32(TMRCONTROL), RUN);	/* stop the timer */
	BIT_SET(TIMER32(TMRCONTROL), LDCNT);	/* get access to counter */
	return (2*TIMER32(TMRCOUNT))/9;		/* and return it */
}

#endif  /* TS_TIMING */

/*
 * sysfs Interface
 */

static ssize_t show_name(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "Name is %s\n", __FILE__);
}
static DEVICE_ATTR(name, S_IRUSR|S_IRGRP|S_IROTH, show_name, NULL);

/* Return /etc/pointercal values.  pointercal order is A1 A2 A0 A4 A5 A3 A6 */
static ssize_t set_pointercal(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int i, temp[7];
	struct touch *t_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i %i %i %i %i %i %i", 
		  &temp[1], &temp[2], &temp[0],
		  &temp[4], &temp[5], &temp[3], &temp[6]) != 7)
		return -EINVAL;
	if (temp[6] == 0)
		return -EINVAL;
	for (i=0; i<7; i++)
		t_dev->a[i] = temp[i];
	dev_dbg(dev, "%s.%s:%d pointercal=%d %d %d %d %d %d %d\n",
		__FILE__, __FUNCTION__, __LINE__,
		t_dev->a[1], t_dev->a[2], t_dev->a[0],
		t_dev->a[4], t_dev->a[5], t_dev->a[3], t_dev->a[6]);
	return(count);
}

static ssize_t show_pointercal(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct touch *t_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d %d %d %d %d %d %d\n",
		t_dev->a[1], t_dev->a[2], t_dev->a[0],
		t_dev->a[4], t_dev->a[5], t_dev->a[3], t_dev->a[6]);
}
static DEVICE_ATTR(pointercal, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP|S_IWOTH, show_pointercal, set_pointercal);


/* report current x1 coordinate, 'x0' is used by calibration programs */
static ssize_t show_x0(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",lf1000_touch_dev->touch_x);
}
static DEVICE_ATTR(x0, S_IRUSR|S_IRGRP|S_IROTH, show_x0, NULL);


/* report current y coordinate, 'y1' is used by calibration programs */
static ssize_t show_y1(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->touch_y);
}
static DEVICE_ATTR(y1, S_IRUSR|S_IRGRP|S_IROTH, show_y1, NULL);

static ssize_t show_touch_x(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->touch_x);
}
static DEVICE_ATTR(touch_x, S_IRUSR|S_IRGRP|S_IROTH, show_touch_x, NULL);

static ssize_t show_touch_y(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->touch_y);
}
static DEVICE_ATTR(touch_y, S_IRUSR|S_IRGRP|S_IROTH, show_touch_y, NULL);

static ssize_t show_touch_pressure(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->touch_pressure);
}
static DEVICE_ATTR(touch_pressure, S_IRUSR|S_IRGRP|S_IROTH, show_touch_pressure, NULL);

static ssize_t show_touch_button(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->touch_button);
}
static DEVICE_ATTR(touch_button, S_IRUSR|S_IRGRP|S_IROTH, show_touch_button, NULL);

static ssize_t show_screen_x(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->screen_x);
}
static DEVICE_ATTR(screen_x, S_IRUSR|S_IRGRP|S_IROTH, show_screen_x, NULL);

static ssize_t show_screen_y(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->screen_y);
}
static DEVICE_ATTR(screen_y, S_IRUSR|S_IRGRP|S_IROTH, show_screen_y, NULL);

static ssize_t show_raw_adc(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	static char *lut = "DV^U";
	return sprintf(buf, "x1=%d x2=%d y1=%d y2=%d p1=%d p2=%d tnt1=%d tnt2=%d S=%c X=%d Y=%d P=%d\n", 
		       lf1000_touch_dev->adc_x, lf1000_touch_dev->adc_x2, 
		       lf1000_touch_dev->adc_y, lf1000_touch_dev->adc_y2, 
		       lf1000_touch_dev->adc_p1, lf1000_touch_dev->adc_p2, 
		       lf1000_touch_dev->adc_tnt1, lf1000_touch_dev->adc_tnt2, 
		       lut[lf1000_touch_dev->ts_state],
		       lf1000_touch_dev->screen_x, lf1000_touch_dev->screen_y,
		       lf1000_touch_dev->adc_pressure);
}
static DEVICE_ATTR(raw_adc, S_IRUSR|S_IRGRP|S_IROTH, show_raw_adc, NULL);

static ssize_t show_delay_in_us(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->delay_in_us);
}
static ssize_t set_delay_in_us(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	lf1000_touch_dev->delay_in_us = temp;
	dev_dbg(dev, "%s.%s:%d delay_in_us=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->delay_in_us);
	return(count);
}

static DEVICE_ATTR(delay_in_us, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_delay_in_us, set_delay_in_us);

static ssize_t show_y_delay_in_us(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->y_delay_in_us);
}
static ssize_t set_y_delay_in_us(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	lf1000_touch_dev->y_delay_in_us = temp;
	dev_dbg(dev, "%s.%s:%d y_delay_in_us=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->y_delay_in_us);
	return(count);
}

static DEVICE_ATTR(y_delay_in_us, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_y_delay_in_us, set_y_delay_in_us);

#ifdef TS_TIMING
/* Us_In_Driver: time spent in driver */
static ssize_t show_us_in_driver(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d/%d/%d N=%d\n",
		       lf1000_touch_dev->min_us,
		       lf1000_touch_dev->N_us > 0 ? 
		       lf1000_touch_dev->sum_us/lf1000_touch_dev->N_us : 0,
		       lf1000_touch_dev->max_us,
		       lf1000_touch_dev->N_us);
}
static ssize_t set_us_in_driver(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	lf1000_touch_dev->min_us = 0;
	lf1000_touch_dev->max_us = 0;
	lf1000_touch_dev->N_us = 0;
	lf1000_touch_dev->sum_us = 0;
	return(count);
}
static DEVICE_ATTR(us_in_driver, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_us_in_driver, set_us_in_driver);

/* us: time spent between write and read, in us */
static ssize_t show_us(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	// struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	/* This is revealing!  I see udelay(1) take 4us, udelay(20)=80us
	LED1_OFF; LED2_OFF;
	udelay (1);
	LED1_OFF; LED2_ON;
	udelay (2);
	LED1_ON; LED2_OFF;
	udelay (5);
	LED1_ON; LED2_ON;
	udelay (10);
	LED1_OFF; LED2_OFF;
	udelay (20);
	LED1_OFF; LED2_ON;
	udelay (2);
	LED1_ON; LED2_OFF;
	udelay (1);
	LED1_ON; LED2_ON;
	*/

        int t = timer_stop();
	timing = 0; // Let other people use timer
	return sprintf(buf, "%d\n", t);
}
static ssize_t set_us(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	/*
	timing = 1; // Don't let other people use timer
	timer_start ();
	*/
	u32 x, y, p;
	if(sscanf(buf, "%u %u %u", &x, &y, &p) != 3)
		return -EINVAL;
	input_report_abs(t_dev->i_dev, ABS_X, x);
	input_report_abs(t_dev->i_dev, ABS_Y, y);
	input_report_abs(t_dev->i_dev, ABS_PRESSURE, p);
	input_sync(t_dev->i_dev);
	return(count);

}
static DEVICE_ATTR(us, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_us, set_us);

#endif

/* Scanning: set to 1 scan various delay times */
static ssize_t show_scanning(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d: %d %d %d %d %d %d %d %d\n", lf1000_touch_dev->scanning,
		       scan_delay_lut[0], scan_delay_lut[1], scan_delay_lut[2], scan_delay_lut[3],
		       scan_delay_lut[4], scan_delay_lut[5], scan_delay_lut[6], scan_delay_lut[7]);
}
static ssize_t set_scanning(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp, temp2[SCAN_DELAY_LUT_SIZE], n, i;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	n = sscanf(buf, "%i %i %i %i %i %i %i %i %i", &temp,
		   &temp2[0], &temp2[1], &temp2[2], &temp2[3], 
		   &temp2[4], &temp2[5], &temp2[6], &temp2[7]);
	if (n<1 || (n>1 && n<9))
		return -EINVAL;
	if (n==1)
		lf1000_touch_dev->scanning = temp;
	if (n==9)
		for (i=0; i<SCAN_DELAY_LUT_SIZE; i++)
			scan_delay_lut[i] = temp2[i];
	lf1000_touch_dev->scan_ctr = 0;
	dev_dbg(dev, "%s.%s:%d scanning=%d\n", __FILE__, __FUNCTION__, 
			__LINE__, lf1000_touch_dev->scanning);
	return(count);
}
static DEVICE_ATTR(scanning, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_scanning, set_scanning);

/* TNT mode: 0-3 for which channel to read from */
static ssize_t show_tnt_mode(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->tnt_mode);
}

static ssize_t set_tnt_mode(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp, n;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	n = sscanf(buf, "%i", &temp);
	if (n<1 || temp<0 || temp>3)
		return -EINVAL;
	lf1000_touch_dev->tnt_mode = temp;
	return(count);
}
static DEVICE_ATTR(tnt_mode, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_tnt_mode, set_tnt_mode);

/* TNT mode: 0-3 for which channel to read from */
static ssize_t show_tnt_plane(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d %d %d\n", 
		       lf1000_touch_dev->tnt_plane[0],
		       lf1000_touch_dev->tnt_plane[1],
		       lf1000_touch_dev->tnt_plane[2]);
}

static ssize_t set_tnt_plane(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp[3], n;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	n = sscanf(buf, "%i %i %i", &temp[0], &temp[1], &temp[2]);
	if (n<3)
		return -EINVAL;
	lf1000_touch_dev->tnt_plane[0] = temp[0];
	lf1000_touch_dev->tnt_plane[1] = temp[1];
	lf1000_touch_dev->tnt_plane[2] = temp[2];
	return(count);
}
static DEVICE_ATTR(tnt_plane, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_tnt_plane, set_tnt_plane);

/* Averaging: set to >1 to average during read_xy */
static ssize_t show_averaging(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->averaging);
}
static ssize_t set_averaging(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->averaging = temp;
	dev_dbg(dev, "%s.%s:%d averaging=%d\n", __FILE__, __FUNCTION__, 
			__LINE__, lf1000_touch_dev->averaging);
	return(count);
}
static DEVICE_ATTR(averaging, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_averaging, set_averaging);

/* Enable Tails for compatibility */
static ssize_t show_tails(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->tails);
}
static ssize_t set_tails(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->tails = temp;
	dev_dbg(dev, "%s.%s:%d tails=%d\n", __FILE__, __FUNCTION__, 
			__LINE__, lf1000_touch_dev->tails);
	return(count);
}
static DEVICE_ATTR(tails, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_tails, set_tails);

/* pressure_curve: N_PCURVE_PTS points on a curve to translate computed 
   pressure to user pressure (0..100) in steps of 10% */
static ssize_t show_pressure_curve(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *t_dev = (struct touch *)dev_get_drvdata(dev);
	int i, x=0;
	for (i=0; i<N_PCURVE_PTS; i++)
		x += sprintf (buf+x, "%d ", t_dev->pressure_curve[i]);
	buf[x-1] = '\n'; /* Replace last ' ' with \n */
	return x;
}
static ssize_t set_pressure_curve(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int i, temp[N_PCURVE_PTS];
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	/* Sorry, I don't want to figure out a loop for this... */
	if(sscanf(buf, "%i %i %i %i %i %i %i %i %i", 
		  &temp[0], &temp[1], &temp[2],
		  &temp[3], &temp[4], &temp[5],
		  &temp[6], &temp[7], &temp[8]) != 9)
		return -EINVAL;
	for (i=0; i<N_PCURVE_PTS; i++)
		lf1000_touch_dev->pressure_curve[i] = temp[i];
	dev_dbg(dev, "%s.%s:%d pressure_curve=%d %d %d %d %d %d %d %d %d\n",
		__FILE__, __FUNCTION__, __LINE__, 
		lf1000_touch_dev->pressure_curve[0],
		lf1000_touch_dev->pressure_curve[1],
		lf1000_touch_dev->pressure_curve[2],
		lf1000_touch_dev->pressure_curve[3],
		lf1000_touch_dev->pressure_curve[4],
		lf1000_touch_dev->pressure_curve[5],
		lf1000_touch_dev->pressure_curve[6],
		lf1000_touch_dev->pressure_curve[7],
		lf1000_touch_dev->pressure_curve[8]);
	return(count);
}
static DEVICE_ATTR(pressure_curve, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_pressure_curve, set_pressure_curve);

static ssize_t show_min_x(struct device *dev, struct device_attribute *attr, char *buf) { return sprintf(buf, "%d\n", 1); }
static ssize_t show_max_x(struct device *dev, struct device_attribute *attr, char *buf) { return sprintf(buf, "%d\n", 1024); }
static ssize_t show_min_y(struct device *dev, struct device_attribute *attr, char *buf) { return sprintf(buf, "%d\n", 1); }
static ssize_t show_max_y(struct device *dev, struct device_attribute *attr, char *buf) { return sprintf(buf, "%d\n", 1024); }

static DEVICE_ATTR(min_x, S_IRUSR|S_IRGRP|S_IROTH, show_min_x, NULL);
static DEVICE_ATTR(max_x, S_IRUSR|S_IRGRP|S_IROTH, show_max_x, NULL);
static DEVICE_ATTR(min_y, S_IRUSR|S_IRGRP|S_IROTH, show_min_y, NULL);
static DEVICE_ATTR(max_y, S_IRUSR|S_IRGRP|S_IROTH, show_max_y, NULL);

static ssize_t show_tnt_delay_in_us(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->tnt_delay_in_us);
}
static ssize_t set_tnt_delay_in_us(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	if (temp < 0)
		return -EINVAL;
	lf1000_touch_dev->tnt_delay_in_us = temp;
	dev_dbg(dev, "%s.%s:%d tnt_delay_in_us=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->tnt_delay_in_us);
	return(count);
}

static DEVICE_ATTR(tnt_delay_in_us, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_tnt_delay_in_us, set_tnt_delay_in_us);

static ssize_t show_max_tnt_down(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->adc_max_tnt_down);
}

static ssize_t set_max_tnt_down(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;

	/* if value out of range then ignore it */
	if (temp < 0) {
		dev_err(dev, "%s: Invalid value %s\n",
			__FUNCTION__, buf);
		return (count);
	}

	lf1000_touch_dev->adc_max_tnt_down = temp;

	dev_dbg(dev, "%s.%s:%d max_delta_tnt=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->adc_max_tnt_down);
	return(count);
}

static DEVICE_ATTR(max_tnt_down, \
	S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, \
	show_max_tnt_down, set_max_tnt_down);


static ssize_t show_min_tnt_up(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->adc_min_tnt_up);
}

static ssize_t set_min_tnt_up(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;

	/* if value out of range then ignore it */
	if (temp < 0) {
		dev_err(dev, "%s: Invalid value %s\n",
			__FUNCTION__, buf);
		return (count);
	}

	lf1000_touch_dev->adc_min_tnt_up = temp;

	dev_dbg(dev, "%s.%s:%d max_delta_tnt=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->adc_min_tnt_up);
	return(count);
}

static DEVICE_ATTR(min_tnt_up, \
	S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, \
	show_min_tnt_up, set_min_tnt_up);


static ssize_t show_max_delta_tnt(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->adc_max_delta_tnt);
}

static ssize_t set_max_delta_tnt(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;

	/* if value out of range then ignore it */
	if (temp < 0) {
		dev_err(dev, "%s: Invalid value %s\n",
			__FUNCTION__, buf);
		return (count);
	}

	lf1000_touch_dev->adc_max_delta_tnt = temp;

	dev_dbg(dev, "%s.%s:%d max_delta_tnt=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->adc_max_delta_tnt);
	return(count);
}

static DEVICE_ATTR(max_delta_tnt, \
	S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, \
	show_max_delta_tnt, set_max_delta_tnt);


static ssize_t show_debounce_in_samples_down(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->debounce_in_samples_down);
}

static ssize_t set_debounce_in_samples_down(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;

	/* if value out of range then ignore it */
	if (temp < 0) 
	{
		dev_err(dev, "%s: Invalid value %s\n",
			__FUNCTION__, buf);
		return (count);
	}

	lf1000_touch_dev->debounce_in_samples_down = temp;
	lf1000_touch_dev->stylus_down_count = 0;	/* reset counter */

	dev_dbg(dev, "%s.%s:%d debounce_in_samples_down=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->debounce_in_samples_down);
	return(count);
}

static DEVICE_ATTR(debounce_in_samples_down, \
	S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, \
	show_debounce_in_samples_down, set_debounce_in_samples_down);

static ssize_t show_debounce_in_samples_up(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->debounce_in_samples_up);
}

static ssize_t set_debounce_in_samples_up(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;

	/* if value out of range then ignore it */
	if (temp < 0) 
	{
		dev_err(dev, "%s: Invalid value %s\n",
			__FUNCTION__, buf);
		return (count);
	}

	lf1000_touch_dev->debounce_in_samples_up = temp;
	lf1000_touch_dev->stylus_down_count = 0;	/* reset counter */

	dev_dbg(dev, "%s.%s:%d debounce_in_samples_up=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->debounce_in_samples_up);
	return(count);
}

static DEVICE_ATTR(debounce_in_samples_up, \
	S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, \
	show_debounce_in_samples_up, set_debounce_in_samples_up);


static ssize_t show_stylus_down_count(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->stylus_down_count);
}

static DEVICE_ATTR(stylus_down_count, \
	S_IRUSR|S_IRGRP|S_IROTH,
	show_stylus_down_count, NULL);

static ssize_t show_a0(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->a[0]);
}
static ssize_t set_a0(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->a[0] = temp;
	dev_dbg(dev, "%s.%s:%d a[0]=%d\n",
		__FILE__, __FUNCTION__, __LINE__, lf1000_touch_dev->a[0]);
	return(count);
}
static DEVICE_ATTR(a0, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_a0, set_a0);

static ssize_t show_a1(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->a[1]);
}

static ssize_t set_a1(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->a[1] = temp;
	dev_dbg(dev, "%s.%s:%d a[1]=%d\n",
		__FILE__, __FUNCTION__, __LINE__, lf1000_touch_dev->a[1]);
	return(count);
}

static DEVICE_ATTR(a1, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_a1, set_a1);

static ssize_t show_a2(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->a[2]);
}

static ssize_t set_a2(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->a[2] = temp;
	dev_dbg(dev, "%s.%s:%d a[2]=%d\n",
		__FILE__, __FUNCTION__, __LINE__, lf1000_touch_dev->a[2]);
	return(count);
}

static DEVICE_ATTR(a2, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_a2, set_a2);


static ssize_t show_a3(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->a[3]);
}
static ssize_t set_a3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->a[3] = temp;
	dev_dbg(dev, "%s.%s:%d a[3]=%d\n",
		__FILE__, __FUNCTION__, __LINE__, lf1000_touch_dev->a[3]);
	return(count);
}

static DEVICE_ATTR(a3, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_a3, set_a3);

static ssize_t show_a4(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->a[4]);
}
static ssize_t set_a4(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->a[4] = temp;
	dev_dbg(dev, "%s.%s:%d a[4]=%d\n",
		__FILE__, __FUNCTION__, __LINE__, lf1000_touch_dev->a[4]);
	return(count);
}

static DEVICE_ATTR(a4, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_a4, set_a4);

static ssize_t show_a5(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->a[5]);
}
static ssize_t set_a5(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->a[5] = temp;
	dev_dbg(dev, "%s.%s:%d a[5]=%d\n",
		__FILE__, __FUNCTION__, __LINE__, lf1000_touch_dev->a[5]);
	return(count);
}

static DEVICE_ATTR(a5, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_a5, set_a5);

static ssize_t show_a6(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->a[6]);
}
static ssize_t set_a6(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	lf1000_touch_dev->a[6] = temp;
	dev_dbg(dev, "%s.%s:%d a[6]=%d\n",
		__FILE__, __FUNCTION__, __LINE__, lf1000_touch_dev->a[6]);
	return(count);
}

static DEVICE_ATTR(a6, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_a6, set_a6);

static ssize_t show_sample_rate_in_hz(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int rate;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	rate = lf1000_touch_dev->sample_rate_in_jiffies;
	rate = (0 < rate) ? (HZ / rate) : 0 ;	// expect positive value
	return sprintf(buf, "%d\n", rate);
}
static ssize_t set_sample_rate_in_hz(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%u", &temp) != 1)
		return -EINVAL;
	temp = (0 < temp )   ? temp : 1;  // set min rate to 1
	temp = (temp <= HZ ) ? temp : HZ; // set max rate to HZ
	lf1000_touch_dev->sample_rate_in_jiffies = (HZ / temp); 
	dev_dbg(dev,  "%s.%s:%d sample_rate_in_jiffies=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->sample_rate_in_jiffies);
	return(count);
}

static DEVICE_ATTR(sample_rate_in_hz, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_sample_rate_in_hz, set_sample_rate_in_hz);

static ssize_t show_report_events(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", lf1000_touch_dev->report_events);
}
static ssize_t set_report_events(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int temp;
	struct touch *lf1000_touch_dev = (struct touch *)dev_get_drvdata(dev);
	if(sscanf(buf, "%i", &temp) != 1)
		return -EINVAL;
	//	if (temp < 0 || temp > 7)	// ensure value between 0 and 7
	//		return -EINVAL;
	lf1000_touch_dev->report_events = temp;
	dev_dbg(dev, "%s.%s:%d report_events=%d\n",
		__FILE__, __FUNCTION__, __LINE__,
		lf1000_touch_dev->report_events);
	return(count);
}

static DEVICE_ATTR(report_events, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH, show_report_events, set_report_events);

static struct attribute *touchscreen_attributes[] = {
	&dev_attr_name.attr,
	&dev_attr_pointercal.attr,
	&dev_attr_x0.attr,
	&dev_attr_y1.attr,
	&dev_attr_touch_x.attr,
	&dev_attr_touch_y.attr,
	&dev_attr_touch_pressure.attr,
	&dev_attr_touch_button.attr,
	&dev_attr_screen_x.attr,
	&dev_attr_screen_y.attr,
	&dev_attr_raw_adc.attr,
	&dev_attr_delay_in_us.attr,
	&dev_attr_y_delay_in_us.attr,
	&dev_attr_averaging.attr,
	&dev_attr_tails.attr,
	&dev_attr_pressure_curve.attr,
	&dev_attr_scanning.attr,
	&dev_attr_tnt_mode.attr,
	&dev_attr_tnt_plane.attr,
	&dev_attr_min_x.attr,
	&dev_attr_max_x.attr,
	&dev_attr_min_y.attr,
	&dev_attr_max_y.attr,
	&dev_attr_tnt_delay_in_us.attr,
	&dev_attr_max_tnt_down.attr,
	&dev_attr_min_tnt_up.attr,
	&dev_attr_max_delta_tnt.attr,
	&dev_attr_debounce_in_samples_down.attr,
	&dev_attr_debounce_in_samples_up.attr,
	&dev_attr_stylus_down_count.attr,
	&dev_attr_a0.attr,
	&dev_attr_a1.attr,
	&dev_attr_a2.attr,
	&dev_attr_a3.attr,
	&dev_attr_a4.attr,
	&dev_attr_a5.attr,
	&dev_attr_a6.attr,
	&dev_attr_sample_rate_in_hz.attr,
	&dev_attr_report_events.attr,
#ifdef TS_TIMING
	&dev_attr_us_in_driver.attr,
	&dev_attr_us.attr,
#endif
	NULL
};

static struct attribute_group touchscreen_attr_group = {
	.attrs = touchscreen_attributes
};

int adc_GetMedian3Reading(int channel)
{
	int a, b, c;
	a = adc_GetReading(channel);
	b = adc_GetReading(channel);
	c = adc_GetReading(channel);
	if (a<b)
	{
		// a X Y   or   c a b
		if (a<c)
			return b<c ? b : c;
		else
			// 2 0 1
			return a;
	}
	else
	{
		// b X Y   or   c b a
		if (b<c)
			return a<c ? a : c;
		else
			return b;
	}
}

int adc_GetMedian4Reading(int channel)
{
	int a, b, c, d, min, max;
	a = adc_GetReading(channel);
	b = adc_GetReading(channel);
	c = adc_GetReading(channel);
	d = adc_GetReading(channel);
	min = a;
	if (min>b) min=b;
	if (min>c) min=c;
	if (min>d) min=d;
	max = a;
	if (max<b) max=b;
	if (max<c) max=c;
	if (max<d) max=d;
	return (a+b+c+d-min-max)/2;
}

#define adc_GetMedianReading(channel)  adc_GetMedian3Reading(channel)

void delay_in_us (void)
{
	if (t_dev->scanning)
		udelay (scan_delay_lut[t_dev->scan_ctr & (SCAN_DELAY_LUT_SIZE-1)]);
	else
		udelay (t_dev->delay_in_us);
}

void y_delay_in_us (void)
{
	if (t_dev->scanning)
		udelay (scan_delay_lut[t_dev->scan_ctr & (SCAN_DELAY_LUT_SIZE-1)]);
	else
		udelay (t_dev->y_delay_in_us);
}

void tnt_delay_in_us (void)
{
	if (t_dev->scanning)
		udelay (scan_delay_lut[t_dev->scan_ctr & (SCAN_DELAY_LUT_SIZE-1)]);
	else
		udelay (t_dev->tnt_delay_in_us);
}

/* experimental version suggested by Rob (from Sam)
 * configure one sheet's terminals as inputs with pullup
 * configure the other sheet's terminals as outputs with level 0.
 * When untouched, the value on the inputs ought to be high.
 * When touched, the value on the inputs will be low.
 *               
 * There seems to be a big difference in voltages between touched and untouched.
 * There's a smaller variation of voltages when touched.  The harder the touch,
 * the lower the voltage.
 */
void set_read_tnt(void) // touch / no-touch
{
	switch (t_dev->tnt_mode)
	{
	case 0:
	case 2:
		// Input on X with pullup, Drive Y low
		// gpio_configure_pin(PORT, PIN, FUNCTION, OUT=1, PULLUP=1, VALUE)
		gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X1),
				   lf1000_l2p_pin(TOUCHSCREEN_X1), GPIO_GPIOFN, 0, 1, 0);
		gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X2),
				   lf1000_l2p_pin(TOUCHSCREEN_X2), GPIO_GPIOFN, 0, 1, 0);
		gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y1),
				   lf1000_l2p_pin(TOUCHSCREEN_Y1), GPIO_GPIOFN, 1, 0, 0);
		gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y2),
				   lf1000_l2p_pin(TOUCHSCREEN_Y2), GPIO_GPIOFN, 1, 0, 0);
		break;
	case 1:
	case 3:
		// Input on Y with pullup, Drive X low
		gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y1),
				   lf1000_l2p_pin(TOUCHSCREEN_Y1), GPIO_GPIOFN, 0, 1, 0);
		gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y2),
				   lf1000_l2p_pin(TOUCHSCREEN_Y2), GPIO_GPIOFN, 0, 1, 0);
		gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X1),
				   lf1000_l2p_pin(TOUCHSCREEN_X1), GPIO_GPIOFN, 1, 0, 0);
		gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X2),
				   lf1000_l2p_pin(TOUCHSCREEN_X2), GPIO_GPIOFN, 1, 0, 0);
	}
}
static void read_first_tnt(void)
{

	if (t_dev->averaging < 0)
		t_dev->adc_tnt1 = adc_GetMedianReading(t_dev->first_adc+t_dev->tnt_mode);
	else
		t_dev->adc_tnt1 = adc_GetReading(t_dev->first_adc+t_dev->tnt_mode);
}
static void read_second_tnt(void)
{
	if (t_dev->averaging < 0)
		t_dev->adc_tnt2 = adc_GetMedianReading(t_dev->first_adc+t_dev->tnt_mode);
	else
		t_dev->adc_tnt2 = adc_GetReading(t_dev->first_adc+t_dev->tnt_mode);
}
/* NOTE: there is no delay before the first reading of tnt1, because the gpios
 *       have been properly configured since tnt2 was read at the end of the
 * previous sampling period.
 */
static void first_reading_tnt(void)
{
    // This assumes the gpio configuration remains from previous reading,
    // which occurs in the _probe() routine or in second_reading_tnt().
    read_first_tnt();
}
/* NOTE: experiments indicated it's necessary to delay longer for the gpio
 *       voltages to settle before reading tnt2.  Apparently this is due
 * to differences in the resistors.
 */
static void second_reading_tnt(void)
{
	set_read_tnt();
	tnt_delay_in_us ();	// Let the screen settle and charge up
	read_second_tnt();
}


void set_read_p12(void)
{
	// 15sep10  Experimental version suggested by Rob and Sam
        //          don't use pull-ups

	// X1 = 1; Y2 = 0;  X2 and Y1 inputs
	// gpio_configure_pin(PORT, PIN, FUNCTION, OUT=1, PULLUP=1, VALUE)
	// Set X1 as input with pullup
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X2),
		                lf1000_l2p_pin(TOUCHSCREEN_X2), GPIO_GPIOFN, 0, 0, 0);
	// Set Y1 as input with pullup
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y1),
		                lf1000_l2p_pin(TOUCHSCREEN_Y1), GPIO_GPIOFN, 0, 0, 0);
	// Set X1 as output high
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X1),
		                lf1000_l2p_pin(TOUCHSCREEN_X1), GPIO_GPIOFN, 1, 0, 1);
	// Set Y2 as output low
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y2),
		                lf1000_l2p_pin(TOUCHSCREEN_Y2), GPIO_GPIOFN, 1, 0, 0);
}


/* Configure the gpios for reading the x-position voltage */
void set_read_x(void)
{
	// float X1 and X2 first, then drive Y1 high and Y2 low
	// gpio_configure_pin(PORT, PIN, FUNCTION, OUT=1, PULLUP=1, VALUE)
	// Set X1 and X2 as inputs with or without pullup, based on "tails" sysfs.
	// With "tails" on, you will get tails!  But it won't require recalibrating your device.
	int PU = t_dev->tails ? 1 : 0;
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X1),
                	    lf1000_l2p_pin(TOUCHSCREEN_X1), GPIO_GPIOFN, 0, PU, 0);
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X2),
                	    lf1000_l2p_pin(TOUCHSCREEN_X2), GPIO_GPIOFN, 0, PU, 0);
	// Set Y1 as output high
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y1),
                   	    lf1000_l2p_pin(TOUCHSCREEN_Y1), GPIO_GPIOFN, 1, 0, 1);
	// Set Y2 as output low
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y2),
                	    lf1000_l2p_pin(TOUCHSCREEN_Y2), GPIO_GPIOFN, 1, 0, 0);
}

/* Configure the gpios for reading the y-position voltage */
void set_read_y(void)
{
	// float Y1 and Y2 first, then drive X1 high and X2 low
	// gpio_configure_pin(PORT, PIN, FUNCTION, OUT=1, PULLUP=1, VALUE)
	// Set Y1 and Y2 as inputs with or without pullup, based on "tails" sysfs.
	// With "tails" on, you will get tails!  But it won't require recalibrating your device.
	int PU = t_dev->tails ? 1 : 0;
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y1),
                	    lf1000_l2p_pin(TOUCHSCREEN_Y1), GPIO_GPIOFN, 0, PU, 0);
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_Y2),
                	    lf1000_l2p_pin(TOUCHSCREEN_Y2), GPIO_GPIOFN, 0, PU, 0);
	// Set X1 as output high
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X1),
                	    lf1000_l2p_pin(TOUCHSCREEN_X1), GPIO_GPIOFN, 1, 0, 1);
	// Set X2 as output low
	gpio_configure_pin(lf1000_l2p_port(TOUCHSCREEN_X2),
                	    lf1000_l2p_pin(TOUCHSCREEN_X2), GPIO_GPIOFN, 1, 0, 0);
}


/*
 * int read_xy(void)
 * returns RXY_UP:         Stylus UP, 
 *         RXY_DOWN:       Stylus DOWN, have ADC values.
 *         RXY_IN_BETWEEN: Stylus neither UP nor DOWN; have ADC values
 *
 * Reads adc to determine touch/no-touch (tnt).
 * If no-touch, immediately returns RXY_UP.
 * Otherwise reads X and Y touchscreen values.
 * Reads voltages to calculate touch pressure.
 * Then reads touch/no-touch again.
 * If second tnt reading indicates no-touch, returns RXY_UP.
 *
 * If both tnt readings are <= t_dev->adc_max_tnt_down and are close to each
 * other (differ by no more than t_dev->adc_max_delta_tnt), returns RXY_DOWN.  
 * If both tnt readings are <= t_dev->adc_max_tnt_down and differ by more
 * than t_dev->adc_max_delta_tnt, returns RXY_IN_BETWEEN.  
 *
 * Also returns RXY_IN_BETWEEN if both tnt readings are <= t_dev->adc_min_tnt_up
 * and at least one of them is > t_dev->adc_max_tnt_down.
 */

#define MAX_TNT_DOWN    19      // Down if all tnt are <= this
#define MIN_TNT_UP      520     // UP if any tnt is > this
#define MAX_DELTA_TNT   5		// 

#define RXY_UP          0
#define RXY_DOWN        1
#define RXY_IN_BETWEEN  2

static int read_xy(void)
{
	int x, y, fancy_tnt;

	first_reading_tnt();
	if (t_dev->adc_tnt1 > t_dev->adc_min_tnt_up) {
		t_dev->adc_tnt2 = 1024; // indicate unread (to prevent lots of output)
		/* Don't return early if we're getting raw reports */
		if (!(t_dev->report_events & REMASK_RAW_REPORT))
			return RXY_UP;          // UP unless both tnt readings are low enough
	}
	/*
	 * now read voltages for x
	 */
	set_read_x();			    // set GPIO pins to read X value
	delay_in_us ();	// Let the screen settle and charge up
	if (t_dev->averaging > 1)
	{
		int s1=0, s2=0, i;
		for (i=0; i<t_dev->averaging; i++)
		{
			s1 += adc_GetReading(t_dev->first_adc);     // read X1 ADC
			s2 += adc_GetReading(t_dev->first_adc + 2); // read X2 ADC
		}
		t_dev->adc_x  = s1 / t_dev->averaging;
		t_dev->adc_x2 = s2 / t_dev->averaging;
	}
	else if (t_dev->averaging < 0)
	{
		t_dev->adc_x  = adc_GetMedianReading(t_dev->first_adc);     // read X1 ADC
		t_dev->adc_x2 = adc_GetMedianReading(t_dev->first_adc + 2); // read X2 ADC
	}
	else
	{
		t_dev->adc_x  = adc_GetReading(t_dev->first_adc);     // read X1 ADC
		t_dev->adc_x2 = adc_GetReading(t_dev->first_adc + 2); // read X2 ADC
	}

	/*
	 * read voltages for y
	 */
	set_read_y();			    // set GPIO pins to read Y value
	y_delay_in_us ();	// Let the screen settle and charge up
	if (t_dev->averaging > 1)
	{
		int s1=0, s2=0, i;
		for (i=0; i<t_dev->averaging; i++)
		{
			s1 += adc_GetReading(t_dev->first_adc + 1); // read Y1 ADC
			s2 += adc_GetReading(t_dev->first_adc + 3); // read Y2 ADC

		}
		t_dev->adc_y  = s1 / t_dev->averaging;
		t_dev->adc_y2 = s2 / t_dev->averaging;
	}
	else if (t_dev->averaging < 0)
	{
		t_dev->adc_y  = adc_GetMedianReading(t_dev->first_adc + 1); // read Y1 ADC
		t_dev->adc_y2 = adc_GetMedianReading(t_dev->first_adc + 3); // read Y2 ADC
	}
	else
	{
		t_dev->adc_y  = adc_GetReading(t_dev->first_adc + 1); // read Y1 ADC
		t_dev->adc_y2 = adc_GetReading(t_dev->first_adc + 3); // read Y2 ADC
	}

	/*
	 * read voltages for pressure
	 */
	set_read_p12();
	delay_in_us ();	// Let the screen settle and charge up
	if (t_dev->averaging > 1)
	{
		int s1=0, s2=0, i;
		for (i=0; i<t_dev->averaging; i++)
		{
			s1 += adc_GetReading(t_dev->first_adc+2);       // read X2 ADC
			s2 += adc_GetReading(t_dev->first_adc+1);       // read Y1 ADC
		}
		t_dev->adc_p1 = s1 / t_dev->averaging;
		t_dev->adc_p2 = s2 / t_dev->averaging;
	}
	else if (t_dev->averaging < 0)
	{
		t_dev->adc_p1 = adc_GetMedianReading(t_dev->first_adc+2);       // read X2 ADC
		t_dev->adc_p2 = adc_GetMedianReading(t_dev->first_adc+1);       // read Y1 ADC
	}
	else
	{
		t_dev->adc_p1 = adc_GetReading(t_dev->first_adc+2);       // read X2 ADC
		t_dev->adc_p2 = adc_GetReading(t_dev->first_adc+1);       // read Y1 ADC
	}

	/* second touch/no-touch reading */
	second_reading_tnt();

	if (t_dev->adc_tnt1 + t_dev->adc_tnt2 > 2*t_dev->adc_min_tnt_up)
		return RXY_UP;   // UP unless both tnt readings are low enough

#if 1
	/* Compute a fancy tnt threshold based on position */
	x = (t_dev->adc_x + t_dev->adc_x2)/2;
	y = (t_dev->adc_y + t_dev->adc_y2)/2;
	fancy_tnt = t_dev->adc_max_tnt_down +
		(t_dev->tnt_plane[0] * x +
		 t_dev->tnt_plane[1] * y +
		 t_dev->tnt_plane[2] + 32768)/65536;
	if (t_dev->adc_tnt1 + t_dev->adc_tnt2 <= 2*fancy_tnt)
#else
	if (t_dev->adc_tnt1 + t_dev->adc_tnt2 <= 2*t_dev->adc_max_tnt_down)
#endif
	{
		// both are low enough for Down; check for stability
		// if unstable, say going down or going up
		if (abs(t_dev->adc_tnt1 - t_dev->adc_tnt2) > t_dev->adc_max_delta_tnt)
		{
			return RXY_IN_BETWEEN;
		}
		else return RXY_DOWN;  // if both low and stable, say Down
	}
        // else both are <= t_dev->adc_min_tnt_up and at least one is greater
        // than t_dev->adc_max_tnt_down.
        // Seems to be in transition
	else {
		return RXY_IN_BETWEEN;
	}
}


/* State transitions and debouncing
 *
 * If read_xy() returns:
 *      0 (up) - increment stylus_up_count;
 *               if (stylus_up_count >= debounce_up)
 *                  if (stylus_up_count == debounce_up) {
 *                      state = TSTATE_UP
 *                      report stylus up
 *                  }
 *               else
 *                  state = TSTATE_GOING_UP
 *                  ignore the touch info
 *
 *      1 (down) - increment stylus_down_count;
 *               if (stylus_down_count >= debounce_down)
 *                  if (state != TSTATE_DOWN {
 *                      state = TSTATE_DOWN
 *                      report stylus down
 *                      stylus_up_count = 0;
 *                  }
 *                  process the touch point
 *               else
 *                  state = TSTATE_GOING_DOWN
 *                  ignore the touch info
 *
 *      2 (going up)
 *                  state = TSTATE_GOING_UP
 *                  ignore the touch info
 *
 *      3 (going down)
 *                  state = TSTATE_GOING_DOWN
 *                  ignore the touch info
 */


static void process_down(int report_events)
{
	if (++t_dev->stylus_down_count >= t_dev->debounce_in_samples_down) {
		t_dev->ts_state = TSTATE_DOWN;
		t_dev->stylus_up_count = 0;
		if (report_events & REMASK_REPORT) /* say stylus down */
		{
			input_report_key(t_dev->i_dev, BTN_TOUCH, 1);
		}
	}
	else
		t_dev->ts_state = TSTATE_GOING_DOWN;
}

static void process_up(int report_events)
{
	if (++t_dev->stylus_up_count >= t_dev->debounce_in_samples_up) {
		t_dev->ts_state = TSTATE_UP;
		t_dev->stylus_down_count = 0;
		if (report_events & REMASK_REPORT) /* say stylus up */
		{
			input_report_key(t_dev->i_dev, BTN_TOUCH, 0);
			// Send a 0 pressure event for TSLIB
			input_report_abs(t_dev->i_dev, ABS_X, t_dev->screen_x);
			input_report_abs(t_dev->i_dev, ABS_Y, t_dev->screen_y);
			input_report_abs(t_dev->i_dev, ABS_PRESSURE, 0);
			input_sync(t_dev->i_dev);
		}

	}
	else
		t_dev->ts_state = TSTATE_GOING_UP;
}

/*
 * void get_touch(struct work_struct work)
 * read touchscreen, debounce stylus up/down, and report screen activity.
 */

static void get_touch(struct work_struct *work)
{
	int report_events = t_dev->report_events;
	static int counter=0;

	t_dev->scan_ctr++;

#ifdef TS_TIMING
	if (!timing) timer_start();
#endif
	t_dev->touch_button = read_xy();

	switch (t_dev->ts_state) {
	case TSTATE_UP:
		if (t_dev->touch_button == RXY_DOWN)
			process_down(report_events);
		else if (t_dev->touch_button == RXY_IN_BETWEEN)
			t_dev->ts_state = TSTATE_GOING_DOWN;
		break;

	case TSTATE_GOING_DOWN:
		if (t_dev->touch_button == RXY_UP) {
			t_dev->ts_state = TSTATE_UP;
		}
		else if (t_dev->touch_button == RXY_DOWN)
			process_down(report_events);
		break;

	case TSTATE_DOWN:
		if (t_dev->touch_button == RXY_IN_BETWEEN)
			t_dev->ts_state = TSTATE_GOING_UP;
		else if (t_dev->touch_button == RXY_UP)
			process_up(report_events);
		break;

	case TSTATE_GOING_UP:
		if (t_dev->touch_button == RXY_DOWN)
			t_dev->ts_state = TSTATE_DOWN;
		else if (t_dev->touch_button == RXY_UP)
			process_up(report_events);
		break;
	}
	if (t_dev->ts_state == TSTATE_DOWN) {
		/* average the x1 and 2 readings; ditto with y1,2 */
		t_dev->touch_x = (t_dev->adc_x + t_dev->adc_x2)/2;
		t_dev->touch_y = (t_dev->adc_y + t_dev->adc_y2)/2;

		/* use calibration data to convert from adc reading to
                 * screen position.
                 */
		t_dev->screen_x = (t_dev->a[1] * t_dev->touch_x +
				   t_dev->a[2] * t_dev->touch_y +
				   t_dev->a[0]) / t_dev->a[6];
		t_dev->screen_y = (t_dev->a[4] * t_dev->touch_x +
				   t_dev->a[5] * t_dev->touch_y +
				   t_dev->a[3]) / t_dev->a[6];
		if (t_dev->adc_p2 == 0) {
			t_dev->adc_pressure = 0;
			t_dev->touch_pressure = 0;
		}
		else {
			int i;
			t_dev->adc_pressure = t_dev->touch_x
				* ((1000 * t_dev->adc_p1)/t_dev->adc_p2 - 1000)
				/ 1000;
		/* Convert computed pressure to linear pressure with piecewise curve */
			t_dev->touch_pressure=10;
			for (i=0; i<N_PCURVE_PTS; i++)
			{
				if (t_dev->adc_pressure < t_dev->pressure_curve[i])
				{
					t_dev->touch_pressure = 100-i*10;
					break;
				}
			}
		}

		if (report_events & REMASK_REPORT) {
			input_report_abs(t_dev->i_dev, ABS_X, t_dev->screen_x);
			input_report_abs(t_dev->i_dev, ABS_Y, t_dev->screen_y);
			input_report_abs(t_dev->i_dev, ABS_PRESSURE, t_dev->touch_pressure);
			input_sync(t_dev->i_dev);
		}
	}

	if (report_events & REMASK_RAW_REPORT) {
		input_report_abs(t_dev->iraw_dev, 0x0, t_dev->adc_x);
		input_report_abs(t_dev->iraw_dev, 0x1, t_dev->adc_x2);
		input_report_abs(t_dev->iraw_dev, 0x2, t_dev->adc_y);
		input_report_abs(t_dev->iraw_dev, 0x3, t_dev->adc_y2);
		input_report_abs(t_dev->iraw_dev, 0x4, t_dev->adc_p1);
		input_report_abs(t_dev->iraw_dev, 0x5, t_dev->adc_p2);
		input_report_abs(t_dev->iraw_dev, 0x6, t_dev->adc_tnt1);
		input_report_abs(t_dev->iraw_dev, 0x7, t_dev->adc_tnt2);
		input_report_abs(t_dev->iraw_dev, 0x8, t_dev->adc_pressure);
		input_report_abs(t_dev->iraw_dev, 0x9, 
				 t_dev->scanning ? 
				 scan_delay_lut[t_dev->scan_ctr & (SCAN_DELAY_LUT_SIZE-1)] : 
				 t_dev->delay_in_us);
		// show life to keep events flowing
		input_report_abs(t_dev->iraw_dev, 0xa, counter++);
		input_sync(t_dev->iraw_dev);
	}

	// Rob Debug--No one was using REMASK_ADC_DBG, so I will.
	// To enable, echo 3 or 9+100*N (instead of 1) >
	// /sys/devices/platform/lf1000-touchscreen/report_events
	if (report_events & (REMASK_ADC_DBG | REMASK_ADC_DBG_MODE))
	{
		static char *lut = "DV^U";
		static int y=0;
		int do_report = 0;

		if (report_events & REMASK_ADC_DBG_MODE)
		{
			// Report N times, decrementing the
			// upper bits in report_events, until 0
			y = report_events >> RESHIFT_COUNT;
			if (y > 0)
			{
				y--;
				t_dev->report_events = (y << RESHIFT_COUNT) |
					(report_events & ((1<<RESHIFT_COUNT)-1));
				do_report = 1;
			}
		}
		else
		{
			// Report once every second
			y=(y+1)%((HZ)/t_dev->sample_rate_in_jiffies);
			if (y==0)
				do_report = 1;

		}
		if (do_report)
		{
			static int lastx, lasty;
			if ((report_events & REMASK_ADC_DBG) &&
			    (report_events & REMASK_ADC_DBG_MODE))
				printk (KERN_ALERT // Super report! 10
					"x1=%4d x2=%4d y1=%4d y2=%4d p1=%4d p2=%4d tnt1=%4d tnt2=%4d P=%4d\n",
					t_dev->adc_x, t_dev->adc_x2,
					t_dev->adc_y, t_dev->adc_y2,
					t_dev->adc_p1, t_dev->adc_p2,
					t_dev->adc_tnt1, t_dev->adc_tnt2,
					t_dev->adc_pressure);
					/* "x1=%4d x2=%4d y1=%4d y2=%4d p1=%4d p2=%4d tnt1=%4d tnt2=%4d S=%c X=%3d Y=%3d P=%4d LP=%3d %4d,%4d\n", */
					/* t_dev->adc_x, t_dev->adc_x2, */
					/* t_dev->adc_y, t_dev->adc_y2, */
					/* t_dev->adc_p1, t_dev->adc_p2, */
					/* t_dev->adc_tnt1, t_dev->adc_tnt2, */
					/* lut[t_dev->ts_state], */
					/* t_dev->screen_x, t_dev->screen_y, */
					/* t_dev->adc_pressure, */
					/* t_dev->touch_pressure, */
					/* t_dev->screen_x-lastx, t_dev->screen_y-lasty); */
			else
				printk (KERN_ALERT // Regular report: 2
					"x1=%4d x2=%4d y1=%4d y2=%4d p1=%4d p2=%4d tnt1=%4d tnt2=%4d S=%c X=%d Y=%d P=%d LP=%d\n",
					t_dev->adc_x, t_dev->adc_x2,
					t_dev->adc_y, t_dev->adc_y2,
					t_dev->adc_p1, t_dev->adc_p2,
					t_dev->adc_tnt1, t_dev->adc_tnt2,
					lut[t_dev->ts_state],
					t_dev->screen_x, t_dev->screen_y,
					t_dev->adc_pressure,
					t_dev->touch_pressure);
			lastx=t_dev->screen_x;
			lasty=t_dev->screen_y;
		}
	}
#ifdef TS_TIMING
	if (t_dev->N_us++)
	{
		u32 t = timing ? 1 : timer_stop();
		t_dev->sum_us += t;
		if (t_dev->max_us < t) t_dev->max_us = t;
		if (t_dev->min_us > t) t_dev->min_us = t;
	}
	else
		t_dev->sum_us = t_dev->max_us = t_dev->min_us = 
			timing ? 1 : timer_stop();
#endif
}

void touchscreen_monitor_task(unsigned long data)
{
	struct touch *t_dev = (struct touch *)data;

	if (!t_dev->stop_timer) {	// not stopping, reload timer
		queue_work(t_dev->touchscreen_tasks, &t_dev->touchscreen_work);
		t_dev->touchscreen_timer.expires += t_dev->sample_rate_in_jiffies;
		t_dev->touchscreen_timer.function = touchscreen_monitor_task;
		t_dev->touchscreen_timer.data = data;
		add_timer(&t_dev->touchscreen_timer);
	}
}

int touchscreen_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,
                                          unsigned long arg)
{
	return 0;
}
	
struct file_operations touchscreen_fops = {
	.owner = THIS_MODULE,
	.ioctl = touchscreen_ioctl,
};

static int lf1000_ts2_probe(struct platform_device *pdev)
{
	struct input_dev *i_dev, *iraw_dev;
	int error, i;

	i_dev = input_allocate_device();

	if (!i_dev)  {
		error = -ENOMEM;
		goto err_free_devs;
	}

	iraw_dev = input_allocate_device();
	if (!iraw_dev)  {
		error = -ENOMEM;
		goto err_free_devs;
	}

	i_dev->name		= "LF1000 touchscreen interface";
	i_dev->phys		= "lf1000/touchscreen";
	i_dev->id.bustype	= BUS_HOST;
	i_dev->id.vendor	= 0x0001;
	i_dev->id.product	= 0x0001;
	i_dev->id.version	= 0x0001;
	t_dev->i_dev		= i_dev;

	iraw_dev->name		= "LF1000 touchscreen raw";
	iraw_dev->phys		= "lf1000/touchscreen-raw";
	iraw_dev->id.bustype	= BUS_HOST;
	iraw_dev->id.vendor	= 0x0001;
	iraw_dev->id.product	= 0x0001;
	iraw_dev->id.version	= 0x0001;
	t_dev->iraw_dev		= iraw_dev;

	t_dev->ts_state          = TSTATE_UP;

	t_dev->adc_tnt1          = 1024;
	t_dev->adc_tnt2          = 1024;
	t_dev->adc_max_tnt_down  = MAX_TNT_DOWN;
	t_dev->adc_min_tnt_up    = MIN_TNT_UP;
	t_dev->adc_max_delta_tnt = MAX_DELTA_TNT;

	t_dev->averaging	 = -1; /* Median mode instead of avg */
	t_dev->tails		 = 0; /* Not compatible with V3 calib */
	t_dev->scanning		 = 0;
	t_dev->tnt_mode		 = gpio_have_gpio_madrid() ? 1 : 0;

	t_dev->tnt_plane[0]	 = 0; /* All 3 0's means no plane offset */
	t_dev->tnt_plane[1]	 = 0;
	t_dev->tnt_plane[2]	 = 0;

	t_dev->delay_in_us	= TS_DELAY_IN_US;
	t_dev->y_delay_in_us	= TS_Y_DELAY_IN_US;
	t_dev->tnt_delay_in_us	= TNT_DELAY_IN_US;

	t_dev->pressure_curve[0] = 468;
	t_dev->pressure_curve[1] = 471;
	t_dev->pressure_curve[2] = 487;
	t_dev->pressure_curve[3] = 495;
	t_dev->pressure_curve[4] = 509;
	t_dev->pressure_curve[5] = 524;
	t_dev->pressure_curve[6] = 553;
	t_dev->pressure_curve[7] = 575;
	t_dev->pressure_curve[8] = 612;

	/* set GPIOs to max output current */
	gpio_set_cur(lf1000_l2p_port(TOUCHSCREEN_X1),
		      lf1000_l2p_pin(TOUCHSCREEN_X1), GPIO_CURRENT_8MA);
	gpio_set_cur(lf1000_l2p_port(TOUCHSCREEN_Y1),
		      lf1000_l2p_pin(TOUCHSCREEN_Y1), GPIO_CURRENT_8MA);
	gpio_set_cur(lf1000_l2p_port(TOUCHSCREEN_X2),
		      lf1000_l2p_pin(TOUCHSCREEN_X2), GPIO_CURRENT_8MA);
	gpio_set_cur(lf1000_l2p_port(TOUCHSCREEN_Y2),
		      lf1000_l2p_pin(TOUCHSCREEN_Y2), GPIO_CURRENT_8MA);

	/*
 	 * set first ADC channel hooked to touchscreen
 	 * Didj TS uses ADC[0] to ADC[3]
 	 * Leapster 3 uses ADC[4] to ADC[7]
 	 */

	if (gpio_have_gpio_dev() || gpio_have_gpio_didj())
		t_dev->first_adc = 0;
	else if (gpio_have_gpio_acorn() || gpio_have_gpio_emerald())
		t_dev->first_adc = 4;
	else {
		t_dev->first_adc = 4;
		dev_dbg(&t_dev->i_dev->dev,
				"%s.%s:%d unknown touchscreen interface",
				__FILE__, __FUNCTION__, __LINE__);
	}

	/* event types that we support */	
	i_dev->evbit[0]			   = BIT(EV_KEY) | BIT(EV_ABS);
	i_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	/* initial screen coordination      min=[0]   max=[1]  fuzz=[2]  */
	input_set_abs_params(i_dev, ABS_X, abs_x[0], abs_x[1], abs_x[2], 0);
	input_set_abs_params(i_dev, ABS_Y, abs_y[0], abs_y[1], abs_y[2], 0);
	input_set_abs_params(i_dev, ABS_PRESSURE, abs_p[0], abs_p[1], abs_p[2], 0);
	platform_set_drvdata(pdev, t_dev);

	error = input_register_device(i_dev);
	if(error)
		goto err_free_devs;

	/* event types that we support */	
	iraw_dev->evbit[0]			   = BIT(EV_ABS);

	/* initial screen coordination      min=[0]   max=[1]  fuzz=[2]  */
	for (i=0x0; i<RAW_ABS_PARAMS; i++)
		input_set_abs_params(iraw_dev, i, 0, 65535, 0, 0);

	error = input_register_device(iraw_dev);
	if(error)
		goto err_free_devs;

	/* setup work queue */
	t_dev->touchscreen_tasks = create_singlethread_workqueue("touchscreen"
								 " tasks");
	INIT_WORK(&t_dev->touchscreen_work, get_touch);

	/* Do the initial configuration for reading tnt */
	set_read_tnt();
	tnt_delay_in_us ();	// Let the screen settle and charge up

	/* grab initial touchscreen settings */
	read_xy();

	/* set default /etc/pointercal values */
	t_dev->a[0] = TS_A0;
	t_dev->a[1] = TS_A1;
	t_dev->a[2] = TS_A2;
	t_dev->a[3] = TS_A3;
	t_dev->a[4] = TS_A4;
	t_dev->a[5] = TS_A5;
	t_dev->a[6] = TS_A6;

	/* setup periodic sampling */
	t_dev->sample_rate_in_jiffies   = TOUCHSCREEN_SAMPLING_J;
	t_dev->debounce_in_samples_down = TS_DEBOUNCE_DOWN; /* min down */
	t_dev->debounce_in_samples_up   = TS_DEBOUNCE_UP;  /* min up    */
	t_dev->report_events     = REMASK_REPORT;   /* send input_event data */
	t_dev->stylus_down_count = 0;	/* # consecutive stylus down samples */
	t_dev->stylus_up_count   = 0;   /* "      "        "     up     "    */
	setup_timer(&t_dev->touchscreen_timer, touchscreen_monitor_task,
		    (unsigned long)t_dev);
	t_dev->touchscreen_timer.expires = get_jiffies_64()
		                                + t_dev->sample_rate_in_jiffies;

	add_timer(&t_dev->touchscreen_timer);	/*run*/

	sysfs_create_group(&pdev->dev.kobj, &touchscreen_attr_group);
	dev_info( &t_dev->i_dev->dev, "lf1000_ts2: probe done ok\n");
	return 0;

err_free_devs:
	dev_crit( &t_dev->i_dev->dev, "lf1000_ts2: probe error\n");
	if (&t_dev->touchscreen_timer != NULL) {
		t_dev->stop_timer = 1;		// don't reload timer
		del_timer_sync(&t_dev->touchscreen_timer);
	}
	sysfs_remove_group(&pdev->dev.kobj, &touchscreen_attr_group);
	input_free_device(t_dev->i_dev);
	input_free_device(t_dev->iraw_dev);
	return error;
}

static int lf1000_ts2_remove(struct platform_device *pdev)
{
	struct touch *t_dev = platform_get_drvdata(pdev);

	if (&t_dev->touchscreen_timer != NULL) {
		t_dev->stop_timer = 1;		// don't reload timer
		del_timer_sync(&t_dev->touchscreen_timer);
	}

	destroy_workqueue(t_dev->touchscreen_tasks);

	sysfs_remove_group(&pdev->dev.kobj, &touchscreen_attr_group);

	input_unregister_device(t_dev->i_dev);
	input_unregister_device(t_dev->iraw_dev);
	return 0;
}

/* fake release function to quiet "does not have a release()" warning */

void lf1000_ts2_release(struct device *dev)
{
}

static struct platform_device lf1000_ts2_device = {
	.name		= "lf1000-touchscreen",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.release = lf1000_ts2_release,
	}
};

static struct platform_driver lf1000_ts2_driver = {
	.probe		= lf1000_ts2_probe,
	.remove		= lf1000_ts2_remove,
	.driver		= {
		.name	= "lf1000-touchscreen",
		.owner	= THIS_MODULE,
		},
};

static int __init lf1000_ts2_init(void)
{
	int ret;
	ret = platform_device_register(&lf1000_ts2_device);
	ret = platform_driver_register(&lf1000_ts2_driver);
	return(ret);
}

static void __exit lf1000_ts2_exit(void)
{
	platform_driver_unregister(&lf1000_ts2_driver);
	platform_device_unregister(&lf1000_ts2_device);
}

module_init(lf1000_ts2_init);
module_exit(lf1000_ts2_exit);

MODULE_AUTHOR("Scott Esters <sesters@leapfrog.com>");
MODULE_DESCRIPTION("LF1000 Touchscreen driver Try 2");
MODULE_LICENSE("GPL");

