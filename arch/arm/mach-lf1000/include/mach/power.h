#ifndef _LF1000_POWER_H_
#define _LF1000_POWER_H_

/* default values */
#define MAX_BATTERY_MV	8000		/* max expected battery value	*/
#define LOW_BATTERY_MV	4200		/* low battery			*/
#define LOW_BATTERY_REPEAT_MV 100	/* repeat every 100mv drop	*/
/*
 * Lower critical battery level below hardware shutoff,
 * allowing play until you die.  Can still adjust level via
 * /sys/devices/platform/lf1000-power/critical_battery_mv interface
 */
#define CRITICAL_BATTERY_MV 2000	/* critical low battery		*/

/* Hysteresis low to normal Battery */
#define NORMAL_BATTERY_MV   (LOW_BATTERY_MV + 400)

enum lf1000_power_status {
	UNKNOWN 		= 0,
	EXTERNAL		= 1,	/* on external power */
	BATTERY			= 2,	/* on battery power */
	LOW_BATTERY		= 3,
	CRITICAL_BATTERY	= 4,
	NIMH			= 5,	/* on NIMH battery power   */
	NIMH_CHARGER		= 6,	/* in NiMH battery charger */
};

enum lf1000_power_source {
	POWER_UNKNOWN		= 0,	/* Unknown power source	*/
	POWER_OTHER		= 1,	/* Unknown power source	*/
	POWER_NIMH		= 2,	/* using NiMH battery */
	POWER_NIMH_CHARGER	= 3,	/* in NiMH battery charger */
	POWER_NIMH_EXTERNAL	= 4,	/* using NiMH with external power */
	POWER_BATTERY		= 5,	/* standard battery */
	POWER_EXTERNAL		= 6,	/* external power source */
};
	
#endif
