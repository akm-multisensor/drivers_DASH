/*
 * Partly modified 2014 Asahi Kasei Microdevices Corp.
 */

/*
 * Definitions for ak8963 compass chip.
 */
#ifndef AK8963_H
#define AK8963_H

#ifdef __KERNEL__
#include <linux/input.h>
#endif

#define AKM_DRIVER_NAME "ak8963"
#define MSC_RX  MSC_SERIAL
#define MSC_RY  MSC_PULSELED
#define MSC_RZ  MSC_GESTURE
#define MSC_ST2 MSC_RAW

#ifdef __KERNEL__
struct ak8963_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	unsigned char	axis_order[3];
	unsigned char	axis_sign[3];
};
#endif

#endif

