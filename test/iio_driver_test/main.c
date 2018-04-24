/*
 * Copyright (C) 2014 ASAHI KASEI MICRODEVICES CORPORATION.
 *
 * Authors: Rikita Yamada <yamada.rj (at) om.asahi-kasei.co.jp>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#define AKM_TRACE 1
#define AKMD_SEQUENTIAL_MEASURE	1
#define AKMD_SEQUENTIAL_TIMER	1
#define AKMD_INTERVAL			100
#define AKMD_BUF_LENGTH			100
#define AKMD_WATERMARK			1

#define TO_STRING_(x) #x
#define TO_STRING(x) TO_STRING_(x)

#define EVENT_CODE_MAGV_X MSC_RX
#define EVENT_CODE_MAGV_Y MSC_RY
#define EVENT_CODE_MAGV_Z MSC_RZ
#define EVENT_CODE_ORIENT_STATUS MSC_ST2

#ifdef DEBUG
#define DEBUG_OUTPUT printf
#else
#define DEBUG_OUTPUT
#endif

#if AKM_TRACE
#define FUNCTION_TRACE	printf("%s\n", __func__)
#else
#define FUNCTION_TRACE
#endif


enum {
	buf_enable = 0,
	continuous,
	/* info, */
	frequency,
	nsf,
	reset,
	selftest,
	single,
	in_magn_x_raw,
	in_magn_y_raw,
	in_magn_z_raw,
	watermark,
	element_x_en,
	element_y_en,
	element_z_en,
	num_of_attr,
};

char const * const SYSFS_BUF_ENABLE_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/iio:device1/buffer/enable );
char const * const SYSFS_CONTINUOUS_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/iio:device1/continuous );
char const * const SYSFS_FREQUENCY_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/iio:device1/in_magn_sampling_frequency );
char const * const SYSFS_NSF_FILE_NAME			= TO_STRING( /sys/bus/iio/devices/iio:device1/nsf );
char const * const SYSFS_RESET_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/iio:device1/reset );
char const * const SYSFS_SELFTEST_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/iio:device1/selftest );
char const * const SYSFS_SINGLE_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/iio:device1/single );
char const * const SYSFS_MAG_X_RAW_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/iio:device1/in_magn_x_raw );
char const * const SYSFS_MAG_Y_RAW_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/iio:device1/in_magn_y_raw );
char const * const SYSFS_MAG_Z_RAW_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/iio:device1/in_magn_z_raw );
char const * const SYSFS_WATERMARK_FILE_NAME	= TO_STRING( /sys/bus/iio/devices/iio:device1/watermark );
char const * const SYSFS_EBL_X_EN_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_x_en );
char const * const SYSFS_EBL_Y_EN_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_y_en );
char const * const SYSFS_EBL_Z_EN_FILE_NAME		= TO_STRING( /sys/bus/iio/devices/iio:device1/scan_elements/in_magn_z_en );

static volatile bool is_prompt_request = false;

static volatile int g_fds[num_of_attr];

void signal_handler( int sig )
{
	if ( sig == SIGINT ) {
		is_prompt_request = true;
	}
}

int perform_enable( int arg_en )
{
	char val_en[1];
	int err;

	if(g_fds[arg_en]){
		switch(arg_en){
			case buf_enable:
				g_fds[arg_en] = open(SYSFS_BUF_ENABLE_FILE_NAME, O_RDWR);
				break;
			case element_x_en:
				g_fds[arg_en] = open(SYSFS_EBL_X_EN_FILE_NAME, O_RDWR);
				break;
			case element_y_en:
				g_fds[arg_en] = open(SYSFS_EBL_Y_EN_FILE_NAME, O_RDWR);
				break;
			case element_z_en:
				g_fds[arg_en] = open(SYSFS_EBL_Z_EN_FILE_NAME, O_RDWR);
				break;
			default:
				break;
		}
	}

	if (g_fds[arg_en] < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to open set - enable - errno = %d\n", err);
		return err;
	}

	val_en[0] = '1';

	if (write(g_fds[arg_en], val_en, sizeof(val_en)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform set - enable - errno = %d\n", err);
		return err;
	}

	usleep(100);
	return 0;
}

int perform_watermark( int arg_wm )
{
	int err;
	char val_string[10];

	if (g_fds[watermark] < 0) {
		g_fds[watermark] = open(SYSFS_WATERMARK_FILE_NAME, O_RDWR);
		if (g_fds[watermark] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open set - watermark - errno = %d\n", err);
			return err;
		}
	}

	memset(val_string, 0, sizeof(val_string));
	snprintf(val_string, sizeof(val_string), "%d", arg_wm);

	if (write(g_fds[watermark], val_string, sizeof(val_string)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform set-watermark - errno = %d\n", err);
		return err;
	}

	return 0;
}

int get_mag_data(int data[4])
{
	char buf[16];
	char *endptr;
	ssize_t nbyte;
	int fd;
	int err = 0;

	FUNCTION_TRACE;

	/* Read X */
	fd = open(SYSFS_MAG_X_RAW_FILE_NAME, O_RDONLY);

	if (fd < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to open magn_x - errno = %d\n", err);
	} else {
		nbyte = read(fd, buf, sizeof(buf));
		if (nbyte == 0) {
			err = errno;
			fprintf(stderr, "Read X error: errno = %d\n", err);
		} else {
			data[0] = strtol(buf, &endptr, 10);
		}
		close(fd);
	}

	/* Read Y */
	fd = open(SYSFS_MAG_Y_RAW_FILE_NAME, O_RDONLY);
	if (fd < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to open magn_y - errno = %d\n", err);
	} else {
		nbyte = read(fd, buf, sizeof(buf));
		if (nbyte == 0) {
			err = errno;
			fprintf(stderr, "Read Y error: errno = %d\n", err);
		} else {
			data[1] = strtol(buf, &endptr, 10);
		}
		close(fd);
	}

	/* Read Z */
	fd = open(SYSFS_MAG_Z_RAW_FILE_NAME, O_RDONLY);
	if (fd < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to open magn_z - errno = %d\n", err);
	} else {
		nbyte = read(fd, buf, sizeof(buf));
		if (nbyte == 0) {
			err = errno;
			fprintf(stderr, "Read Z error: errno = %d\n", err);
		} else {
			data[2] = strtol(buf, &endptr, 10);
		}
		close(fd);
	}

	return 0;
}

int perform_singleshot( void )
{
	int err;
	char value[1];

	FUNCTION_TRACE;

	if (g_fds[single] < 0) {
		g_fds[single] = open(SYSFS_SINGLE_FILE_NAME, O_WRONLY);
		if (g_fds[single] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open single-shot - errno = %d\n", err);
			return err;
		}
	}

	value[0] = '1';

	if (write(g_fds[single], value, sizeof(value)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform single-shot - errno = %d\n", err);
		return err;
	}

	return 0;
}

int perform_continuous(int mode)
{
	int err;
	char value[8];

	FUNCTION_TRACE;

	if (g_fds[continuous] < 0) {
		g_fds[continuous] = open(SYSFS_CONTINUOUS_FILE_NAME, O_WRONLY);
		if (g_fds[continuous] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open continusous - errno = %d\n", err);
			return err;
		}
	}

	sprintf(value, "%d", mode);

	if (write(g_fds[continuous], value, sizeof(value)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform continuous - errno = %d\n", err);
		return err;
	}

	return 0;
}

int perform_selftest( void )
{
	int err;
	char value[1];

	FUNCTION_TRACE;

	if (g_fds[selftest] < 0) {
		g_fds[selftest] = open(SYSFS_SELFTEST_FILE_NAME, O_WRONLY);
		if (g_fds[selftest] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open self-test - errno = %d\n", err);
			return err;
		}
	}

	value[0] = '1';

	if (write(g_fds[selftest], value, strlen(value)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform self-test - errno = %d\n", err);
		return err;
	}

	return 0;
}

int perform_interval( int arg_interval )
{
	int err;
	char val_string[10];

	FUNCTION_TRACE;

	if (g_fds[frequency] < 0) {
		g_fds[frequency] = open(SYSFS_FREQUENCY_FILE_NAME, O_RDWR);
		if (g_fds[frequency] < 0) {
			err = errno;
			fprintf(stderr, "ecompass: failed to open set - frequency - errno = %d\n", err);
			return err;
		}
	}

	if (arg_interval > 999999999) {
		arg_interval = 999999999;
	}
	if (arg_interval < 0) {
		arg_interval = 0;
	}
	memset(val_string, 0, sizeof(val_string));
	snprintf(val_string, sizeof(val_string), "%d", arg_interval);

	if (write(g_fds[frequency], val_string, sizeof(val_string)) < 0) {
		err = errno;
		fprintf(stderr, "ecompass: failed to perform set-frequency - errno = %d\n", err);
		return err;
	}

	return 0;
}

int action_loop(void)
{
	bool is_loop_continue = true;
	int err;
	int data[4];
	char msg[20];


	FUNCTION_TRACE;

	/* Init */
	data[0] = data[1] = data[2] = data[3] = 0;

	err = perform_enable(element_x_en);
	err = perform_enable(element_y_en);
	err = perform_enable(element_z_en);
	err = perform_enable(buf_enable);

#if AKMD_SEQUENTIAL_MEASURE
#if AKMD_SEQUENTIAL_TIMER
	err = perform_watermark(AKMD_WATERMARK);
	err = perform_interval(AKMD_INTERVAL);
#else
	err = perform_continuous(AKMD_INTERVAL);
#endif


	if (err < 0) {
		return err;
	}

	while( is_loop_continue ) {
#else
	while( is_loop_continue ) {

		/* single-shot */
		err = perform_singleshot();
		if (err < 0) {
			return err;
		}
#endif
		usleep(AKMD_INTERVAL * 1000);

		/* Get data */
		if (get_mag_data(data)) {
			fprintf(stderr, "get_mag_data failed.\n");
		} else {
			printf("Data:%8.2f, %8.2f, %8.2f\n",
					data[0]/65536.f,
					data[1]/65536.f,
					data[2]/65536.f);
		}

		if( true == is_prompt_request ){
			/* Reset flag */
			is_prompt_request = false;
			/* Menu */
			printf("1) Menu 1.\n");
			printf("2) Menu 2.\n");
			printf("3) Menu 3.\n");
			printf("q) quit.\n");
			fgets(msg, 20, stdin);
			/* Process */
			switch(msg[0]){
				case '1':
					break;
				case '2':
					break;
				case '3':
					break;
				case 'q':
					is_loop_continue = false;
					break;
				default:
					break;
			}
		}
	}

	/* Stop measure */
#if AKMD_SEQUENTIAL_MEASURE
#if AKMD_SEQUENTIAL_TIMER
	err = perform_interval(-1);
#else
	err = perform_continuous(-1);
#endif
	if (err < 0) {
		return err;
	}
#endif

	printf( "ecompass: good bye.\n" );
	return 0;
}

int main( int argc, char** argv )
{
	int err = 0;
	int i;

	/* Init */
	for (i=0; i<num_of_attr; i++) {
		g_fds[i] = -1;
	}

	/* SIGNAL */
	signal( SIGINT, signal_handler );

	/* Main function */
	err = action_loop();

	if (err < 0) {
		fprintf( stderr, "ecompass: error = %d\n", err);
	}

	for (i = 0; i < num_of_attr; i++) {
		if (g_fds[i] >= 0) {
			for (i = 0; i < 100; ++i) {
				if (0 == close(g_fds[i])) {
					break;
				}
				usleep(1000);
			}
		}
	}
	return err;
}


