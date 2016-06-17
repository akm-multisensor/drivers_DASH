/******************************************************************************
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 ******************************************************************************/
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define AKM_TRACE		0

#define TO_STRING_(x)	#x
#define TO_STRING(x)	TO_STRING_(x)

#ifdef DEBUG
#define DEBUG_OUTPUT  printf
#else
#define DEBUG_OUTPUT
#endif

#if AKM_TRACE
#define FUNCTION_TRACE  printf("%s\n", __func__);
#else
#define FUNCTION_TRACE
#endif

char const *const IIO_EVENT_DEVICE_PATH = "/dev/iio:device0";
char const *const IIO_PATH = "/sys/bus/iio/devices/iio:device0";
char const *const IIO_SCAN_ELEMENTS_EN = "scan_elements/in_magn_%c_en";
char const *const IIO_SCAN_ELEMENTS_IDX = "scan_elements/in_magn_%c_index";

static volatile bool is_prompt_request = false;
static unsigned int elem;

static void signal_handler(int sig)
{
	if (sig == SIGINT) {
		is_prompt_request = true;
	}
}

int get_data(int iio_dev_fd)
{
	char	rdata[100];
	int		err = 0;
	fd_set	read_fds;
	ssize_t	nbyte;
	int		i;
	int32_t *val;

	FUNCTION_TRACE

	if (iio_dev_fd < 0) {
		return -1;
	}

	FD_ZERO(&read_fds);
	FD_SET(iio_dev_fd, &read_fds);
	err = select(iio_dev_fd + 1, &read_fds, NULL, NULL, NULL);

	if (err <= 0) {
		fprintf(stderr, "select failed: err=%d", err);
		return -1;
	}

	if (!FD_ISSET(iio_dev_fd, &read_fds)) {
		return -1;
	}

	nbyte = read(iio_dev_fd, rdata, sizeof(rdata));

	if (nbyte < 0) {
		fprintf(stderr, "Input read error: num=%d", nbyte);
		return -1;
	}

	printf("=== data ===\n  ");
	if (nbyte >= 24) {
		val = (int32_t *)rdata;
		/* assume that x,y,z,st (32bit) & time (64bit) */
		for (i = 0; i < 3; i++) {
			printf("%10.3f ", (*val / 65536.0f));
			val++;
		}
		printf("uT, st=0x%04X,", *val);
		val++;
		printf(" tm=%lf sec", (*(uint64_t *)val) / 1000000000.0);
	} else {
	printf("\n");
		for (i = 0; i < nbyte; i++) {
			printf("0x%02X,", rdata[i]);
		}
	}
	printf("\n");

	return 0;
}

void action_loop(int iio_dev_fd)
{
	bool is_loop_continue = true;

	FUNCTION_TRACE

	while (is_loop_continue) {
		if (true == is_prompt_request) {
			/* Reset flag */
			is_prompt_request = false;
			is_loop_continue = false;
		}
		if (get_data(iio_dev_fd) < 0) {
			fprintf(stderr,
					"Failed to get data.\n");
			is_loop_continue = false;
		}
	}

	printf("good bye.\n");
}

int read_attr_int(char *path, int *val)
{
	char pbuf[128];
	char rbuf[32];
	int fd, ret;
	int nread;

	fd = -1;
	ret = 0;

	snprintf(pbuf, sizeof(pbuf), "%s/%s", IIO_PATH, path);
	fd = open(pbuf, O_RDONLY);
	if (fd < 0) {
		ret = errno;
		fprintf(stderr, "fail to open with err=%d\n", ret);
		fprintf(stderr, "  %s\n", pbuf);
		goto err_check_elements;
	}
	/* fill with 0 */
	memset(rbuf, 0, sizeof(rbuf));
	/* keep the last byte '0' for terminating rbuf */
	nread = read(fd, rbuf, sizeof(rbuf) - 1);
	if (nread < 0) {
		ret = errno;
		fprintf(stderr, "fail to read with err=%d\n", ret);
		fprintf(stderr, "  %s\n", pbuf);
		goto err_check_elements;
	}
	*val = atoi(rbuf);

err_check_elements:
	if (fd >= 0)
		close(fd);
	return ret;
}

int check_elements(void)
{
	char path[128];
	char rbuf[32];
	int idx, en;
	int err;

	elem = 0;
	/* for X */
	snprintf(path, sizeof(path), IIO_SCAN_ELEMENTS_IDX, 'x');
	err = read_attr_int(path, &idx);
	if (err < 0)
		return err;
	snprintf(path, sizeof(path), IIO_SCAN_ELEMENTS_EN, 'x');
	err = read_attr_int(path, &en);
	if (err < 0)
		return err;
	if (en)
		elem |= (1 << idx);

	return 0;
}

int main(int argc, char **argv)
{
	int err = 0;
	int iio_dev_fd;
	int i;

	iio_dev_fd = open(IIO_EVENT_DEVICE_PATH, O_RDONLY);

	if (iio_dev_fd < 0) {
		err = errno;
		fprintf(stderr,
				"Failed to open iio device file - errno = %d\n",
				err);
		return err;
	}

	// SIGNAL
	signal(SIGINT, signal_handler);

	// check enabled
	// Main function
	action_loop(iio_dev_fd);

	for (i = 0; i < 100; ++i) {
		if (0 == close(iio_dev_fd)) {
			break;
		}

		usleep(1000);
	}

	return err;
}

