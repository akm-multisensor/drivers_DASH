/******************************************************************************
 *
 *  $Id$
 *
 * -- Copyright Notice --
 *
 * Copyright (c) 2004 Asahi Kasei Microdevices Corporation, Japan
 * All Rights Reserved.
 *
 * This software program is the proprietary program of Asahi Kasei Microdevices
 * Corporation("AKM") licensed to authorized Licensee under the respective
 * agreement between the Licensee and AKM only for use with AKM's electronic
 * compass IC.
 *
 * THIS SOFTWARE IS PROVIDED TO YOU "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABLITY, FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT OF
 * THIRD PARTY RIGHTS, AND WE SHALL NOT BE LIABLE FOR ANY LOSSES AND DAMAGES
 * WHICH MAY OCCUR THROUGH USE OF THIS SOFTWARE.
 *
 * -- End Asahi Kasei Microdevices Copyright Notice --
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

#include <linux/ak0991x.h>
#include <linux/input.h>

#define AKM_TRACE                0
#define AKM_PRINT_HEX            0
#define AKMD_SEQUENTIAL_MEASURE  1
#define AKMD_SEQUENTIAL_TIMER    1
#define AKMD_MEASURE_FREQ_HZ     10


#define TO_STRING_(x)  #x
#define TO_STRING(x)   TO_STRING_(x)

#if AKM_TRACE
#define FUNCTION_TRACE  printf("%s\n", __func__);
#else
#define FUNCTION_TRACE
#endif

enum {
    continuous = 0,
    info,
    interval,
    nsf,
    reset,
    selftest,
    single,
    num_of_attr
};

char const *const SYSFS_CONTINUOUS_FILE_NAME = TO_STRING(
        /sys/devices/virtual/input/input1/continuous);
char const *const SYSFS_INFO_FILE_NAME = TO_STRING(
        /sys/devices/virtual/input/input1/info);
char const *const SYSFS_INTERVAL_FILE_NAME = TO_STRING(
        /sys/devices/virtual/input/input1/interval);
char const *const SYSFS_NSF_FILE_NAME = TO_STRING(
        /sys/devices/virtual/input/input1/nsf);
char const *const SYSFS_RESET_FILE_NAME = TO_STRING(
        /sys/devices/virtual/input/input1/reset);
char const *const SYSFS_SELFTEST_FILE_NAME = TO_STRING(
        /sys/devices/virtual/input/input1/selftest);
char const *const SYSFS_SINGLE_FILE_NAME = TO_STRING(
        /sys/devices/virtual/input/input1/single);
char const *const INPUT_EVENT_DEVICE_PATH = "/dev/input/event1";

static volatile bool is_prompt_request = false;

static volatile int g_fds[num_of_attr];

static void signal_handler(int sig)
{
    if (sig == SIGINT) {
        is_prompt_request = true;
    }
}

int get_mag_data(
    int      input_dev_fd,
    int32_t  data[4],
    uint64_t *ts)
{
    const int          EVENT_NUM = 64;
    struct input_event iev[EVENT_NUM];
    int                err = 0;
    fd_set             read_fds;
    struct timeval     timeout = { .tv_sec = 1, .tv_usec = 0 };
    ssize_t            nbyte;
    size_t             nread;
    int                i;

    FUNCTION_TRACE

    if (input_dev_fd < 0) {
        return -1;
    }

    FD_ZERO(&read_fds);
    FD_SET(input_dev_fd, &read_fds);
    err = select(input_dev_fd + 1, &read_fds, NULL, NULL, &timeout);

    if (err <= 0) {
        return -1;
    }

    if (!FD_ISSET(input_dev_fd, &read_fds)) {
        return -1;
    }

    nbyte = read(input_dev_fd, iev, EVENT_NUM * sizeof(struct input_event));

    if (nbyte < 0 || nbyte % sizeof(struct input_event)) {
        fprintf(stderr, "Input read error: num=%d", nbyte);
        return -1;
    }

    nread = nbyte / sizeof(struct input_event);

    for (i = 0; i < nread; i++) {
        if (iev[i].type == EV_SYN) {
#if AKM_PRINT_HEX
            printf("Dat:%8d, %8d, %8d, 0x%02X, Ts:0x%LX\n",
                   data[0],
                   data[1],
                   data[2],
                   data[3],
                   *ts);
#else
            printf("Dat:%8.2f, %8.2f, %8.2f, 0x%02X, Ts:%f\n",
                   data[0] / 65536.f,
                   data[1] / 65536.f,
                   data[2] / 65536.f,
                   data[3],
                   *ts / 1000000000.f);
#endif
        }

        if (iev[i].type != EV_MSC) {
            continue;
        }

        switch (iev[i].code) {
        case MSC_RX:
            data[0] = iev[i].value;
            break;

        case MSC_RY:
            data[1] = iev[i].value;
            break;

        case MSC_RZ:
            data[2] = iev[i].value;
            break;

        case MSC_ST2:
            data[3] = iev[i].value;
            break;

        /* low byte must be the first */
        case MSC_TSL:
            *ts = (uint32_t)iev[i].value;
            break;

        case MSC_TSH:
            *ts += ((uint64_t)iev[i].value << 32);
            break;

        default:
            fprintf(stderr, "Unknown code (%d, %d)\n", iev[i].code,
                    iev[i].value);
            break;
        }
    }

    return 0;
}

int perform_singleshot()
{
    int  err;
    char value[1];

    FUNCTION_TRACE

    if (g_fds[single] < 0) {
        g_fds[single] = open(SYSFS_SINGLE_FILE_NAME, O_WRONLY);

        if (g_fds[single] < 0) {
            err = errno;
            fprintf(stderr,
                    "ecompass: failed to open single-shot - errno = %d\n", err);
            return err;
        }
    }

    value[0] = '1';

    if (write(g_fds[single], value, sizeof(value)) < 0) {
        err = errno;
        fprintf(stderr,
                "ecompass: failed to perform single-shot - errno = %d\n", err);
        return err;
    }

    return 0;
}

int perform_continuous(int mode)
{
    int  err;
    char value[8];

    FUNCTION_TRACE

    if (g_fds[continuous] < 0) {
        g_fds[continuous] = open(SYSFS_CONTINUOUS_FILE_NAME, O_WRONLY);

        if (g_fds[continuous] < 0) {
            err = errno;
            fprintf(stderr,
                    "ecompass: failed to open continusous - errno = %d\n", err);
            return err;
        }
    }

    sprintf(value, "%d", mode);

    if (write(g_fds[continuous], value, sizeof(value)) < 0) {
        err = errno;
        fprintf(stderr, "ecompass: failed to perform continuous - errno = %d\n",
                err);
        return err;
    }

    return 0;
}

int perform_selftest()
{
    int  err;
    char value[1];

    FUNCTION_TRACE

    if (g_fds[selftest] < 0) {
        g_fds[selftest] = open(SYSFS_SELFTEST_FILE_NAME, O_WRONLY);

        if (g_fds[selftest] < 0) {
            err = errno;
            fprintf(stderr, "ecompass: failed to open self-test - errno = %d\n",
                    err);
            return err;
        }
    }

    value[0] = '1';

    if (write(g_fds[selftest], value, strlen(value)) < 0) {
        err = errno;
        fprintf(stderr, "ecompass: failed to perform self-test - errno = %d\n",
                err);
        return err;
    }

    return 0;
}

int set_interval(int val)
{
    int  err;
    char val_string[10];

    FUNCTION_TRACE

    if (g_fds[interval] < 0) {
        g_fds[interval] = open(SYSFS_INTERVAL_FILE_NAME, O_RDWR);

        if (g_fds[interval] < 0) {
            err = errno;
            fprintf(stderr,
                    "ecompass: failed to open set-interval - errno = %d\n",
                    err);
            return err;
        }
    }

    if (val > 999999999) {
        val = 999999999;
    }

    if (val < 0) {
        val = 0;
    }

    memset(val_string, 0, sizeof(val_string));
    snprintf(val_string, sizeof(val_string), "%d", val);

    if (write(g_fds[interval], val_string, sizeof(val_string)) < 0) {
        err = errno;
        fprintf(stderr,
                "ecompass: failed to perform set-interval - errno = %d\n", err);
        return err;
    }

    return 0;
}

int action_loop(int input_dev_fd)
{
    bool     is_loop_continue = true;
    int      err;
    uint32_t ms;
    int32_t  data[4];
    uint64_t ts;
    char     msg[20];

    FUNCTION_TRACE

    if (input_dev_fd < 0) {
        return EBADF;
    }

    // Init
    ms = 1000 / AKMD_MEASURE_FREQ_HZ;
    data[0] = data[1] = data[2] = data[3] = 0;

#if AKMD_SEQUENTIAL_MEASURE
#if AKMD_SEQUENTIAL_TIMER
    err = set_interval(ms);
#else
    err = perform_continuous(ms);
#endif

    if (err < 0) {
        return err;
    }

    while (is_loop_continue) {
#else

    while (is_loop_continue) {
        // single-shot
        err = perform_singleshot();

        if (err < 0) {
            return err;
        }

        usleep(ms * 1000);
#endif

        // Get data
        if (get_mag_data(input_dev_fd, data, &ts)) {
            fprintf(stderr, "get_mag_data failed.\n");
        }

        if (true == is_prompt_request) {
            /* Reset flag */
            is_prompt_request = false;
            /* Menu */
            printf("1) Menu 1.\n");
            printf("2) Menu 2.\n");
            printf("3) Menu 3.\n");
            printf("q) quit.\n");
            fgets(msg, 20, stdin);

            /* Process */
            switch (msg[0]) {
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

#if AKMD_SEQUENTIAL_MEASURE
    // Stop measure
    err = perform_continuous(-1);

    if (err < 0) {
        return err;
    }
#endif

    printf("ecompass: good bye.\n");
    return 0;
}

int main(
    int  argc,
    char **argv)
{
    int err = 0;
    int input_dev_fd;
    int i;

    input_dev_fd = open(INPUT_EVENT_DEVICE_PATH, O_RDONLY);

    if (input_dev_fd < 0) {
        err = errno;
        fprintf(stderr,
                "ecompass: failed to open input device file - errno = %d\n",
                errno);
        return err;
    }

    // Init
    for (i = 0; i < num_of_attr; i++) {
        g_fds[i] = -1;
    }

    // SIGNAL
    signal(SIGINT, signal_handler);

    // Main function
    err = action_loop(input_dev_fd);

    if (err < 0) {
        fprintf(stderr, "ecompass: error = %d\n", err);
    }

    for (i = 0; i < 100; ++i) {
        if (0 == close(input_dev_fd)) {
            break;
        }

        usleep(1000);
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
