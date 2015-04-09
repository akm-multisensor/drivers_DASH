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
#include <time.h>
#include <unistd.h>
#include <linux/input.h>

#define AKM_TRACE      0

#define TO_STRING_(x)  #x
#define TO_STRING(x)   TO_STRING_(x)

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

char const *const INPUT_EVENT_DEVICE_PATH = "/dev/input/event1";

static volatile bool is_prompt_request = false;

static void signal_handler(int sig)
{
    if (sig == SIGINT) {
        is_prompt_request = true;
    }
}

int get_data(int input_dev_fd)
{
    struct input_event iev;
    int                err = 0;
    fd_set             read_fds;
    ssize_t            nbyte;

    FUNCTION_TRACE

    if (input_dev_fd < 0) {
        return -1;
    }

    FD_ZERO(&read_fds);
    FD_SET(input_dev_fd, &read_fds);
    err = select(input_dev_fd + 1, &read_fds, NULL, NULL, NULL);

    if (err <= 0) {
        return -1;
    }

    if (!FD_ISSET(input_dev_fd, &read_fds)) {
        return -1;
    }

    nbyte = read(input_dev_fd, &iev, sizeof(struct input_event));

    if (nbyte < 0 || nbyte != sizeof(struct input_event)) {
        fprintf(stderr, "Input read error: num=%d", nbyte);
        return -1;
    }

    printf("=== type=%d ===\n", iev.type);
    printf("  code=%d\n", iev.code);
    printf("  val =%d\n\n", iev.value);

    return 0;
}

void action_loop(int input_dev_fd)
{
    bool is_loop_continue = true;
    int  err;

    FUNCTION_TRACE

    while (is_loop_continue) {
        if (true == is_prompt_request) {
            /* Reset flag */
            is_prompt_request = false;
            is_loop_continue = false;
        }
        if (get_data(input_dev_fd) < 0) {
            fprintf(stderr,
                "Failed to get data.\n",
                errno);
            is_loop_continue = false;
        }
    }

    printf("good bye.\n");
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
                "Failed to open input device file - errno = %d\n",
                errno);
        return err;
    }

    // SIGNAL
    signal(SIGINT, signal_handler);

    // Main function
    action_loop(input_dev_fd);

    for (i = 0; i < 100; ++i) {
        if (0 == close(input_dev_fd)) {
            break;
        }

        usleep(1000);
    }

    return err;
}

