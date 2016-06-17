/******************************************************************************
 *
 * Copyright (c) 2014 Asahi Kasei Microdevices Corporation, Japan
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
 ******************************************************************************/
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

void print_usage(const char *prog)
{
    fprintf(stdout, "Usage: %s [-b bus][-s slave]\n", prog);
    fprintf(stdout, " -b bus : i2c bus number.\n");
    fprintf(stdout, " -s addr: i2c slave address.\n");
}

/* declare and set default value */
int g_bus = 0;
int g_slave = 0xC;

int opt_parse(
    int  argc,
    char **argv)
{
    int  opt;
    char optVal;
    char *prog;

    if ((prog = strrchr(argv[0], '/')) != NULL) {
        /* remove last slash */
        prog++;
    } else {
        /* use argument */
        prog = argv[0];
    }

    while ((opt = getopt(argc, argv, "b:s:h")) != -1) {
        switch (opt) {
        case 'b':
            g_bus = strtol(optarg, NULL, 0);
            break;

        case 's':
            g_slave = strtol(optarg, NULL, 0);
            break;

        case 'h':
        default:
            print_usage(prog);
            exit(1);
        }
    }

    return 0;
}

int read_operation(int fd)
{
    unsigned char buf[2];
    char          msg[10];
    int           err;

    fprintf(stdout, "addr --> ");
    fgets(msg, 10, stdin);
    buf[0] = strtol(msg, NULL, 0);

    /* write address */
    if ((write(fd, buf, 1)) != 1) {
        err = errno;
        fprintf(stderr, "\nFailed to set address (fd=%d, 0x%02X).\n", fd,
                buf[0]);
        fprintf(stderr, " err=%d: %s.\n\n", err, strerror(err));
        return 1;
    }

    /* read */
    if (read(fd, &buf[1], 1) != 1) {
        err = errno;
        fprintf(stderr, "\nFailed to read from the device (fd=%d, 0x%02X).\n",
                fd, buf[0]);
        fprintf(stderr, " err=%d: %s.\n\n", err, strerror(err));
        return 1;
    }

    fprintf(stdout, "Read success (0x%02X, 0x%02X).\n", buf[0], buf[1]);
    return 0;
}

int write_operation(int fd)
{
    unsigned char buf[2];
    char          msg[10];
    int           err;

    /* input address */
    fprintf(stdout, "addr --> ");
    fgets(msg, 10, stdin);
    buf[0] = strtol(msg, NULL, 0);
    /* input data */
    fprintf(stdout, "data --> ");
    fgets(msg, 10, stdin);
    buf[1] = strtol(msg, NULL, 0);

    /* write */
    if ((write(fd, buf, 2)) != 2) {
        err = errno;
        fprintf(stderr,
                "\nFailed to write to the device (fd=%d, 0x%02X, 0x%02X).\n",
                fd, buf[0],
                buf[1]);
        fprintf(stderr, " err=%d: %s.\n\n", err, strerror(err));
        return 1;
    }

    fprintf(stdout, "Write success.\n");
    return 0;
}

int main(
    int  argc,
    char **argv)
{
    /* */
    char i2c_dev_fn[64];
    char msg[10];
    int  err;
    int  cmd;
    int  fd = -1;
    int  loop = 1;

    /* parse option */
    opt_parse(argc, argv);

    /* try to open i2c bus */
    sprintf(i2c_dev_fn, "/dev/i2c-%d", g_bus);

    if ((fd = open(i2c_dev_fn, O_RDWR)) < 0) {
        err = errno;
        fprintf(stderr, "\nFailed to open i2c port (%s).\n", i2c_dev_fn);
        fprintf(stderr, " err=%d: %s.\n\n", err, strerror(err));
        return 1;
    }

    fprintf(stdout, "Bus '%s' opened at fd=%d.\n", i2c_dev_fn, fd);

    /* try to set slave address */
    if (ioctl(fd, I2C_SLAVE, g_slave) < 0) {
        err = errno;
        fprintf(stderr, "\nUnable to get bus access (%02X).\n", g_slave);
        fprintf(stderr, " err=%d: %s.\n\n", err, strerror(err));
        return 1;
    }

    fprintf(stdout, "Set slave address to 0x%02X.\n", g_slave);

    while (loop) {
        /* interactive menu */
        fprintf(stdout, "----- Select Operation -----\n");
        fprintf(stdout, "  R. Read from address.\n");
        fprintf(stdout, "  W. Write to address.\n");
        fprintf(stdout, "  Q. Quit.\n");
        fprintf(stdout, "----------------------------\n");
        fprintf(stdout, "     --> ");
        fgets(msg, 10, stdin);
        cmd = tolower(msg[0]);

        if (cmd == 'r') {
            /* start read operation */
            read_operation(fd);
        } else if (cmd == 'w') {
            /* start write operation */
            write_operation(fd);
        } else if (cmd == 'q') {
            loop = 0;
        } else {
            fprintf(stdout, "\nUnknown command (%c).\n\n", cmd);
        }
    }

    fprintf(stdout, "\nbye bye.\n\n");
    return 0;
}
