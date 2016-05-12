/*---------------------------------------------------------------------------
 * stepper.c
 * Test Routine for Stepper SPI driver
 * Author: Truby.Zong
 * Date:   2014-05-09
 *
 * Copyright (C) 2014 Truby Zong <truby.zong@gmail.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 ---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdint.h>
#include <stepper_spi.h>

#define ST_DEV "/dev/stepper_spi"

int main(int argc, char *argv[])
{
    int i;
    int ret;
    int fd;
    stepper_cmd_t cmd; 

    printf("open dev\n");
    fd = open(ST_DEV, O_RDWR);
    if (fd < 0) {
        printf("open fe_reg dev err\n"); 
        return -1;
    }

    cmd.enable = 0x1; 
    cmd.sleep = 0x0;
    cmd.reset = 0x0;
    cmd.decay = 0x0;
    cmd.mode = 0x7;

    printf("set up motor mode\n");
    printf("enable %#x\n", cmd.enable);
    printf("sleep %#x\n", cmd.sleep);
    printf("reset %#x\n", cmd.reset);
    printf("decay %#x\n", cmd.decay);
    printf("mode %#x\n", cmd.mode);

    ret = ioctl(fd, STEPPER_SET_CMD, &cmd);
    if (ret < 0) {
        perror("ioctl failed");
        return -1;
    }

    close(fd);
    
    printf("ok\n");
    return 0;
}

