/*---------------------------------------------------------------------------
 * stepper_spi.h
 * The Stepper Motor mode control driver
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
#ifndef __STEPPER_SPI_H__
#define __STEPPER_SPI_H__
/*
 * --------------------------------------------------------------------------
 * M2 M1 M0    M0 M1 M2          Step Mode
 * --------------------------------------------------------------------------
 * 0  0  0  -> 0  0  0  :  Full step (2-phase excitation) with 71% current
 * 0  0  1  -> 1  0  0  :  1/2 step (1-2 phase excitation)
 * 0  1  0  -> 0  1  0  :  1/4 step (W1-2 phase excitation)
 * 0  1  1  -> 1  1  0  :  8 microsteps / step
 * 1  0  0  -> 0  0  1  :  16 microsteps / step
 * 1  0  1  -> 1  0  1  :  32 microsteps / step
 * 1  1  0  -> 0  1  1  :  32 microsteps / step
 * 1  1  1  -> 1  1  1  :  32 microsteps / step
 * --------------------------------------------------------------------------
 */
#define STEP_MODE_1    (0x0)
#define STEP_MODE_2    (0x4)
#define STEP_MODE_4    (0x2)
#define STEP_MODE_8    (0x6)
#define STEP_MODE_16   (0x1)
#define STEP_MODE_32   (0x5)

typedef struct stepper_cmd {
    uint8_t enable;
    uint8_t sleep;
    uint8_t reset;
    uint8_t decay;
    uint8_t mode;
} stepper_cmd_t;

/* ioctl commands for stepper spi driver */
#define STEPPER_IOC_MAGIC    't'

#define STEPPER_SET_CMD     _IOW (STEPPER_IOC_MAGIC, 0x90, stepper_cmd_t)
#define STEPPER_GET_CMD     _IOR (STEPPER_IOC_MAGIC, 0x91, stepper_cmd_t)

#define STEPPER_CH_MIN_X    _IO (STEPPER_IOC_MAGIC, 0x92)
#define STEPPER_CH_MIN_Y    _IO (STEPPER_IOC_MAGIC, 0x93)
#define STEPPER_CH_MIN_Z    _IO (STEPPER_IOC_MAGIC, 0x94)

#define STEPPER_CH_MAX_X    _IO (STEPPER_IOC_MAGIC, 0x95)
#define STEPPER_CH_MAX_Y    _IO (STEPPER_IOC_MAGIC, 0x96)
#define STEPPER_CH_MAX_Z    _IO (STEPPER_IOC_MAGIC, 0x97)
#define AUTOLEVEL_Z			_IO (STEPPER_IOC_MAGIC, 0x98)
#define AUTOLEVEL_Z_GPIO_INPUT  _IO (STEPPER_IOC_MAGIC, 0x99)
#define AUTOLEVEL_Z_GPIO_OUTPUT _IO (STEPPER_IOC_MAGIC, 0x9a)
#ifdef __cplusplus
extern "C"
{
#endif




#ifdef __cplusplus
}
#endif

#endif
