/*

MIT License

Copyright (c) 2020 Mika Tuupola

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

-cut-

This file is part of hardware agnostic I2C driver for PCF8563 RTC:
https://github.com/tuupola/pcf8563

SPDX-License-Identifier: MIT
*/

/*
Copyright (c) 2020 Mauro Riva
Modified by Mauro Riva (lemariva.com) for MicroPython support
*/

#ifndef _PCF8563_H
#define _PCF8563_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <time.h>
#include "i2cdev.h"


#define	PCF8563_ADDRESS	         (0x51)
#define	PCF8563_CONTROL_STATUS1  (0x00)
#define	PCF8563_TESTC            (0b00001000)
#define	PCF8563_STOP             (0b00100000)
#define	PCF8563_TEST1            (0b10000000)
#define	PCF8563_CONTROL_STATUS2  (0x01)
#define	PCF8563_TIE              (0b00000001)
#define	PCF8563_AIE              (0b00000010)
#define	PCF8563_TF               (0b00000100)
#define	PCF8563_AF               (0b00001000)
#define	PCF8563_TI_TP            (0b00010000)
#define	PCF8563_SECONDS          (0x02)
#define	PCF8563_MINUTES          (0x03)
#define	PCF8563_HOURS            (0x04)
#define	PCF8563_DAY              (0x05)
#define	PCF8563_WEEKDAY          (0x06)
#define	PCF8563_MONTH            (0x07)
#define	PCF8563_YEAR             (0x08)
#define	PCF8563_TIME_SIZE        (0x07)
#define	PCF8563_CENTURY_BIT      (0b10000000)

#define PCF8563_MINUTE_ALARM     (0x09)
#define PCF8563_HOUR_ALARM       (0x0a)
#define PCF8563_DAY_ALARM        (0x0b)
#define PCF8563_WEEKDAY_ALARM    (0x0c)
#define PCF8563_ALARM_DISABLE    (0b10000000)
#define PCF8563_ALARM_NONE       (0xff)
#define PCF8563_ALARM_SIZE       (0x04)

#define PCF8563_TIMER_CONTROL    (0x0e)
#define PCF8563_TIMER_ENABLE     (0b10000000)
#define PCF8563_TIMER_4_096KHZ   (0b00000000)
#define PCF8563_TIMER_64HZ       (0b00000001)
#define PCF8563_TIMER_1HZ        (0b00000010)
#define PCF8563_TIMER_1_60HZ     (0b00000011)
#define PCF8563_TIMER            (0x0f)

/* IOCTL commands */
#define PCF8563_ALARM_SET        (0x0900)
#define PCF8563_ALARM_READ       (0x0901)
#define PCF8563_CONTROL_STATUS1_READ     (0x0000)
#define PCF8563_CONTROL_STATUS1_WRITE    (0x0001)
#define PCF8563_CONTROL_STATUS2_READ     (0x0100)
#define PCF8563_CONTROL_STATUS2_WRITE    (0x0101)
#define PCF8563_TIMER_CONTROL_READ       (0x0e00)
#define PCF8563_TIMER_CONTROL_WRITE      (0x0e01)
#define PCF8563_TIMER_READ               (0x0f00)
#define PCF8563_TIMER_WRITE              (0x0f01)

/* Status codes. */
#define PCF8563_ERROR_NOTTY      (-1)
#define PCF8563_OK               (0x00)
#define PCF8563_ERR_LOW_VOLTAGE  (0x80)

typedef int32_t pcf8563_err_t;

pcf8563_err_t pcf8563Init(I2C_Dev *i2cPort);
pcf8563_err_t pcf8563_init();
pcf8563_err_t pcf8563_read(struct tm *time);
pcf8563_err_t pcf8563_write(const struct tm *time);
pcf8563_err_t pcf8563_ioctl(int16_t command, void *buffer);
pcf8563_err_t pcf8563_start_timer(bool enable);
pcf8563_err_t pcf8563_close();

#ifdef __cplusplus
}
#endif
#endif