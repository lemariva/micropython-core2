// I2Cdev library collection - MPU6886 I2C device class
// Based on InvenSense MPU-6886 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg
Adapted to Crazyflie FW by Bitcraze

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*
Copyright (c) 2020 Mauro Riva
Adapted by Mauro Riva (lemariva.com) for MicroPython support
*/

#ifndef _MPU6886_H_
#define _MPU6886_H_

#include "i2cdev.h"


#define M2T(X) ((unsigned int)(X)/ portTICK_PERIOD_MS) //ms to tick
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
#define T2M(X) ((unsigned int)(X)* portTICK_PERIOD_MS)   //tick to ms

#define MPU6886_DEFAULT_ADDRESS     0x68
#define MPU6886_SELF_TEST_DELAY_MS  10

#define MPU6886_ERROR_NOTTY      (-1)
#define MPU6886_OK               (0x00)

//Product ID Description for MPU6886:  | High 4 bits  | Low 4 bits        |
//                                     | Product Name | Product Revision  |
#define MPU6886_REV_C4_ES     0x14  //        0001           0100
#define MPU6886_REV_C5_ES     0x15  //        0001           0101
#define MPU6886_REV_D6_ES     0x16  //        0001           0110
#define MPU6886_REV_D7_ES     0x17  //        0001           0111
#define MPU6886_REV_D8_ES     0x18  //        0001           1000
#define MPU6886_REV_C4        0x54  //        0101           0100
#define MPU6886_REV_C5        0x55  //        0101           0101
#define MPU6886_REV_D6        0x56  //        0101           0110
#define MPU6886_REV_D7        0x57  //        0101           0111
#define MPU6886_REV_D8        0x58  //        0101           1000
#define MPU6886_REV_D9        0x59  //        0101           1001

#define MPU6886_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6886_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6886_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6886_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6886_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6886_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6886_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6886_RA_XA_OFFS_L_TC     0x07
#define MPU6886_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6886_RA_YA_OFFS_L_TC     0x09
#define MPU6886_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6886_RA_ZA_OFFS_L_TC     0x0B
#define MPU6886_RA_PRODUCT_ID       0x0C
#define MPU6886_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6886_RA_XG_OFFS_USRL     0x14
#define MPU6886_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6886_RA_YG_OFFS_USRL     0x16
#define MPU6886_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6886_RA_ZG_OFFS_USRL     0x18
#define MPU6886_RA_SMPLRT_DIV       0x19
#define MPU6886_RA_CONFIG           0x1A
#define MPU6886_RA_GYRO_CONFIG      0x1B
#define MPU6886_RA_ACCEL_CONFIG     0x1C
#define MPU6886_RA_FF_THR           0x1D
#define MPU6886_RA_FF_DUR           0x1E
#define MPU6886_RA_MOT_THR          0x1F
#define MPU6886_RA_MOT_DUR          0x20
#define MPU6886_RA_ZRMOT_THR        0x21
#define MPU6886_RA_ZRMOT_DUR        0x22
#define MPU6886_RA_FIFO_EN          0x23
#define MPU6886_RA_I2C_MST_CTRL     0x24
#define MPU6886_RA_I2C_SLV0_ADDR    0x25
#define MPU6886_RA_I2C_SLV0_REG     0x26
#define MPU6886_RA_I2C_SLV0_CTRL    0x27
#define MPU6886_RA_I2C_SLV1_ADDR    0x28
#define MPU6886_RA_I2C_SLV1_REG     0x29
#define MPU6886_RA_I2C_SLV1_CTRL    0x2A
#define MPU6886_RA_I2C_SLV2_ADDR    0x2B
#define MPU6886_RA_I2C_SLV2_REG     0x2C
#define MPU6886_RA_I2C_SLV2_CTRL    0x2D
#define MPU6886_RA_I2C_SLV3_ADDR    0x2E
#define MPU6886_RA_I2C_SLV3_REG     0x2F
#define MPU6886_RA_I2C_SLV3_CTRL    0x30
#define MPU6886_RA_I2C_SLV4_ADDR    0x31
#define MPU6886_RA_I2C_SLV4_REG     0x32
#define MPU6886_RA_I2C_SLV4_DO      0x33
#define MPU6886_RA_I2C_SLV4_CTRL    0x34
#define MPU6886_RA_I2C_SLV4_DI      0x35
#define MPU6886_RA_I2C_MST_STATUS   0x36
#define MPU6886_RA_INT_PIN_CFG      0x37
#define MPU6886_RA_INT_ENABLE       0x38
#define MPU6886_RA_DMP_INT_STATUS   0x39
#define MPU6886_RA_INT_STATUS       0x3A
#define MPU6886_RA_ACCEL_XOUT_H     0x3B
#define MPU6886_RA_ACCEL_XOUT_L     0x3C
#define MPU6886_RA_ACCEL_YOUT_H     0x3D
#define MPU6886_RA_ACCEL_YOUT_L     0x3E
#define MPU6886_RA_ACCEL_ZOUT_H     0x3F
#define MPU6886_RA_ACCEL_ZOUT_L     0x40
#define MPU6886_RA_TEMP_OUT_H       0x41
#define MPU6886_RA_TEMP_OUT_L       0x42
#define MPU6886_RA_GYRO_XOUT_H      0x43
#define MPU6886_RA_GYRO_XOUT_L      0x44
#define MPU6886_RA_GYRO_YOUT_H      0x45
#define MPU6886_RA_GYRO_YOUT_L      0x46
#define MPU6886_RA_GYRO_ZOUT_H      0x47
#define MPU6886_RA_GYRO_ZOUT_L      0x48
#define MPU6886_RA_EXT_SENS_DATA_00 0x49
#define MPU6886_RA_EXT_SENS_DATA_01 0x4A
#define MPU6886_RA_EXT_SENS_DATA_02 0x4B
#define MPU6886_RA_EXT_SENS_DATA_03 0x4C
#define MPU6886_RA_EXT_SENS_DATA_04 0x4D
#define MPU6886_RA_EXT_SENS_DATA_05 0x4E
#define MPU6886_RA_EXT_SENS_DATA_06 0x4F
#define MPU6886_RA_EXT_SENS_DATA_07 0x50
#define MPU6886_RA_EXT_SENS_DATA_08 0x51
#define MPU6886_RA_EXT_SENS_DATA_09 0x52
#define MPU6886_RA_EXT_SENS_DATA_10 0x53
#define MPU6886_RA_EXT_SENS_DATA_11 0x54
#define MPU6886_RA_EXT_SENS_DATA_12 0x55
#define MPU6886_RA_EXT_SENS_DATA_13 0x56
#define MPU6886_RA_EXT_SENS_DATA_14 0x57
#define MPU6886_RA_EXT_SENS_DATA_15 0x58
#define MPU6886_RA_EXT_SENS_DATA_16 0x59
#define MPU6886_RA_EXT_SENS_DATA_17 0x5A
#define MPU6886_RA_EXT_SENS_DATA_18 0x5B
#define MPU6886_RA_EXT_SENS_DATA_19 0x5C
#define MPU6886_RA_EXT_SENS_DATA_20 0x5D
#define MPU6886_RA_EXT_SENS_DATA_21 0x5E
#define MPU6886_RA_EXT_SENS_DATA_22 0x5F
#define MPU6886_RA_EXT_SENS_DATA_23 0x60
#define MPU6886_RA_MOT_DETECT_STATUS    0x61
#define MPU6886_RA_I2C_SLV0_DO      0x63
#define MPU6886_RA_I2C_SLV1_DO      0x64
#define MPU6886_RA_I2C_SLV2_DO      0x65
#define MPU6886_RA_I2C_SLV3_DO      0x66
#define MPU6886_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6886_RA_SIGNAL_PATH_RESET    0x68
#define MPU6886_RA_MOT_DETECT_CTRL      0x69
#define MPU6886_RA_USER_CTRL        0x6A
#define MPU6886_RA_PWR_MGMT_1       0x6B
#define MPU6886_RA_PWR_MGMT_2       0x6C
#define MPU6886_RA_BANK_SEL         0x6D
#define MPU6886_RA_MEM_START_ADDR   0x6E
#define MPU6886_RA_MEM_R_W          0x6F
#define MPU6886_RA_DMP_CFG_1        0x70
#define MPU6886_RA_DMP_CFG_2        0x71
#define MPU6886_RA_FIFO_COUNTH      0x72
#define MPU6886_RA_FIFO_COUNTL      0x73
#define MPU6886_RA_FIFO_R_W         0x74
#define MPU6886_RA_WHO_AM_I         0x75

#define MPU6886_TC_PWR_MODE_BIT     7
#define MPU6886_TC_OFFSET_BIT       6
#define MPU6886_TC_OFFSET_LENGTH    6
#define MPU6886_TC_OTP_BNK_VLD_BIT  0

#define MPU6886_VDDIO_LEVEL_VLOGIC  0
#define MPU6886_VDDIO_LEVEL_VDD     1

#define MPU6886_CFG_EXT_SYNC_SET_BIT    5
#define MPU6886_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU6886_CFG_DLPF_CFG_BIT    2
#define MPU6886_CFG_DLPF_CFG_LENGTH 3

#define MPU6886_EXT_SYNC_DISABLED       0x0
#define MPU6886_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU6886_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU6886_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU6886_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU6886_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU6886_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU6886_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU6886_DLPF_BW_256         0x00
#define MPU6886_DLPF_BW_188         0x01
#define MPU6886_DLPF_BW_98          0x02
#define MPU6886_DLPF_BW_42          0x03
#define MPU6886_DLPF_BW_20          0x04
#define MPU6886_DLPF_BW_10          0x05
#define MPU6886_DLPF_BW_5           0x06

#define MPU6886_GCONFIG_XG_ST_BIT       7
#define MPU6886_GCONFIG_YG_ST_BIT       6
#define MPU6886_GCONFIG_ZG_ST_BIT       5
#define MPU6886_GCONFIG_FS_SEL_BIT      4
#define MPU6886_GCONFIG_FS_SEL_LENGTH   2


#define MPU6886_GYRO_FS_250         0x00
#define MPU6886_GYRO_FS_500         0x01
#define MPU6886_GYRO_FS_1000        0x02
#define MPU6886_GYRO_FS_2000        0x03

#define MPU6886_ACONFIG_XA_ST_BIT           7
#define MPU6886_ACONFIG_YA_ST_BIT           6
#define MPU6886_ACONFIG_ZA_ST_BIT           5
#define MPU6886_ACONFIG_AFS_SEL_BIT         4
#define MPU6886_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6886_ACONFIG_ACCEL_HPF_BIT       2
#define MPU6886_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU6886_ACCEL_FS_2          0x00
#define MPU6886_ACCEL_FS_4          0x01
#define MPU6886_ACCEL_FS_8          0x02
#define MPU6886_ACCEL_FS_16         0x03

#define MPU6886_DHPF_RESET          0x00
#define MPU6886_DHPF_5              0x01
#define MPU6886_DHPF_2P5            0x02
#define MPU6886_DHPF_1P25           0x03
#define MPU6886_DHPF_0P63           0x04
#define MPU6886_DHPF_HOLD           0x07

#define MPU6886_TEMP_FIFO_EN_BIT    7
#define MPU6886_XG_FIFO_EN_BIT      6
#define MPU6886_YG_FIFO_EN_BIT      5
#define MPU6886_ZG_FIFO_EN_BIT      4
#define MPU6886_ACCEL_FIFO_EN_BIT   3
#define MPU6886_SLV2_FIFO_EN_BIT    2
#define MPU6886_SLV1_FIFO_EN_BIT    1
#define MPU6886_SLV0_FIFO_EN_BIT    0

#define MPU6886_MULT_MST_EN_BIT     7
#define MPU6886_WAIT_FOR_ES_BIT     6
#define MPU6886_SLV_3_FIFO_EN_BIT   5
#define MPU6886_I2C_MST_P_NSR_BIT   4
#define MPU6886_I2C_MST_CLK_BIT     3
#define MPU6886_I2C_MST_CLK_LENGTH  4

#define MPU6886_CLOCK_DIV_348       0x0
#define MPU6886_CLOCK_DIV_333       0x1
#define MPU6886_CLOCK_DIV_320       0x2
#define MPU6886_CLOCK_DIV_308       0x3
#define MPU6886_CLOCK_DIV_296       0x4
#define MPU6886_CLOCK_DIV_286       0x5
#define MPU6886_CLOCK_DIV_276       0x6
#define MPU6886_CLOCK_DIV_267       0x7
#define MPU6886_CLOCK_DIV_258       0x8
#define MPU6886_CLOCK_DIV_500       0x9
#define MPU6886_CLOCK_DIV_471       0xA
#define MPU6886_CLOCK_DIV_444       0xB
#define MPU6886_CLOCK_DIV_421       0xC
#define MPU6886_CLOCK_DIV_400       0xD
#define MPU6886_CLOCK_DIV_381       0xE
#define MPU6886_CLOCK_DIV_364       0xF

#define MPU6886_I2C_SLV_RW_BIT      7
#define MPU6886_I2C_SLV_ADDR_BIT    6
#define MPU6886_I2C_SLV_ADDR_LENGTH 7
#define MPU6886_I2C_SLV_EN_BIT      7
#define MPU6886_I2C_SLV_BYTE_SW_BIT 6
#define MPU6886_I2C_SLV_REG_DIS_BIT 5
#define MPU6886_I2C_SLV_GRP_BIT     4
#define MPU6886_I2C_SLV_LEN_BIT     3
#define MPU6886_I2C_SLV_LEN_LENGTH  4

#define MPU6886_I2C_SLV4_RW_BIT         7
#define MPU6886_I2C_SLV4_ADDR_BIT       6
#define MPU6886_I2C_SLV4_ADDR_LENGTH    7
#define MPU6886_I2C_SLV4_EN_BIT         7
#define MPU6886_I2C_SLV4_INT_EN_BIT     6
#define MPU6886_I2C_SLV4_REG_DIS_BIT    5
#define MPU6886_I2C_SLV4_MST_DLY_BIT    4
#define MPU6886_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU6886_MST_PASS_THROUGH_BIT    7
#define MPU6886_MST_I2C_SLV4_DONE_BIT   6
#define MPU6886_MST_I2C_LOST_ARB_BIT    5
#define MPU6886_MST_I2C_SLV4_NACK_BIT   4
#define MPU6886_MST_I2C_SLV3_NACK_BIT   3
#define MPU6886_MST_I2C_SLV2_NACK_BIT   2
#define MPU6886_MST_I2C_SLV1_NACK_BIT   1
#define MPU6886_MST_I2C_SLV0_NACK_BIT   0

#define MPU6886_INTCFG_INT_LEVEL_BIT        7
#define MPU6886_INTCFG_INT_OPEN_BIT         6
#define MPU6886_INTCFG_LATCH_INT_EN_BIT     5
#define MPU6886_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU6886_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU6886_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU6886_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU6886_INTCFG_CLKOUT_EN_BIT        0

#define MPU6886_INTMODE_ACTIVEHIGH  0x00
#define MPU6886_INTMODE_ACTIVELOW   0x01

#define MPU6886_INTDRV_PUSHPULL     0x00
#define MPU6886_INTDRV_OPENDRAIN    0x01

#define MPU6886_INTLATCH_50USPULSE  0x00
#define MPU6886_INTLATCH_WAITCLEAR  0x01

#define MPU6886_INTCLEAR_STATUSREAD 0x00
#define MPU6886_INTCLEAR_ANYREAD    0x01

#define MPU6886_INTERRUPT_FF_BIT            7
#define MPU6886_INTERRUPT_MOT_BIT           6
#define MPU6886_INTERRUPT_ZMOT_BIT          5
#define MPU6886_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU6886_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU6886_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU6886_INTERRUPT_DMP_INT_BIT       1
#define MPU6886_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU6886_DMPINT_5_BIT            5
#define MPU6886_DMPINT_4_BIT            4
#define MPU6886_DMPINT_3_BIT            3
#define MPU6886_DMPINT_2_BIT            2
#define MPU6886_DMPINT_1_BIT            1
#define MPU6886_DMPINT_0_BIT            0

#define MPU6886_MOTION_MOT_XNEG_BIT     7
#define MPU6886_MOTION_MOT_XPOS_BIT     6
#define MPU6886_MOTION_MOT_YNEG_BIT     5
#define MPU6886_MOTION_MOT_YPOS_BIT     4
#define MPU6886_MOTION_MOT_ZNEG_BIT     3
#define MPU6886_MOTION_MOT_ZPOS_BIT     2
#define MPU6886_MOTION_MOT_ZRMOT_BIT    0

#define MPU6886_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU6886_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU6886_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU6886_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU6886_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU6886_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU6886_PATHRESET_GYRO_RESET_BIT    2
#define MPU6886_PATHRESET_ACCEL_RESET_BIT   1
#define MPU6886_PATHRESET_TEMP_RESET_BIT    0

#define MPU6886_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU6886_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU6886_DETECT_FF_COUNT_BIT             3
#define MPU6886_DETECT_FF_COUNT_LENGTH          2
#define MPU6886_DETECT_MOT_COUNT_BIT            1
#define MPU6886_DETECT_MOT_COUNT_LENGTH         2

#define MPU6886_DETECT_DECREMENT_RESET  0x0
#define MPU6886_DETECT_DECREMENT_1      0x1
#define MPU6886_DETECT_DECREMENT_2      0x2
#define MPU6886_DETECT_DECREMENT_4      0x3

#define MPU6886_USERCTRL_DMP_EN_BIT             7
#define MPU6886_USERCTRL_FIFO_EN_BIT            6
#define MPU6886_USERCTRL_I2C_MST_EN_BIT         5
#define MPU6886_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU6886_USERCTRL_DMP_RESET_BIT          3
#define MPU6886_USERCTRL_FIFO_RESET_BIT         2
#define MPU6886_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU6886_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU6886_PWR1_DEVICE_RESET_BIT   7
#define MPU6886_PWR1_SLEEP_BIT          6
#define MPU6886_PWR1_CYCLE_BIT          5
#define MPU6886_PWR1_TEMP_DIS_BIT       3
#define MPU6886_PWR1_CLKSEL_BIT         2
#define MPU6886_PWR1_CLKSEL_LENGTH      3

#define MPU6886_CLOCK_INTERNAL          0x00
#define MPU6886_CLOCK_AUTO              0x01
#define MPU6886_CLOCK_KEEP_RESET        0x07

#define MPU6886_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6886_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6886_PWR2_STBY_XA_BIT            5
#define MPU6886_PWR2_STBY_YA_BIT            4
#define MPU6886_PWR2_STBY_ZA_BIT            3
#define MPU6886_PWR2_STBY_XG_BIT            2
#define MPU6886_PWR2_STBY_YG_BIT            1
#define MPU6886_PWR2_STBY_ZG_BIT            0

#define MPU6886_WAKE_FREQ_1P25      0x0
#define MPU6886_WAKE_FREQ_2P5       0x1
#define MPU6886_WAKE_FREQ_5         0x2
#define MPU6886_WAKE_FREQ_10        0x3

#define MPU6886_BANKSEL_PRFTCH_EN_BIT       6
#define MPU6886_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU6886_BANKSEL_MEM_SEL_BIT         4
#define MPU6886_BANKSEL_MEM_SEL_LENGTH      5

#define MPU6886_WHO_AM_I_BIT        6
#define MPU6886_WHO_AM_I_LENGTH     6

#define MPU6886_DMP_MEMORY_BANKS        8
#define MPU6886_DMP_MEMORY_BANK_SIZE    256
#define MPU6886_DMP_MEMORY_CHUNK_SIZE   16

#define MPU6886_DEG_PER_LSB_250  (float)((2 * 250.0) / 65536.0)
#define MPU6886_DEG_PER_LSB_500  (float)((2 * 500.0) / 65536.0)
#define MPU6886_DEG_PER_LSB_1000 (float)((2 * 1000.0) / 65536.0)
#define MPU6886_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)

#define MPU6886_G_PER_LSB_2      (float)((2 * 2) / 65536.0)
#define MPU6886_G_PER_LSB_4      (float)((2 * 4) / 65536.0)
#define MPU6886_G_PER_LSB_8      (float)((2 * 8) / 65536.0)
#define MPU6886_G_PER_LSB_16     (float)((2 * 16) / 65536.0)

#define MPU6886_ST_GYRO_LOW      10.0   // deg/s
#define MPU6886_ST_GYRO_HIGH     105.0  // deg/s
#define MPU6886_ST_ACCEL_LOW     0.300  // G
#define MPU6886_ST_ACCEL_HIGH    0.950  // G

// note: DMP code memory blocks defined at end of header file

int mpu6886Init(I2C_Dev *i2cPort);
bool mpu6886Test(void);

bool mpu6886TestConnection();
bool mpu6886EvaluateSelfTest(float low, float high, float value, char *string);
bool mpu6886SelfTest();


// AUX_VDDIO register
uint8_t mpu6886GetAuxVDDIOLevel();
void mpu6886SetAuxVDDIOLevel(uint8_t level);

// SMPLRT_DIV register
uint8_t mpu6886GetRate();
void mpu6886SetRate(uint8_t rate);

// CONFIG register
uint8_t mpu6886GetExternalFrameSync();
void mpu6886SetExternalFrameSync(uint8_t sync);
uint8_t mpu6886GetDLPFMode();
void mpu6886SetDLPFMode(uint8_t bandwidth);

// GYRO_CONFIG register
void mpu6886SetGyroXSelfTest(bool enabled);
void mpu6886SetGyroYSelfTest(bool enabled);
void mpu6886SetGyroZSelfTest(bool enabled);
uint8_t mpu6886GetFullScaleGyroRangeId();
float mpu6886GetFullScaleGyroDPL();
void mpu6886SetFullScaleGyroRange(uint8_t range);

// ACCEL_CONFIG register
bool mpu6886GetAccelXSelfTest();
void mpu6886SetAccelXSelfTest(bool enabled);
bool mpu6886GetAccelYSelfTest();
void mpu6886SetAccelYSelfTest(bool enabled);
bool mpu6886GetAccelZSelfTest();
void mpu6886SetAccelZSelfTest(bool enabled);
uint8_t mpu6886GetFullScaleAccelRangeId();
void mpu6886SetFullScaleAccelRange(uint8_t range);
float mpu6886GetFullScaleAccelGPL();
uint8_t mpu6886GetDHPFMode();
void mpu6886SetDHPFMode(uint8_t mode);

// FF_THR register
uint8_t mpu6886GetFreefallDetectionThreshold();
void mpu6886SetFreefallDetectionThreshold(uint8_t threshold);

// FF_DUR register
uint8_t mpu6886GetFreefallDetectionDuration();
void mpu6886SetFreefallDetectionDuration(uint8_t duration);

// MOT_THR register
uint8_t mpu6886GetMotionDetectionThreshold();
void mpu6886SetMotionDetectionThreshold(uint8_t threshold);

// MOT_DUR register
uint8_t mpu6886GetMotionDetectionDuration();
void mpu6886SetMotionDetectionDuration(uint8_t duration);

// ZRMOT_THR register
uint8_t mpu6886GetZeroMotionDetectionThreshold();
void mpu6886SetZeroMotionDetectionThreshold(uint8_t threshold);

// ZRMOT_DUR register
uint8_t mpu6886GetZeroMotionDetectionDuration();
void mpu6886SetZeroMotionDetectionDuration(uint8_t duration);

// FIFO_EN register
bool mpu6886GetTempFIFOEnabled();
void mpu6886SetTempFIFOEnabled(bool enabled);
bool mpu6886GetXGyroFIFOEnabled();
void mpu6886SetXGyroFIFOEnabled(bool enabled);
bool mpu6886GetYGyroFIFOEnabled();
void mpu6886SetYGyroFIFOEnabled(bool enabled);
bool mpu6886GetZGyroFIFOEnabled();
void mpu6886SetZGyroFIFOEnabled(bool enabled);
bool mpu6886GetAccelFIFOEnabled();
void mpu6886SetAccelFIFOEnabled(bool enabled);
bool mpu6886GetSlave2FIFOEnabled();
void mpu6886SetSlave2FIFOEnabled(bool enabled);
bool mpu6886GetSlave1FIFOEnabled();
void mpu6886SetSlave1FIFOEnabled(bool enabled);
bool mpu6886GetSlave0FIFOEnabled();
void mpu6886SetSlave0FIFOEnabled(bool enabled);

// I2C_MST_CTRL register
bool mpu6886GetMultiMasterEnabled();
void mpu6886SetMultiMasterEnabled(bool enabled);
bool mpu6886GetWaitForExternalSensorEnabled();
void mpu6886SetWaitForExternalSensorEnabled(bool enabled);
bool mpu6886GetSlave3FIFOEnabled();
void mpu6886SetSlave3FIFOEnabled(bool enabled);
bool mpu6886GetSlaveReadWriteTransitionEnabled();
void mpu6886SetSlaveReadWriteTransitionEnabled(bool enabled);
uint8_t mpu6886GetMasterClockSpeed();
void mpu6886SetMasterClockSpeed(uint8_t speed);

// I2C_SLV* registers (Slave 0-3)
uint8_t mpu6886GetSlaveAddress(uint8_t num);
void mpu6886SetSlaveAddress(uint8_t num, uint8_t address);
uint8_t mpu6886GetSlaveRegister(uint8_t num);
void mpu6886SetSlaveRegister(uint8_t num, uint8_t reg);
bool mpu6886GetSlaveEnabled(uint8_t num);
void mpu6886SetSlaveEnabled(uint8_t num, bool enabled);
bool mpu6886GetSlaveWordByteSwap(uint8_t num);
void mpu6886SetSlaveWordByteSwap(uint8_t num, bool enabled);
bool mpu6886GetSlaveWriteMode(uint8_t num);
void mpu6886SetSlaveWriteMode(uint8_t num, bool mode);
bool mpu6886GetSlaveWordGroupOffmpu6886Set(uint8_t num);
void setSlaveWordGroupOffset(uint8_t num, bool enabled);
uint8_t mpu6886GetSlaveDataLength(uint8_t num);
void mpu6886SetSlaveDataLength(uint8_t num, uint8_t length);

// I2C_SLV* registers (Slave 4)
uint8_t mpu6886GetSlave4Address();
void mpu6886SetSlave4Address(uint8_t address);
uint8_t mpu6886GetSlave4Register();
void mpu6886SetSlave4Register(uint8_t reg);
void mpu6886SetSlave4OutputByte(uint8_t data);
bool mpu6886GetSlave4Enabled();
void mpu6886SetSlave4Enabled(bool enabled);
bool mpu6886GetSlave4InterruptEnabled();
void mpu6886SetSlave4InterruptEnabled(bool enabled);
bool mpu6886GetSlave4WriteMode();
void mpu6886SetSlave4WriteMode(bool mode);
uint8_t mpu6886GetSlave4MasterDelay();
void mpu6886SetSlave4MasterDelay(uint8_t delay);
uint8_t mpu6886GetSlate4InputByte();

// I2C_MST_STATUS register
bool mpu6886GetPassthroughStatus();
bool mpu6886GetSlave4IsDone();
bool mpu6886GetLostArbitration();
bool mpu6886GetSlave4Nack();
bool mpu6886GetSlave3Nack();
bool mpu6886GetSlave2Nack();
bool mpu6886GetSlave1Nack();
bool mpu6886GetSlave0Nack();

// INT_PIN_CFG register
bool mpu6886GetInterruptMode();
void mpu6886SetInterruptMode(bool mode);
bool mpu6886GetInterruptDrive();
void mpu6886SetInterruptDrive(bool drive);
bool mpu6886GetInterruptLatch();
void mpu6886SetInterruptLatch(bool latch);
bool mpu6886GetInterruptLatchClear();
void mpu6886SetInterruptLatchClear(bool clear);
bool mpu6886GetFSyncInterruptLevel();
void mpu6886SetFSyncInterruptLevel(bool level);
bool mpu6886GetFSyncInterruptEnabled();
void mpu6886SetFSyncInterruptEnabled(bool enabled);
bool mpu6886GetI2CBypassEnabled();
void mpu6886SetI2CBypassEnabled(bool enabled);
bool mpu6886GetClockOutputEnabled();
void mpu6886SetClockOutputEnabled(bool enabled);

// INT_ENABLE register
uint8_t mpu6886GetIntEnabled();
void mpu6886SetIntEnabled(uint8_t enabled);
bool mpu6886GetIntFreefallEnabled();
void mpu6886SetIntFreefallEnabled(bool enabled);
bool mpu6886GetIntMotionEnabled();
void mpu6886SetIntMotionEnabled(bool enabled);
bool mpu6886GetIntZeroMotionEnabled();
void mpu6886SetIntZeroMotionEnabled(bool enabled);
bool mpu6886GetIntFIFOBufferOverflowEnabled();
void mpu6886SetIntFIFOBufferOverflowEnabled(bool enabled);
bool mpu6886GetIntI2CMasterEnabled();
void mpu6886SetIntI2CMasterEnabled(bool enabled);
bool mpu6886GetIntDataReadyEnabled();
void mpu6886SetIntDataReadyEnabled(bool enabled);

// INT_STATUS register
uint8_t mpu6886GetIntStatus();
bool mpu6886GetIntFreefallStatus();
bool mpu6886GetIntMotionStatus();
bool mpu6886GetIntZeroMotionStatus();
bool mpu6886GetIntFIFOBufferOverflowStatus();
bool mpu6886GetIntI2CMasterStatus();
bool mpu6886GetIntDataReadyStatus();

// ACCEL_*OUT_* registers
void mpu6886GetMotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);
void mpu6886GetMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void mpu6886GetAcceleration(int16_t *x, int16_t *y, int16_t *z);
int16_t mpu6886GetAccelerationX();
int16_t mpu6886GetAccelerationY();
int16_t mpu6886GetAccelerationZ();

// TEMP_OUT_* registers
int16_t mpu6886GetTemperature();

// GYRO_*OUT_* registers
void mpu6886GetRotation(int16_t *x, int16_t *y, int16_t *z);
int16_t mpu6886GetRotationX();
int16_t mpu6886GetRotationY();
int16_t mpu6886GetRotationZ();

// EXT_SENS_DATA_* registers
uint8_t mpu6886GetExternalSensorByte(int position);
uint16_t mpu6886GetExternalSensorWord(int position);
uint32_t mpu6886GetExternalSensorDWord(int position);

// MOT_DETECT_STATUS register
bool mpu6886GetXNegMotionDetected();
bool mpu6886GetXPosMotionDetected();
bool mpu6886GetYNegMotionDetected();
bool mpu6886GetYPosMotionDetected();
bool mpu6886GetZNegMotionDetected();
bool mpu6886GetZPosMotionDetected();
bool mpu6886GetZeroMotionDetected();

// I2C_SLV*_DO register
void mpu6886SetSlaveOutputByte(uint8_t num, uint8_t data);

// I2C_MST_DELAY_CTRL register
bool mpu6886GetExternalShadowDelayEnabled();
void mpu6886SetExternalShadowDelayEnabled(bool enabled);
bool mpu6886GetSlaveDelayEnabled(uint8_t num);
void mpu6886SetSlaveDelayEnabled(uint8_t num, bool enabled);

// SIGNAL_PATH_RESET register
void rempu6886SetGyroscopePath();
void rempu6886SetAccelerometerPath();
void rempu6886SetTemperaturePath();

// MOT_DETECT_CTRL register
uint8_t mpu6886GetAccelerometerPowerOnDelay();
void mpu6886SetAccelerometerPowerOnDelay(uint8_t delay);
uint8_t mpu6886GetFreefallDetectionCounterDecrement();
void mpu6886SetFreefallDetectionCounterDecrement(uint8_t decrement);
uint8_t mpu6886GetMotionDetectionCounterDecrement();
void mpu6886SetMotionDetectionCounterDecrement(uint8_t decrement);

// USER_CTRL register
bool mpu6886GetFIFOEnabled();
void mpu6886SetFIFOEnabled(bool enabled);
bool mpu6886GetI2CMasterModeEnabled();
void mpu6886SetI2CMasterModeEnabled(bool enabled);
void mpu6886SwitchSPIEnabled(bool enabled);
void mpu6886ResetFIFO();
void mpu6886ResetI2CMaster();
void mpu6886ResetSensors();

// PWR_MGMT_1 register
void mpu6886Reset();
bool mpu6886GetSleepEnabled();
void mpu6886SetSleepEnabled(bool enabled);
bool mpu6886GetWakeCycleEnabled();
void mpu6886SetWakeCycleEnabled(bool enabled);
bool mpu6886GetTempSensorEnabled();
void mpu6886SetTempSensorEnabled(bool enabled);
uint8_t mpu6886GetClockSource();
void mpu6886SetClockSource(uint8_t source);

// PWR_MGMT_2 register
uint8_t mpu6886GetWakeFrequency();
void mpu6886SetWakeFrequency(uint8_t frequency);
bool mpu6886GetStandbyXAccelEnabled();
void mpu6886SetStandbyXAccelEnabled(bool enabled);
bool mpu6886GetStandbyYAccelEnabled();
void mpu6886SetStandbyYAccelEnabled(bool enabled);
bool mpu6886GetStandbyZAccelEnabled();
void mpu6886SetStandbyZAccelEnabled(bool enabled);
bool mpu6886GetStandbyXGyroEnabled();
void mpu6886SetStandbyXGyroEnabled(bool enabled);
bool mpu6886GetStandbyYGyroEnabled();
void mpu6886SetStandbyYGyroEnabled(bool enabled);
bool mpu6886GetStandbyZGyroEnabled();
void mpu6886SetStandbyZGyroEnabled(bool enabled);

// FIFO_COUNT_* registers
uint16_t mpu6886GetFIFOCount();

// FIFO_R_W register
uint8_t mpu6886GetFIFOByte();
void mpu6886SetFIFOByte(uint8_t data);
void mpu6886GetFIFOBytes(uint8_t *data, uint8_t length);

// WHO_AM_I register
uint8_t mpu6886GetDeviceID();
void mpu6886SetDeviceID(uint8_t id);

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

// XG_OFFS_TC register
uint8_t mpu6886GetOTPBankValid();
void mpu6886SetOTPBankValid(bool enabled);
int8_t mpu6886GetXGyroOffset();
void mpu6886SetXGyroOffset(int8_t offset);

// YG_OFFS_TC register
int8_t mpu6886GetYGyroOffset();
void mpu6886SetYGyroOffset(int8_t offset);

// ZG_OFFS_TC register
int8_t mpu6886GetZGyroOffset();
void  mpu6886SetGyroOffset(int8_t offset);

// X_FINE_GAIN register
int8_t mpu6886GetXFineGain();
void mpu6886SetXFineGain(int8_t gain);

// Y_FINE_GAIN register
int8_t mpu6886GetYFineGain();
void mpu6886SetYFineGain(int8_t gain);

// Z_FINE_GAIN register
int8_t mpu6886GetZFineGain();
void mpu6886SetZFineGain(int8_t gain);

// XA_OFFS_* registers
int16_t mpu6886GetXAccelOffset();
void mpu6886SetXAccelOffset(int16_t offset);

// YA_OFFS_* register
int16_t mpu6886GetYAccelOffset();
void mpu6886SetYAccelOffset(int16_t offset);

// ZA_OFFS_* register
int16_t mpu6886GetZAccelOffset();
void mpu6886SetZAccelOffset(int16_t offset);

// XG_OFFS_USR* registers
int16_t mpu6886GetXGyroOffsetUser();
void mpu6886SetXGyroOffsetUser(int16_t offset);

// YG_OFFS_USR* register
int16_t mpu6886GetYGyroOffsetUser();
void mpu6886SetYGyroOffsetUser(int16_t offset);

// ZG_OFFS_USR* register
int16_t mpu6886GetZGyroOffsetUser();
void mpu6886SetZGyroOffsetUser(int16_t offset);

// INT_ENABLE register (DMP functions)
bool mpu6886GetIntPLLReadyEnabled();
void mpu6886SetIntPLLReadyEnabled(bool enabled);
bool mpu6886GetIntDMPEnabled();
void mpu6886SetIntDMPEnabled(bool enabled);

// DMP_INT_STATUS
bool mpu6886GetDMPInt5Status();
bool mpu6886GetDMPInt4Status();
bool mpu6886GetDMPInt3Status();
bool mpu6886GetDMPInt2Status();
bool mpu6886GetDMPInt1Status();
bool mpu6886GetDMPInt0Status();

// INT_STATUS register (DMP functions)
bool mpu6886GetIntPLLReadyStatus();
bool mpu6886GetIntDMPStatus();

// USER_CTRL register (DMP functions)
bool mpu6886GetDMPEnabled();
void mpu6886SetDMPEnabled(bool enabled);
void mpu6886ResetDMP();

// BANK_SEL register
void mpu6886SetMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank);

// MEM_START_ADDR register
void mpu6886SetMemoryStartAddress(uint8_t address);

// MEM_R_W register
uint8_t mpu6886ReadMemoryByte();
void mpu6886WriteMemoryByte(uint8_t data);
void mpu6886ReadMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
bool mpu6886WriteMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);
bool mpu6886WriteProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);

bool mpu6886WriteDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);
bool mpu6886WiteProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

// DMP_CFG_1 register
uint8_t mpu6886GetDMPConfig1();
void mpu6886SetDMPConfig1(uint8_t config);

// DMP_CFG_2 register
uint8_t mpu6886GetDMPConfig2();
void mpu6886SetDMPConfig2(uint8_t config);


#ifdef MPU6886_INCLUDE_DMP_MOTIONAPPS20
/* This is only included if you want it, since it eats about 2K of program
 * memory, which is a waste if you aren't using the DMP (or if you aren't
 * using this particular flavor of DMP).
 *
 * Source is from the InvenSense MotionApps v2 demo code. Original source is
 * unavailable, unless you happen to be amazing as decompiling binary by
 * hand (in which case, please contact me, and I'm totally serious).
 *
 * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
 * DMP reverse-engineering he did to help make this bit of wizardry
 * possible.
 */

#define MPU6886_DMP_CODE_SIZE 1929

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
const prog_uchar dmpMemory[MPU6886_DMP_CODE_SIZE] PROGMEM = {
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 138 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

#define MPU6886_DMP_CONFIG_SIZE 192

// thanks to Noah Zerkin for piecing this stuff together!
const prog_uchar dmpConfig[MPU6886_DMP_CONFIG_SIZE] PROGMEM = {
    //  BANK    OFFSET  LENGTH  [DATA]
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,         // FCFG_1 inv_set_gyro_calibration
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,         // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,   // D_0_104 inv_set_gyro_calibration
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,   // D_0_24 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // D_1_152 inv_set_accel_calibration
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,         // FCFG_7 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x20, 0x00,               // D_0_108 inv_set_accel_calibration
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_22
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,   // D_1_236 inv_apply_endian_accel
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,   // FCFG_5 inv_set_bias_update
    0x00,   0xA3,   0x01,   0x00,                     // D_0_163 inv_set_dead_zone
    // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07,   0x86,   0x01,   0xFE,                     // CFG_6 inv_set_fifo_interupt
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07,   0x7E,   0x01,   0x30,                     // CFG_16 inv_set_footer
    0x07,   0x46,   0x01,   0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x0A                // D_0_22 inv_set_fifo_rate
};

#endif

#endif /* _MPU6886_H_ */
