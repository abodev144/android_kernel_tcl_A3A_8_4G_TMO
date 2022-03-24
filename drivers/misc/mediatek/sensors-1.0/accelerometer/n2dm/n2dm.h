/* linux/drivers/hwmon/N2DM.c
 *
 * (C) Copyright 2008
 * MediaTek <www.mediatek.com>
 *
 * N2DM driver for MT6xxx
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef N2DM_H
#define N2DM_H

#include <linux/ioctl.h>

extern struct acc_hw *n2dm_get_cust_acc_hw(void) ;

#define N2DM_I2C_SLAVE_ADDR     0x10//0x10<-> SD0=GND;   0x12<-> SD0=High

/* N2DM Register Map  (Please refer to N2DM Specifications) */
#define N2DM_REG_CTL_REG1       0x20
#define N2DM_REG_CTL_REG2       0x21
#define N2DM_REG_CTL_REG3       0x22
#define N2DM_REG_CTL_REG4       0x23
#define N2DM_REG_DATAX0         0x28
#define N2DM_REG_OUT_X          0x28
#define N2DM_REG_OUT_Y          0x2A
#define N2DM_REG_OUT_Z          0x2C


#define N2DM_REG_DEVID          0x0F //added by lanying.he 

/*
#define N2DM_REG_DEVID          0x00
#define N2DM_REG_THRESH_TAP     0x1D
#define N2DM_REG_OFSX           0x1E
#define N2DM_REG_OFSY           0x1F
#define N2DM_REG_OFSZ           0x20
#define N2DM_REG_DUR            0x21
#define N2DM_REG_THRESH_ACT     0x24
#define N2DM_REG_THRESH_INACT   0x25
#define N2DM_REG_TIME_INACT     0x26
#define N2DM_REG_ACT_INACT_CTL  0x27
#define N2DM_REG_THRESH_FF      0x28
#define N2DM_REG_TIME_FF        0x29
#define N2DM_REG_TAP_AXES       0x2A
#define N2DM_REG_ACT_TAP_STATUS 0x2B
#define N2DM_REG_BW_RATE        0x2C
#define N2DM_REG_POWER_CTL      0x2D
#define N2DM_REG_INT_ENABLE     0x2E
#define N2DM_REG_INT_MAP        0x2F
#define N2DM_REG_INT_SOURCE     0x30
#define N2DM_REG_DATA_FORMAT    0x31
#define N2DM_REG_DATAX0         0x32
#define N2DM_REG_FIFO_CTL       0x38
#define N2DM_REG_FIFO_STATUS    0x39
*/
//Added by yuduan.xie for significant begin
#define KXTJ2_1009_REG_POWER_CTL       0x30

#define KXTJ2_1009_MOTION_DETECT       0x0A

#define N2DM_WAKEUP_THRESHOLD          0x32
#define N2DM_WAKEUP_TIMER              0x33

//#define KXTJ2_1009_INT_SOURCE2_XYZ   0x17
#define KXTJ2_1009_INT_SOURCE1         0x31
#define KXTJ2_1009_INT_SOURCE2         0x35
#define KXTJ2_1009_INT_REG             0x1a
//Added by yuduan.xie for significant end

#define N2DM_FIXED_DEVID               0x33

#define N2DM_BW_200HZ           0x60
#define N2DM_BW_100HZ           0x50 //400 or 100 on other choise //changed
#define N2DM_BW_50HZ            0x40

#define N2DM_FULLRANG_LSB       0XFF

#define N2DM_MEASURE_MODE       0x08    //changed 
#define N2DM_DATA_READY         0x07    //changed

//#define N2DM_FULL_RES         0x08
#define N2DM_RANGE_2G           0x00
#define N2DM_RANGE_4G           0x10
#define N2DM_RANGE_8G           0x20 //8g or 2g no ohter choise//changed
//#define N2DM_RANGE_16G        0x30 //8g or 2g no ohter choise//changed

#define N2DM_SELF_TEST          0x10 //changed

#define N2DM_STREAM_MODE        0x80
#define N2DM_SAMPLES_15         0x0F

#define N2DM_FS_8G_LSB_G        0x20
#define N2DM_FS_4G_LSB_G        0x10
#define N2DM_FS_2G_LSB_G        0x00

#define N2DM_LEFT_JUSTIFY       0x04
#define N2DM_RIGHT_JUSTIFY      0x00


#define N2DM_SUCCESS                 0
#define N2DM_ERR_I2C                -1
#define N2DM_ERR_STATUS             -3
#define N2DM_ERR_SETUP_FAILURE      -4
#define N2DM_ERR_GETGSENSORDATA     -5
#define N2DM_ERR_IDENTIFICATION     -6

#define N2DM_BUFSIZE                256

#endif

