/* drivers/i2c/chips/n2dm.c - N2DM motion sensor driver
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <cust_acc.h>
#include "n2dm.h"
#include <accel.h>

#ifdef CUSTOM_KERNEL_SENSORHUB
    #include <SCP_sensorHub.h>
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

//added by lanying.he for significant begin
#include "hwmsen_helper.h"
#include "step_counter.h"
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinctrl-state.h>
//added by lanying.he for significant end
#define POWER_NONE_MACRO MT65XX_POWER_NONE

/*----------------------------------------------------------------------------*/
//#define I2C_DRIVERID_N2DM 345
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_N2DM_LOWPASS   /*apply low pass filter on output*/
/*----------------------------------------------------------------------------*/
#define N2DM_AXIS_X          0
#define N2DM_AXIS_Y          1
#define N2DM_AXIS_Z          2
#define N2DM_AXES_NUM        3
#define N2DM_DATA_LEN        6
#define N2DM_DEV_NAME        "N2DM"
/*----------------------------------------------------------------------------*/

/***************************SIGNIFICANT***************************/
//added by lanying.he for significant begin
#define SIGNIFICANT
//added by lanying.he for significant end
#ifdef SIGNIFICANT
    int n2dm_set_acc_interrupter(bool flag);
    int n2dm_setup_eint(void);
    int n2dm_acc_init_flag = -1;
    static void eintWork(struct work_struct *work);
    static bool acc_enable_status = false;
    static bool significant_enable_status = false;
#endif
/***************************SIGNIFICANT***************************/

struct acc_hw accel_cust_n2dm;

static struct acc_hw *hw = &accel_cust_n2dm; //modify by lanying.he

/* For  driver get cust info */
struct acc_hw *get_cust_acc_n2dm(void)
{
    return &accel_cust_n2dm;  //modify by lanying.he
}

static const struct i2c_device_id n2dm_i2c_id[] = {{N2DM_DEV_NAME, 0}, {}};
/*the adapter id will be available in customization*/
//static struct i2c_board_info __initdata i2c_N2DM={ I2C_BOARD_INFO("N2DM", (0x10>>1))};

//static unsigned short n2dm_force[] = {0x00, N2DM_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const n2dm_forces[] = { n2dm_force, NULL };
//static struct i2c_client_address_data n2dm_addr_data = { .forces = n2dm_forces,};

/*----------------------------------------------------------------------------*/
static int n2dm_i2c_probe(struct i2c_client *client,
                          const struct i2c_device_id *id);
static int n2dm_i2c_remove(struct i2c_client *client);
//static int n2dm_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
    static int n2dm_suspend(struct device *dev);
    static int n2dm_resume(struct device *client);
#endif
static bool pre_suspend; //adde by yan.gong

extern struct acc_hw *n2dm_get_cust_acc_hw(void);

static int  n2dm_local_init(void);
static int  n2dm_remove(void);
/*Add-Begin by zhikui.li@tcl.com,Remove unused variables*/
//static int n2dm_init_flag =-1; // 0<==>OK -1 <==> fail
/*Add-end by zhikui.li@tcl.com,Remove unused variables*/
#ifdef CONFIG_TCT_ROBUST
extern int robust_accel_i2c_check_status;
#endif

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI    = 0X08,
    ADX_TRC_INFO    = 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor {
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][N2DM_AXES_NUM];
    int sum[N2DM_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
static struct acc_init_info n2dm_init_info = {
    .name = "n2dm",
    .init = n2dm_local_init,
    .uninit = n2dm_remove,
};
/*----------------------------------------------------------------------------*/
struct n2dm_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
#ifdef SIGNIFICANT
    struct      device_node *irq_node;
    int         irq;
    struct work_struct eint_work;
#endif
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;
    s16                     cali_sw[N2DM_AXES_NUM + 1];

    /*data*/
    s8                      offset[N2DM_AXES_NUM + 1]; /*+1: for 4-byte alignment*/
    s16                     data[N2DM_AXES_NUM + 1];

#if defined(CONFIG_N2DM_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif
    /*early suspend*/
#ifdef USE_EARLY_SUSPEND
    struct early_suspend    early_drv;
#endif
};

static const struct of_device_id accel_of_match[] = {
    {.compatible = "mediatek,n2dm"},
    {},
};
/*----------------------------------------------------------------------------*/
#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops n2dm_acc_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(n2dm_suspend, n2dm_resume)
};
#endif
static struct i2c_driver n2dm_i2c_driver = {
    .driver = {
        //        .owner          = THIS_MODULE,
        .name           = N2DM_DEV_NAME,
        .of_match_table = accel_of_match,

#ifdef CONFIG_PM_SLEEP
        .pm  = &n2dm_acc_pm_ops,
#endif
    },
    .probe             = n2dm_i2c_probe,
    //.detect               = n2dm_i2c_detect,
    .remove                = n2dm_i2c_remove,
    .id_table = n2dm_i2c_id,
    //  .address_data = &n2dm_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *n2dm_i2c_client = NULL;
static struct n2dm_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
//static char selftestRes[10] = {0};
static DEFINE_MUTEX(n2dm_i2c_mutex);
static DEFINE_MUTEX(n2dm_op_mutex);
static bool enable_status = false;
static int sensor_suspend = 0;

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_ERR GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution n2dm_data_resolution[] = {
    /* combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1024},   // dataformat +/-2g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*2*1000)/(2^12);  1024 = (2^12)/(2*2)
    {{ 1, 9}, 512},   // dataformat +/-4g  in 12-bit resolution;  { 1, 9} = 1.9 = (2*4*1000)/(2^12);  512 = (2^12)/(2*4)
    {{ 3, 9}, 256},   // dataformat +/-8g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*8*1000)/(2^12);  1024 = (2^12)/(2*8)
};
/*----------------------------------------------------------------------------*/
static struct data_resolution n2dm_offset_resolution = {{15, 6}, 64};

/*--------------------read function----------------------------------*/
static int n2dm_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data,
                               u8 len)
{
    u8 beg = addr;
    int err;
    struct i2c_msg msgs[2] = {{0}, {0}};
    mutex_lock(&n2dm_i2c_mutex);
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &beg;
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    if (!client) {
        mutex_unlock(&n2dm_i2c_mutex);
        return -EINVAL;
    } else if (len > C_I2C_FIFO_SIZE) {
        GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&n2dm_i2c_mutex);
        return -EINVAL;
    }
    err = i2c_transfer(client->adapter, msgs, sizeof(msgs) / sizeof(msgs[0]));
    //GSE_LOG(" n2dm_i2c_read_block return value  %d\n", err);
    if (err < 0) {
        GSE_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
        err = -EIO;
    } else
        err = 0;
    mutex_unlock(&n2dm_i2c_mutex);
    return err; //if success will return 0
}

static int n2dm_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data,
                                u8 len)
{
    /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    char buf[C_I2C_FIFO_SIZE];
    err = 0;
    mutex_lock(&n2dm_i2c_mutex);
    if (!client) {
        mutex_unlock(&n2dm_i2c_mutex);
        return -EINVAL;
    } else if (len >= C_I2C_FIFO_SIZE) {
        GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&n2dm_i2c_mutex);
        return -EINVAL;
    }
    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
        buf[num++] = data[idx];
    err = i2c_master_send(client, buf, num);
    if (err < 0) {
        GSE_ERR("send command error!!\n");
        mutex_unlock(&n2dm_i2c_mutex);
        return -EFAULT;
    }
    mutex_unlock(&n2dm_i2c_mutex);
    return err; //if success will return transfer lenth
}
/*----------------------------------------------------------------------------*/

static void dumpReg(struct i2c_client *client)
{
    int i = 0;
    u8 addr = 0x20;
    u8 regdata = 0;
    for(i = 0; i < 3 ; i++) {
        //dump all
        n2dm_i2c_read_block(client, addr, &regdata, 1);
        GSE_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
        addr++;
    }
}
/*--------------------ADXL power control function----------------------------------*/
static void N2DM_power(struct acc_hw *hw, unsigned int on)
{
#if 0
    static unsigned int power_on = 0;
    if(hw->power_id != POWER_NONE_MACRO) {      // have externel LDO
        GSE_LOG("power %s\n", on ? "on" : "off");
        if(power_on == on)  // power status not change
            GSE_LOG("ignore power control: %d\n", on);
        else if(on) { // power on
            if(!hwPowerOn(hw->power_id, hw->power_vol, "N2DM"))
                GSE_ERR("power on fails!!\n");
        } else { // power off
            if (!hwPowerDown(hw->power_id, "N2DM"))
                GSE_ERR("power off fail!!\n");
        }
    }
    power_on = on;
#endif
}
/*----------------------------------------------------------------------------*/
static int N2DM_SetDataResolution(struct n2dm_i2c_data *obj)
{
    int err;
    u8  dat, reso;
    if((err = n2dm_i2c_read_block(obj->client, N2DM_REG_CTL_REG4, &dat,
                                  0x01)) < 0) {
        GSE_ERR("write data format fail!!\n");
        return err;
    }
    /*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
    reso  = (dat & 0x30) >> 4;
    if(reso >= 0x3)
        reso = 0x2;
    if(reso < sizeof(n2dm_data_resolution) / sizeof(n2dm_data_resolution[0])) {
        obj->reso = &n2dm_data_resolution[reso];
        return 0;
    } else
        return -EINVAL;
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadData(struct i2c_client *client, s16 data[N2DM_AXES_NUM])
{
    struct n2dm_i2c_data *priv = i2c_get_clientdata(client);
    //u8 addr = N2DM_REG_DATAX0;
    u8 buf[N2DM_DATA_LEN] = {0};
    int err = 0;
    if(NULL == client)
        err = -EINVAL;
    else {
        if((n2dm_i2c_read_block(client, N2DM_REG_OUT_X, buf, 0x01)) < 0) {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }
        if((n2dm_i2c_read_block(client, N2DM_REG_OUT_X + 1, &buf[1], 0x01)) < 0) {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }
        data[N2DM_AXIS_X] = (s16)((buf[0] + (buf[1] << 8)) >> 4);
        if((n2dm_i2c_read_block(client, N2DM_REG_OUT_Y, &buf[2], 0x01)) < 0) {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }
        if((n2dm_i2c_read_block(client, N2DM_REG_OUT_Y + 1, &buf[3], 0x01)) < 0) {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }
        data[N2DM_AXIS_Y] =  (s16)((s16)(buf[2] + ( buf[3] << 8)) >> 4);
        if((n2dm_i2c_read_block(client, N2DM_REG_OUT_Z, &buf[4], 0x01)) < 0) {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }
        if((n2dm_i2c_read_block(client, N2DM_REG_OUT_Z + 1, &buf[5], 0x01)) < 0) {
            GSE_ERR("read  G sensor data register err!\n");
            return -1;
        }
        data[N2DM_AXIS_Z] = (s16)((buf[4] + (buf[5] << 8)) >> 4);
        //GSE_LOG("[%08X %08X %08X %08x %08x %08x]\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
        data[N2DM_AXIS_X] &= 0xfff;
        data[N2DM_AXIS_Y] &= 0xfff;
        data[N2DM_AXIS_Z] &= 0xfff;
        if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA) {
            GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[N2DM_AXIS_X],
                    data[N2DM_AXIS_Y], data[N2DM_AXIS_Z],
                    data[N2DM_AXIS_X], data[N2DM_AXIS_Y], data[N2DM_AXIS_Z]);
        }
        if(data[N2DM_AXIS_X] & 0x800) {
            data[N2DM_AXIS_X] = ~data[N2DM_AXIS_X];
            data[N2DM_AXIS_X] &= 0xfff;
            data[N2DM_AXIS_X] += 1;
            data[N2DM_AXIS_X] = -data[N2DM_AXIS_X];
        }
        if(data[N2DM_AXIS_Y] & 0x800) {
            data[N2DM_AXIS_Y] = ~data[N2DM_AXIS_Y];
            data[N2DM_AXIS_Y] &= 0xfff;
            data[N2DM_AXIS_Y] += 1;
            data[N2DM_AXIS_Y] = -data[N2DM_AXIS_Y];
        }
        if(data[N2DM_AXIS_Z] & 0x800) {
            data[N2DM_AXIS_Z] = ~data[N2DM_AXIS_Z];
            data[N2DM_AXIS_Z] &= 0xfff;
            data[N2DM_AXIS_Z] += 1;
            data[N2DM_AXIS_Z] = -data[N2DM_AXIS_Z];
        }
        if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA) {
            GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[N2DM_AXIS_X],
                    data[N2DM_AXIS_Y], data[N2DM_AXIS_Z],
                    data[N2DM_AXIS_X], data[N2DM_AXIS_Y], data[N2DM_AXIS_Z]);
        }
#ifdef CONFIG_N2DM_LOWPASS
        if(atomic_read(&priv->filter)) {
            if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend)) {
                int idx, firlen = atomic_read(&priv->firlen);
                if(priv->fir.num < firlen) {
                    priv->fir.raw[priv->fir.num][N2DM_AXIS_X] = data[N2DM_AXIS_X];
                    priv->fir.raw[priv->fir.num][N2DM_AXIS_Y] = data[N2DM_AXIS_Y];
                    priv->fir.raw[priv->fir.num][N2DM_AXIS_Z] = data[N2DM_AXIS_Z];
                    priv->fir.sum[N2DM_AXIS_X] += data[N2DM_AXIS_X];
                    priv->fir.sum[N2DM_AXIS_Y] += data[N2DM_AXIS_Y];
                    priv->fir.sum[N2DM_AXIS_Z] += data[N2DM_AXIS_Z];
                    if(atomic_read(&priv->trace) & ADX_TRC_FILTER) {
                        GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
                                priv->fir.raw[priv->fir.num][N2DM_AXIS_X],
                                priv->fir.raw[priv->fir.num][N2DM_AXIS_Y],
                                priv->fir.raw[priv->fir.num][N2DM_AXIS_Z],
                                priv->fir.sum[N2DM_AXIS_X], priv->fir.sum[N2DM_AXIS_Y],
                                priv->fir.sum[N2DM_AXIS_Z]);
                    }
                    priv->fir.num++;
                    priv->fir.idx++;
                } else {
                    idx = priv->fir.idx % firlen;
                    priv->fir.sum[N2DM_AXIS_X] -= priv->fir.raw[idx][N2DM_AXIS_X];
                    priv->fir.sum[N2DM_AXIS_Y] -= priv->fir.raw[idx][N2DM_AXIS_Y];
                    priv->fir.sum[N2DM_AXIS_Z] -= priv->fir.raw[idx][N2DM_AXIS_Z];
                    priv->fir.raw[idx][N2DM_AXIS_X] = data[N2DM_AXIS_X];
                    priv->fir.raw[idx][N2DM_AXIS_Y] = data[N2DM_AXIS_Y];
                    priv->fir.raw[idx][N2DM_AXIS_Z] = data[N2DM_AXIS_Z];
                    priv->fir.sum[N2DM_AXIS_X] += data[N2DM_AXIS_X];
                    priv->fir.sum[N2DM_AXIS_Y] += data[N2DM_AXIS_Y];
                    priv->fir.sum[N2DM_AXIS_Z] += data[N2DM_AXIS_Z];
                    priv->fir.idx++;
                    data[N2DM_AXIS_X] = priv->fir.sum[N2DM_AXIS_X] / firlen;
                    data[N2DM_AXIS_Y] = priv->fir.sum[N2DM_AXIS_Y] / firlen;
                    data[N2DM_AXIS_Z] = priv->fir.sum[N2DM_AXIS_Z] / firlen;
                    if(atomic_read(&priv->trace) & ADX_TRC_FILTER) {
                        GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
                                priv->fir.raw[idx][N2DM_AXIS_X], priv->fir.raw[idx][N2DM_AXIS_Y],
                                priv->fir.raw[idx][N2DM_AXIS_Z],
                                priv->fir.sum[N2DM_AXIS_X], priv->fir.sum[N2DM_AXIS_Y],
                                priv->fir.sum[N2DM_AXIS_Z],
                                data[N2DM_AXIS_X], data[N2DM_AXIS_Y], data[N2DM_AXIS_Z]);
                    }
                }
            }
        }
#endif
    }
    return err;
}
/*----------------------------------------------------------------------------*/
/*
static int N2DM_ReadOffset(struct i2c_client *client, s8 ofs[N2DM_AXES_NUM])
{
    int err;

    return err;
}
*/
/*----------------------------------------------------------------------------*/
static int N2DM_ResetCalibration(struct i2c_client *client)
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
    return 0;
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadCalibration(struct i2c_client *client,
                                int dat[N2DM_AXES_NUM])
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    dat[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X] *
                                     obj->cali_sw[N2DM_AXIS_X];
    dat[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y] *
                                     obj->cali_sw[N2DM_AXIS_Y];
    dat[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z] *
                                     obj->cali_sw[N2DM_AXIS_Z];
    return 0;
}
/*----------------------------------------------------------------------------*/
/*
static int N2DM_ReadCalibrationEx(struct i2c_client *client, int act[N2DM_AXES_NUM], int raw[N2DM_AXES_NUM])
{

    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    int err;
    int mul;

    if(err = N2DM_ReadOffset(client, obj->offset))
    {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }

    mul = obj->reso->sensitivity/n2dm_offset_resolution.sensitivity;
    raw[N2DM_AXIS_X] = obj->offset[N2DM_AXIS_X]*mul + obj->cali_sw[N2DM_AXIS_X];
    raw[N2DM_AXIS_Y] = obj->offset[N2DM_AXIS_Y]*mul + obj->cali_sw[N2DM_AXIS_Y];
    raw[N2DM_AXIS_Z] = obj->offset[N2DM_AXIS_Z]*mul + obj->cali_sw[N2DM_AXIS_Z];

    act[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X]*raw[N2DM_AXIS_X];
    act[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y]*raw[N2DM_AXIS_Y];
    act[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z]*raw[N2DM_AXIS_Z];

    return 0;
}
*/
/*----------------------------------------------------------------------------*/
static int N2DM_WriteCalibration(struct i2c_client *client,
                                 int dat[N2DM_AXES_NUM])
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;
    //int cali[N2DM_AXES_NUM];
    GSE_FUN();
    if(!obj || ! dat) {
        GSE_ERR("null ptr!!\n");
        return -EINVAL;
    } else {
        s16 cali[N2DM_AXES_NUM];
        cali[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X] *
                                          obj->cali_sw[N2DM_AXIS_X];
        cali[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y] *
                                          obj->cali_sw[N2DM_AXIS_Y];
        cali[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z] *
                                          obj->cali_sw[N2DM_AXIS_Z];
        cali[N2DM_AXIS_X] += dat[N2DM_AXIS_X];
        cali[N2DM_AXIS_Y] += dat[N2DM_AXIS_Y];
        cali[N2DM_AXIS_Z] += dat[N2DM_AXIS_Z];
        obj->cali_sw[N2DM_AXIS_X] += obj->cvt.sign[N2DM_AXIS_X] *
                                     dat[obj->cvt.map[N2DM_AXIS_X]];
        obj->cali_sw[N2DM_AXIS_Y] += obj->cvt.sign[N2DM_AXIS_Y] *
                                     dat[obj->cvt.map[N2DM_AXIS_Y]];
        obj->cali_sw[N2DM_AXIS_Z] += obj->cvt.sign[N2DM_AXIS_Z] *
                                     dat[obj->cvt.map[N2DM_AXIS_Z]];
    }
    return err;
}
/*----------------------------------------------------------------------------*/

static int N2DM_CheckDeviceID(struct i2c_client *client)
{
    u8 databuf[10];
    int res = 0;
    memset(databuf, 0, sizeof(u8) * 10);
    databuf[0] = N2DM_REG_DEVID;
    res = i2c_master_send(client, databuf, 0x1);
    if(res <= 0)
        goto exit_N2DM_CheckDeviceID;
    udelay(500);
    databuf[0] = 0x0;
    res = i2c_master_recv(client, databuf, 0x01);
    if(res <= 0)
        goto exit_N2DM_CheckDeviceID;
    if(databuf[0] != N2DM_FIXED_DEVID)
        return N2DM_ERR_IDENTIFICATION;
exit_N2DM_CheckDeviceID:
    if (res <= 0)
        return N2DM_ERR_I2C;
    return N2DM_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int N2DM_SetPowerMode(struct i2c_client *client, bool enable)
{
    u8 databuf[2];
    int res = 0;
    u8 addr = N2DM_REG_CTL_REG1;
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    GSE_ERR(" lanying.he enter Sensor power status is sensor_power = %d\n",
            sensor_power);
    if(enable == sensor_power) {
        GSE_LOG("Sensor power status is newest!\n");
        return N2DM_SUCCESS;
    }
    if((n2dm_i2c_read_block(client, addr, databuf, 0x01)) < 0) {
        GSE_ERR("read power ctl register err!\n");
        return N2DM_ERR_I2C;
    }
    if(enable == true)
        databuf[0] &=  ~N2DM_MEASURE_MODE;
    else
        databuf[0] |= N2DM_MEASURE_MODE;
    res = n2dm_i2c_write_block(client, N2DM_REG_CTL_REG1, databuf, 0x1);
    if(res <= 0) {
        GSE_LOG("set power mode failed!\n");
        return N2DM_ERR_I2C;
    } else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
        GSE_LOG("set power mode ok %d!\n", databuf[1]);
    sensor_power = enable;
    GSE_ERR("lanying.he leave Sensor power status is sensor_power = %d\n",
            sensor_power);
    return N2DM_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int N2DM_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    u8 databuf[10];
    u8 addr = N2DM_REG_CTL_REG4;
    int res = 0;
    memset(databuf, 0, sizeof(u8) * 10);
    if((n2dm_i2c_read_block(client, addr, databuf, 0x01)) < 0) {
        GSE_ERR("read reg_ctl_reg4 register err!\n");
        return N2DM_ERR_I2C;
    }
    databuf[0] &= ~0x30;
    databuf[0] |= (dataformat | 0x88); // 0x88: enable BDU & HR bit
    res = n2dm_i2c_write_block(client, N2DM_REG_CTL_REG4, databuf, 0x1);
    if(res < 0)
        return N2DM_ERR_I2C;
    return N2DM_SetDataResolution(obj);
}
/*----------------------------------------------------------------------------*/
static int N2DM_SetBWRate(struct i2c_client *client, u8 bwrate)
{
    u8 databuf[10];
    u8 addr = N2DM_REG_CTL_REG1;
    int res = 0;
    memset(databuf, 0, sizeof(u8) * 10);
    if((n2dm_i2c_read_block(client, addr, databuf, 0x01)) < 0) {
        GSE_ERR("read reg_ctl_reg1 register err!\n");
        return N2DM_ERR_I2C;
    }
    databuf[0] &= ~0xF0;
    databuf[0] |= bwrate;
    res = n2dm_i2c_write_block(client, N2DM_REG_CTL_REG1, databuf, 0x1);
    if(res < 0)
        return N2DM_ERR_I2C;
    return N2DM_SUCCESS;
}
/*----------------------------------------------------------------------------*/
//enalbe data ready interrupt
static int N2DM_SetIntEnable(struct i2c_client *client, u8 intenable)
{
    u8 databuf[2];
    u8 addr = N2DM_REG_CTL_REG3;
    int res = 0;
    memset(databuf, 0, sizeof(u8) * 2);
    if((n2dm_i2c_read_block(client, addr, databuf, 0x01)) < 0) {
        GSE_ERR("read reg_ctl_reg1 register err!\n");
        return N2DM_ERR_I2C;
    }
    databuf[0] = 0x00;
    res = n2dm_i2c_write_block(client, N2DM_REG_CTL_REG3, databuf, 0x01);
    if(res < 0)
        return N2DM_ERR_I2C;
    return N2DM_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int N2DM_Init(struct i2c_client *client, int reset_cali)
{
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    int res = 0;
    u8 databuf[2] = {0, 0};
    res = N2DM_CheckDeviceID(client);
    if(res != N2DM_SUCCESS)
        return res;
    // first clear reg1
    databuf[0] = 0x07;
    res = n2dm_i2c_write_block(client, N2DM_REG_CTL_REG1, databuf, 0x01);
    if(res < 0) {
        GSE_ERR("N2DM_Init step 1!\n");
        return res;
    }
    res = N2DM_SetBWRate(client, N2DM_BW_100HZ);//400 or 100 no other choice
    if(res < 0) {
        GSE_ERR("N2DM_Init step 2!\n");
        return res;
    }
    res = N2DM_SetDataFormat(client, N2DM_RANGE_2G);//8g or 2G no oher choise
    if(res < 0) {
        GSE_ERR("N2DM_Init step 3!\n");
        return res;
    }
    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;
    res = N2DM_SetIntEnable(client, false);
    if(res < 0) {
        GSE_ERR("N2DM_Init step 4!\n");
        return res;
    }
    res = N2DM_SetPowerMode(client, enable_status);//false);
    if(res < 0) {
        GSE_ERR("N2DM_Init step 5!\n");
        return res;
    }
    if(0 != reset_cali) {
        //reset calibration only in power on
        res = N2DM_ResetCalibration(client);
        if(res < 0)
            return res;
    }
#ifdef CONFIG_N2DM_LOWPASS
    memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif
    return N2DM_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
    u8 databuf[10];
    memset(databuf, 0, sizeof(u8) * 10);
    if((NULL == buf) || (bufsize <= 30))
        return -1;
    if(NULL == client) {
        *buf = 0;
        return -2;
    }
    sprintf(buf, "N2DM Chip");
    return 0;
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadSensorData(struct i2c_client *client, char *buf,
                               int bufsize)
{
    struct n2dm_i2c_data *obj = (struct n2dm_i2c_data *)i2c_get_clientdata(client);
    u8 databuf[20];
    int acc[N2DM_AXES_NUM];
    int res = 0;
    memset(databuf, 0, sizeof(u8) * 10);
    if(NULL == buf)
        return -1;
    if(NULL == client) {
        *buf = 0;
        return -2;
    }
    if(sensor_suspend == 1) {
        //GSE_LOG("sensor in suspend read not data!\n");
        return 0;
    }
#if 0
    if(sensor_power == FALSE) {
        res = N2DM_SetPowerMode(client, true);
        if(res)
            GSE_ERR("Power on n2dm error %d!\n", res);
        msleep(20);
    }
#endif
    if((res = N2DM_ReadData(client, obj->data))) {
        GSE_ERR("I2C error: ret value=%d", res);
        return -3;
    } else {
        obj->data[N2DM_AXIS_X] += obj->cali_sw[N2DM_AXIS_X];
        obj->data[N2DM_AXIS_Y] += obj->cali_sw[N2DM_AXIS_Y];
        obj->data[N2DM_AXIS_Z] += obj->cali_sw[N2DM_AXIS_Z];
        /*remap coordinate*/
        acc[obj->cvt.map[N2DM_AXIS_X]] = obj->cvt.sign[N2DM_AXIS_X] *
                                         obj->data[N2DM_AXIS_X];
        acc[obj->cvt.map[N2DM_AXIS_Y]] = obj->cvt.sign[N2DM_AXIS_Y] *
                                         obj->data[N2DM_AXIS_Y];
        acc[obj->cvt.map[N2DM_AXIS_Z]] = obj->cvt.sign[N2DM_AXIS_Z] *
                                         obj->data[N2DM_AXIS_Z];
        //GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[N2DM_AXIS_X], acc[N2DM_AXIS_Y], acc[N2DM_AXIS_Z]);
        //Out put the mg
        acc[N2DM_AXIS_X] = acc[N2DM_AXIS_X] * GRAVITY_EARTH_1000 /
                           obj->reso->sensitivity;
        acc[N2DM_AXIS_Y] = acc[N2DM_AXIS_Y] * GRAVITY_EARTH_1000 /
                           obj->reso->sensitivity;
        acc[N2DM_AXIS_Z] = acc[N2DM_AXIS_Z] * GRAVITY_EARTH_1000 /
                           obj->reso->sensitivity;
        sprintf(buf, "%04x %04x %04x", acc[N2DM_AXIS_X], acc[N2DM_AXIS_Y],
                acc[N2DM_AXIS_Z]);
        if(atomic_read(&obj->trace) &
           ADX_TRC_IOCTL) { //atomic_read(&obj->trace) & ADX_TRC_IOCTL
            GSE_LOG("gsensor data: %s!\n", buf);
            dumpReg(client);
        }
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
static int N2DM_ReadRawData(struct i2c_client *client, char *buf)
{
    struct n2dm_i2c_data *obj = (struct n2dm_i2c_data *)i2c_get_clientdata(client);
    int res = 0;
    if (!buf || !client)
        return EINVAL;
    if((res = N2DM_ReadData(client, obj->data))) {
        GSE_ERR("I2C error: ret value=%d", res);
        return EIO;
    } else {
        sprintf(buf, "%04x %04x %04x", obj->data[N2DM_AXIS_X],
                obj->data[N2DM_AXIS_Y], obj->data[N2DM_AXIS_Z]);
    }
    return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client;
    char strbuf[N2DM_BUFSIZE];
    if(NULL == client) {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    N2DM_ReadChipInfo(client, strbuf, N2DM_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client;
    char strbuf[N2DM_BUFSIZE];
    if(NULL == client) {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    N2DM_ReadSensorData(client, strbuf, N2DM_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client;
    struct n2dm_i2c_data *obj;
    int err, len, mul;
    int tmp[N2DM_AXES_NUM];
    len = 0;
    if(NULL == client) {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    obj = i2c_get_clientdata(client);
    if((err = N2DM_ReadCalibration(client, tmp)))
        return -EINVAL;
    else {
        mul = obj->reso->sensitivity / n2dm_offset_resolution.sensitivity;
        len += snprintf(buf + len, PAGE_SIZE - len,
                        "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,
                        obj->offset[N2DM_AXIS_X], obj->offset[N2DM_AXIS_Y], obj->offset[N2DM_AXIS_Z],
                        obj->offset[N2DM_AXIS_X], obj->offset[N2DM_AXIS_Y], obj->offset[N2DM_AXIS_Z]);
        len += snprintf(buf + len, PAGE_SIZE - len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1,
                        obj->cali_sw[N2DM_AXIS_X], obj->cali_sw[N2DM_AXIS_Y],
                        obj->cali_sw[N2DM_AXIS_Z]);
        len += snprintf(buf + len, PAGE_SIZE - len,
                        "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n",
                        obj->offset[N2DM_AXIS_X] * mul + obj->cali_sw[N2DM_AXIS_X],
                        obj->offset[N2DM_AXIS_Y] * mul + obj->cali_sw[N2DM_AXIS_Y],
                        obj->offset[N2DM_AXIS_Z] * mul + obj->cali_sw[N2DM_AXIS_Z],
                        tmp[N2DM_AXIS_X], tmp[N2DM_AXIS_Y], tmp[N2DM_AXIS_Z]);
        return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf,
                                size_t count)
{
    struct i2c_client *client = n2dm_i2c_client;
    int err, x, y, z;
    int dat[N2DM_AXES_NUM];
    if(!strncmp(buf, "rst", 3)) {
        if((err = N2DM_ResetCalibration(client)))
            GSE_ERR("reset offset err = %d\n", err);
    } else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z)) {
        dat[N2DM_AXIS_X] = x;
        dat[N2DM_AXIS_Y] = y;
        dat[N2DM_AXIS_Z] = z;
        if((err = N2DM_WriteCalibration(client, dat)))
            GSE_ERR("write calibration err = %d\n", err);
    } else
        GSE_ERR("invalid format\n");
    return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = n2dm_i2c_client;
    struct n2dm_i2c_data *obj;
    u8 data;
    if(NULL == client) {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }
    obj = i2c_get_clientdata(client);
    n2dm_i2c_read_block(client, N2DM_REG_CTL_REG1, &data, 0x01);
    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_N2DM_LOWPASS
    struct i2c_client *client = n2dm_i2c_client;
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    if(atomic_read(&obj->firlen)) {
        int idx, len = atomic_read(&obj->firlen);
        GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);
        for(idx = 0; idx < len; idx++)
            GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][N2DM_AXIS_X],
                    obj->fir.raw[idx][N2DM_AXIS_Y], obj->fir.raw[idx][N2DM_AXIS_Z]);
        GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[N2DM_AXIS_X],
                obj->fir.sum[N2DM_AXIS_Y], obj->fir.sum[N2DM_AXIS_Z]);
        GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[N2DM_AXIS_X] / len,
                obj->fir.sum[N2DM_AXIS_Y] / len, obj->fir.sum[N2DM_AXIS_Z] / len);
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
    return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf,
                                  size_t count)
{
#ifdef CONFIG_N2DM_LOWPASS
    struct i2c_client *client = n2dm_i2c_client;
    struct n2dm_i2c_data *obj = i2c_get_clientdata(client);
    int firlen;
    if(1 != sscanf(buf, "%d", &firlen))
        GSE_ERR("invallid format\n");
    else if(firlen > C_MAX_FIR_LENGTH)
        GSE_ERR("exceeds maximum filter length\n");
    else {
        atomic_set(&obj->firlen, firlen);
        if(0 == firlen)
            atomic_set(&obj->fir_en, 0);
        else {
            memset(&obj->fir, 0x00, sizeof(obj->fir));
            atomic_set(&obj->fir_en, 1);
        }
    }
#endif
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
    ssize_t res;
    struct n2dm_i2c_data *obj = obj_i2c_data;
    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf,
                                 size_t count)
{
    struct n2dm_i2c_data *obj = obj_i2c_data;
    int trace;
    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if(1 == sscanf(buf, "0x%x", &trace))
        atomic_set(&obj->trace, trace);
    else
        GSE_ERR("invalid content: '%s'\n", buf);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct n2dm_i2c_data *obj = obj_i2c_data;
    if (obj == NULL) {
        GSE_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    if(obj->hw) {
        len += snprintf(buf + len, PAGE_SIZE - len, "CUST: %d %d (%d %d)\n",
                        obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);
    } else
        len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
    return len;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,
                   NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,
                   NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,
                   store_cali_value);
static DRIVER_ATTR(power,                S_IRUGO, show_power_status,
                   NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,
                   store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,
                   store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,
                   NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *n2dm_attr_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_cali,         /*show calibration data*/
    &driver_attr_power,         /*show power reg*/
    &driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_status,
};
/*----------------------------------------------------------------------------*/
static int n2dm_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(n2dm_attr_list) / sizeof(n2dm_attr_list[0]));
    if (driver == NULL)
        return -EINVAL;
    for(idx = 0; idx < num; idx++) {
        if((err = driver_create_file(driver, n2dm_attr_list[idx]))) {
            GSE_ERR("driver_create_file (%s) = %d\n", n2dm_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int n2dm_delete_attr(struct device_driver *driver)
{
    int idx , err = 0;
    int num = (int)(sizeof(n2dm_attr_list) / sizeof(n2dm_attr_list[0]));
    if(driver == NULL)
        return -EINVAL;
    for(idx = 0; idx < num; idx++)
        driver_remove_file(driver, n2dm_attr_list[idx]);
    return err;
}

/*----------------------------------------------------------------------------*/
int n2dm_operate(void *self, uint32_t command, void *buff_in, int size_in,
                 void *buff_out, int size_out, int *actualout)
{
    int err = 0;
    int value, sample_delay;
    struct n2dm_i2c_data *priv = (struct n2dm_i2c_data *)self;
    struct hwm_sensor_data *gsensor_data;
    char buff[N2DM_BUFSIZE];
    //GSE_FUN(f);
    switch (command) {
    case SENSOR_DELAY:
        if((buff_in == NULL) || (size_in < sizeof(int))) {
            GSE_ERR("Set delay parameter error!\n");
            err = -EINVAL;
        } else {
            value = *(int *)buff_in;
            if(value <= 5)
                sample_delay = N2DM_BW_200HZ;
            else if(value <= 10)
                sample_delay = N2DM_BW_100HZ;
            else
                sample_delay = N2DM_BW_50HZ;
            mutex_lock(&n2dm_op_mutex);
            err = N2DM_SetBWRate(priv->client, sample_delay);
            if(err != N2DM_SUCCESS ) //0x2C->BW=100Hz
                GSE_ERR("Set delay parameter error!\n");
            mutex_unlock(&n2dm_op_mutex);
            if(value >= 50)
                atomic_set(&priv->filter, 0);
            else {
                priv->fir.num = 0;
                priv->fir.idx = 0;
                priv->fir.sum[N2DM_AXIS_X] = 0;
                priv->fir.sum[N2DM_AXIS_Y] = 0;
                priv->fir.sum[N2DM_AXIS_Z] = 0;
                atomic_set(&priv->filter, 1);
            }
        }
        break;
    case SENSOR_ENABLE:
        if((buff_in == NULL) || (size_in < sizeof(int))) {
            GSE_ERR("Enable sensor parameter error!\n");
            err = -EINVAL;
        } else {
            value = *(int *)buff_in;
            mutex_lock(&n2dm_op_mutex);
            GSE_LOG("Gsensor device enable function enable = %d, sensor_power = %d!\n",
                    value, sensor_power);
            if(((value == 0) && (sensor_power == false)) || ((value == 1) &&
                    (sensor_power == true))) {
                enable_status = sensor_power;
                GSE_LOG("Gsensor device have updated!\n");
            } else {
                enable_status = !sensor_power;
                err = N2DM_SetPowerMode( priv->client, !sensor_power);
                GSE_LOG("Gsensor not in suspend N2DM_SetPowerMode!, enable_status = %d\n",
                        enable_status);
            }
            mutex_unlock(&n2dm_op_mutex);
        }
        break;
    case SENSOR_GET_DATA:
        if((buff_out == NULL) || (size_out < sizeof(struct hwm_sensor_data))) {
            GSE_ERR("get sensor data parameter error!\n");
            err = -EINVAL;
        } else {
            mutex_lock(&n2dm_op_mutex);
            gsensor_data = (struct hwm_sensor_data *)buff_out;
            N2DM_ReadSensorData(priv->client, buff, N2DM_BUFSIZE);
            sscanf(buff, "%x %x %x", &gsensor_data->values[0],
                   &gsensor_data->values[1], &gsensor_data->values[2]);
            gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            gsensor_data->value_divide = 1000;
            mutex_unlock(&n2dm_op_mutex);
        }
        break;
    default:
        GSE_ERR("gsensor operate function no this parameter %d!\n", command);
        err = -1;
        break;
    }
    return err;
}

/******************************************************************************
 * Function Configuration
******************************************************************************/
static int n2dm_open(struct inode *inode, struct file *file)
{
    file->private_data = n2dm_i2c_client;
    if(file->private_data == NULL) {
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int n2dm_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
static long n2dm_unlocked_ioctl(struct file *file, unsigned int cmd,
                                unsigned long arg)

{
    struct i2c_client *client = (struct i2c_client *)file->private_data;
    struct n2dm_i2c_data *obj = (struct n2dm_i2c_data *)i2c_get_clientdata(client);
    char strbuf[N2DM_BUFSIZE];
    void __user *data;
    struct SENSOR_DATA sensor_data;
    long err = 0;
    int cali[3];
    //GSE_FUN(f);
    if(_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    else if(_IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    if(err) {
        GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }
    switch(cmd) {
    case GSENSOR_IOCTL_INIT:
        N2DM_Init(client, 0);
        break;
    case GSENSOR_IOCTL_READ_CHIPINFO:
        data = (void __user *) arg;
        if(data == NULL) {
            err = -EINVAL;
            break;
        }
        N2DM_ReadChipInfo(client, strbuf, N2DM_BUFSIZE);
        if(copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            err = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_SENSORDATA:
        data = (void __user *) arg;
        if(data == NULL) {
            err = -EINVAL;
            break;
        }
        N2DM_SetPowerMode(client, true);
        N2DM_ReadSensorData(client, strbuf, N2DM_BUFSIZE);
        if(copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
            err = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_GAIN:
        data = (void __user *) arg;
        if(data == NULL) {
            err = -EINVAL;
            break;
        }
        if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D))) {
            err = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_OFFSET:
        data = (void __user *) arg;
        if(data == NULL) {
            err = -EINVAL;
            break;
        }
        if(copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D))) {
            err = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_READ_RAW_DATA:
        data = (void __user *) arg;
        if(data == NULL) {
            err = -EINVAL;
            break;
        }
        N2DM_ReadRawData(client, strbuf);
        if(copy_to_user(data, &strbuf, strlen(strbuf) + 1)) {
            err = -EFAULT;
            break;
        }
        break;
    case GSENSOR_IOCTL_SET_CALI:
        data = (void __user *)arg;
        if(data == NULL) {
            err = -EINVAL;
            break;
        }
        if(copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
            err = -EFAULT;
            break;
        }
        if(atomic_read(&obj->suspend)) {
            GSE_ERR("Perform calibration in suspend state!!\n");
            err = -EINVAL;
        } else {
            cali[N2DM_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
            cali[N2DM_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
            cali[N2DM_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;
            err = N2DM_WriteCalibration(client, cali);
        }
        break;
    case GSENSOR_IOCTL_CLR_CALI:
        err = N2DM_ResetCalibration(client);
        break;
    case GSENSOR_IOCTL_GET_CALI:
        data = (void __user *)arg;
        if(data == NULL) {
            err = -EINVAL;
            break;
        }
        if((err = N2DM_ReadCalibration(client, cali)))
            break;
        sensor_data.x = cali[N2DM_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        sensor_data.y = cali[N2DM_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        sensor_data.z = cali[N2DM_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
        if(copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
            err = -EFAULT;
            break;
        }
        break;
    default:
        GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
        err = -ENOIOCTLCMD;
        break;
    }
    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations n2dm_fops = {
    .owner = THIS_MODULE,
    .open = n2dm_open,
    .release = n2dm_release,
    .unlocked_ioctl = n2dm_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice n2dm_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gsensor",
    .fops = &n2dm_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef USE_EARLY_SUSPEND
/*----------------------------------------------------------------------------*/
static int n2dm_suspend(struct device *dev)
{
    struct n2dm_i2c_data *obj = obj_i2c_data;
    int err = 0;
    //u8 dat;
    GSE_FUN();
    printk("gongyan");
#ifdef SIGNIFICANT
    if(significant_enable_status == true)
        return 0;
#endif
    mutex_lock(&n2dm_op_mutex);
    if(obj == NULL) {
        mutex_unlock(&n2dm_op_mutex);
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }
    //read old data
    atomic_set(&obj->suspend, 1);
    pre_suspend = sensor_power;
    if(0 != (err = N2DM_SetPowerMode(obj->client, false))) {
        GSE_ERR("write power control fail!!\n");
        mutex_unlock(&n2dm_op_mutex);
        return err;
    }
    N2DM_power(obj->hw, 0);
    sensor_suspend = 1;
    mutex_unlock(&n2dm_op_mutex);
    GSE_ERR("lanying.he suspend out\n");
    return err;
}
/*----------------------------------------------------------------------------*/
static int n2dm_resume(struct device *dev)
{
    struct n2dm_i2c_data *obj = obj_i2c_data;
    int err;
    GSE_FUN();
    //Begin modified by yan.gong for XR6097200 on 2018/3/9
#ifdef SIGNIFICANT
    if(significant_enable_status == true)
        return 0;
#endif
    mutex_lock(&n2dm_op_mutex);
    //End modified by yan.gong for XR6097200 on 2018/3/9
    if(obj == NULL) {
        mutex_unlock(&n2dm_op_mutex);
        GSE_ERR("null pointer!!\n");
        return -EINVAL;
    }
    err = N2DM_SetPowerMode(obj->client, pre_suspend);
    if (err) {
        mutex_unlock(&n2dm_op_mutex);
        ACC_PR_ERR("initialize client fail!!\n");
        return err;
    }
    atomic_set(&obj->suspend, 0);
    sensor_suspend = 0;
    mutex_unlock(&n2dm_op_mutex);
    GSE_ERR("lanying.he resume out\n");
    return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/

static void n2dm_early_suspend(struct early_suspend *h)
{
    struct n2dm_i2c_data *obj = container_of(h, struct n2dm_i2c_data, early_drv);
    u8 databuf[2];
    int err = 0;
    u8 addr = N2DM_REG_CTL_REG1;
    if(obj == NULL) {
        GSE_ERR("null pointer!!\n");
        return;
    }
    atomic_set(&obj->suspend, 1);
    mutex_lock(&n2dm_op_mutex);
    GSE_FUN();
    if((err = N2DM_SetPowerMode(obj->client, false))) {
        GSE_ERR("write power control fail!!\n");
        mutex_unlock(&n2dm_op_mutex);
        return;
    }
    sensor_suspend = 1;
    mutex_unlock(&n2dm_op_mutex);
    N2DM_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void n2dm_late_resume(struct early_suspend *h)
{
    struct n2dm_i2c_data *obj = container_of(h, struct n2dm_i2c_data, early_drv);
    int err;
    if(obj == NULL) {
        GSE_ERR("null pointer!!\n");
        return;
    }
    N2DM_power(obj->hw, 1);
    mutex_lock(&n2dm_op_mutex);
    GSE_FUN();
    if((err = N2DM_Init(obj->client, 0))) {
        GSE_ERR("initialize client fail!!\n");
        mutex_unlock(&n2dm_op_mutex);
        return;
    }
    sensor_suspend = 0;
    mutex_unlock(&n2dm_op_mutex);
    atomic_set(&obj->suspend, 0);
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
#if 0
static int n2dm_i2c_detect(struct i2c_client *client,
                           struct i2c_board_info *info)
{
    strcpy(info->type, N2DM_DEV_NAME);
    return 0;
}
#endif
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int n2dm_open_report_data(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int n2dm_enable_nodata(int en)
{
    int res = 0;
    bool power = false;
    if(1 == en)
        power = true;
    if(0 == en)
        power = false;
	GSE_ERR("lanying.he nodata comein en=%d\n", en);
#ifdef SIGNIFICANT
    acc_enable_status = power;
    if(significant_enable_status) {
        GSE_ERR("lanying.he significant motion sensor is open gsensor cant not  operate\n");
        return 0;
    }
#endif
    res = N2DM_SetPowerMode(obj_i2c_data->client, power);
    if(res != N2DM_SUCCESS) {
        GSE_ERR("N2DM_SetPowerMode fail!\n");
        return -1;
    }
    GSE_LOG("n2dm_enable_nodata OK!\n");
    return 0;
}

static int n2dm_set_delay(u64 ns)
{
    int value = 0;
    int sample_delay = 0;
    int err;
    value = (int)ns / 1000 / 1000;
    if(value <= 5)
        sample_delay = N2DM_BW_200HZ;
    else if(value <= 10)
        sample_delay = N2DM_BW_100HZ;
    else
        sample_delay = N2DM_BW_50HZ;
    mutex_lock(&n2dm_op_mutex);
    err = N2DM_SetBWRate(obj_i2c_data->client, sample_delay);
    if(err != N2DM_SUCCESS ) //0x2C->BW=100Hz
        GSE_ERR("Set delay parameter error!\n");
    mutex_unlock(&n2dm_op_mutex);
    if(value >= 50)
        atomic_set(&obj_i2c_data->filter, 0);
    else {
        obj_i2c_data->fir.num = 0;
        obj_i2c_data->fir.idx = 0;
        obj_i2c_data->fir.sum[N2DM_AXIS_X] = 0;
        obj_i2c_data->fir.sum[N2DM_AXIS_Y] = 0;
        obj_i2c_data->fir.sum[N2DM_AXIS_Z] = 0;
        atomic_set(&obj_i2c_data->filter, 1);
    }
    GSE_LOG("n2dm_set_delay (%d)\n", value);
    return 0;
}

static int n2dm_get_data(int *x , int *y, int *z, int *status)
{
    char buff[N2DM_BUFSIZE];
    N2DM_ReadSensorData(obj_i2c_data->client, buff, N2DM_BUFSIZE);
    sscanf(buff, "%x %x %x", x, y, z);
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

static int n2dm_batch(int flag, int64_t samplingPeriodNs,
                      int64_t maxBatchReportLatencyNs)
{
    int value = 0;
    ACC_LOG("gsensor_batch (%d), chip only use 1024HZ\n", value);
    return 0;
}

static int n2dm_flush(void)
{
    return acc_flush_report();
}

/*----------------------------------------------------------------------------*/
static int n2dm_i2c_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
    struct i2c_client *new_client;
    struct n2dm_i2c_data *obj;
    //struct acc_drv_obj sobj;
    int err = 0;
    int retry = 0;
    struct acc_control_path ctl = {0};
    struct acc_data_path data = {0};
    GSE_FUN();
    err = get_accel_dts_func(client->dev.of_node, hw);
    if (err < 0) {
        GSE_ERR("get dts info fail\n");
        err = -EFAULT;
        goto exit;
    }

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }

    obj->hw = hw;

    if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt))) {
        GSE_ERR("invalid direction: %d\n", obj->hw->direction);
        goto exit;
    }
    obj_i2c_data = obj;
    obj->client = client;
    new_client = obj->client;
    i2c_set_clientdata(new_client, obj);
    atomic_set(&obj->trace, 0);
    atomic_set(&obj->suspend, 0);
#ifdef CONFIG_N2DM_LOWPASS
    if(obj->hw->firlen > C_MAX_FIR_LENGTH)
        atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
    else
        atomic_set(&obj->firlen, obj->hw->firlen);
    if(atomic_read(&obj->firlen) > 0)
        atomic_set(&obj->fir_en, 1);
#endif
    n2dm_i2c_client = new_client;
    for(retry = 0; retry < 3; retry++) {
        if((err = N2DM_Init(new_client, 1))) {
            GSE_ERR("n2dm_device init cilent fail time: %d\n", retry);
            continue;
        } else //added by lanying.he
            break;
    }
    if(err != 0)
        goto exit_init_failed;
#ifdef SIGNIFICANT
    INIT_WORK(&obj->eint_work,  eintWork);  //added by lanying.he for significant
#endif
    if((err = misc_register(&n2dm_device))) {
        GSE_ERR("n2dm_device register failed\n");
        goto exit_misc_device_register_failed;
    }
    if((err = n2dm_create_attr(&(n2dm_init_info.platform_diver_addr->driver)))) {
        GSE_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
    ctl.open_report_data = n2dm_open_report_data;
    ctl.enable_nodata = n2dm_enable_nodata;
    ctl.set_delay  = n2dm_set_delay;
    ctl.is_report_input_direct = false;
    ctl.batch = n2dm_batch;
    ctl.flush = n2dm_flush;
    ctl.is_support_batch = false;
    err = acc_register_control_path(&ctl);
    if(err) {
        GSE_ERR("register acc control path err\n");
        goto exit_kfree;
    }
    data.get_data = n2dm_get_data;
    data.vender_div = 1000;
    err = acc_register_data_path(&data);
    if(err) {
        GSE_ERR("register acc data path err\n");
        goto exit_kfree;
    }
#ifdef USE_EARLY_SUSPEND
    obj->early_drv.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2;
    obj->early_drv.suspend  = n2dm_early_suspend;
    obj->early_drv.resume   = n2dm_late_resume;
    register_early_suspend(&obj->early_drv);
#endif
    GSE_ERR("%s: OK\n", __func__);
/*Add-Begin by zhikui.li@tcl.com,Remove unused variables*/
    //n2dm_init_flag = 0;
    n2dm_acc_init_flag = 0;
/*Add-end by zhikui.li@tcl.com,Remove unused variables*/
    #ifdef CONFIG_TCT_ROBUST
    robust_accel_i2c_check_status = 1;
    #endif
    return 0;
exit_create_attr_failed:
    misc_deregister(&n2dm_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(new_client);
exit_kfree:
    kfree(obj);
exit:
    GSE_ERR("%s: err = %d\n", __func__, err);
/*Add-Begin by zhikui.li@tcl.com,Remove unused variables*/
    //n2dm_init_flag = -1;
/*Add-end by zhikui.li@tcl.com,Remove unused variables*/
    return err;
}

/*----------------------------------------------------------------------------*/
static int n2dm_i2c_remove(struct i2c_client *client)
{
    int err = 0;
    if((err = n2dm_delete_attr(&(n2dm_init_info.platform_diver_addr->driver))))
        GSE_ERR("n2dm_delete_attr fail: %d\n", err);
    misc_deregister(&n2dm_device);
    //if((err = hwmsen_detach(ID_ACCELEROMETER)))
    n2dm_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int  n2dm_remove(void)
{
    // struct acc_hw *hw = n2dm_get_cust_acc_hw();
    GSE_FUN();
    N2DM_power(hw, 0);
    i2c_del_driver(&n2dm_i2c_driver);
    return 0;
}
/*-----------------------significant beginning-----------------------------------------------------*/
//added by lanying.he for significant beginning
#ifdef SIGNIFICANT
static irqreturn_t eint_func(int irq , void *desc) //liupeng begin
{
    struct n2dm_i2c_data *priv = obj_i2c_data;
    GSE_FUN();
    //disable_irq_nosync(obj_i2c_data->irq); //liupeng
    disable_irq_nosync(priv->irq);
    schedule_work(&priv->eint_work);
    return IRQ_HANDLED;
}

static void eintWork(struct work_struct *work)
{
    u8 databuf[2] = {0};
    struct n2dm_i2c_data *obj = obj_i2c_data;
    printk("eintWork comein!!\n");
    if(obj == NULL) {
        GSE_ERR("obj_i2c_data is null pointer!!\n");
        goto eint_work_exit;
    }
    step_notify(TYPE_SIGNIFICANT);
    memset(databuf, 0, sizeof(u8) * 2);
    if(hwmsen_read_byte(obj->client, 0x35, databuf)) {
        GSE_ERR("%s read clear_eint_register err!\n", __func__);
        goto eint_work_exit;
    }
eint_work_exit:
    enable_irq(obj->irq);
}

int n2dm_setup_eint(void)
{
    int ret;
    u32 ints[2] = { 0, 0 };
    struct n2dm_i2c_data *obj = obj_i2c_data;
    //  obj_i2c_data->irq_node = of_find_compatible_node(NULL,NULL,"mediatek, GSE_1-eint");
    obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, GSE_1-eint");
    if (obj->irq_node) {
        of_property_read_u32_array(obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
        //of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
        gpio_set_debounce(ints[0], ints[1]);
        obj->irq = irq_of_parse_and_map(obj->irq_node, 0);
        if (!obj->irq) {
            printk("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        ret = request_irq(obj->irq, eint_func,  IRQF_TRIGGER_FALLING,
                          "mediatek, GSE_1-eint", NULL);
        if (ret != 0)
            printk("[significant]EINT IRQ LINE NOT AVAILABLE\n");
        else
            printk("[significant] %s set EINT finished, obj_i2c_data->irq=%d,\n", __func__,
                   obj->irq);
    } else
        printk("[significant] %s can't find compatible node\n", __func__);
    enable_irq_wake(obj->irq);//lzk
    return 0;
}
#if 0
static irqreturn_t eint_func(int irq , void *desc) //liupeng begin
{
    struct n2dm_i2c_data *priv = obj_i2c_data;
    //  GSE_LOG("GSE into  interrupt liupeng !\n");
    schedule_work(&priv->eint_work);
    disable_irq_nosync(obj_i2c_data->irq); //liupeng
    return IRQ_HANDLED;
}

static void eintWork(struct work_struct *work)
{
    u8 databuf[2] = {0};
    struct n2dm_i2c_data *obj = obj_i2c_data;
    if(obj == NULL) {
        GSE_ERR("mc3xxx_obj_i2c_data is null pointer!!\n");
        goto eint_work_exit;
    }
    step_notify(TYPE_SIGNIFICANT);
    msleep(10);
    memset(databuf, 0, sizeof(u8) * 2);
    if(hwmsen_read_byte(obj->client, 0x35, databuf)) {
        GSE_ERR("%s read clear_eint_register err!\n", __func__);
        goto eint_work_exit;
    }
eint_work_exit:
    enable_irq(obj_i2c_data->irq);  //liupeng
}

int n2dm_setup_eint(void)
{
    int ret;
    u32 ints[2] = { 0, 0 };
    obj_i2c_data->irq_node = of_find_compatible_node(NULL, NULL,
                             "mediatek, GSE_1-eint");
    if (obj_i2c_data->irq_node) {
        of_property_read_u32_array(obj_i2c_data->irq_node, "debounce", ints,
                                   ARRAY_SIZE(ints));
        //of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
        gpio_set_debounce(ints[0], ints[1]);
        obj_i2c_data->irq = irq_of_parse_and_map(obj_i2c_data->irq_node, 0);
        if (!obj_i2c_data->irq) {
            GSE_ERR("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }
        ret = request_irq(obj_i2c_data->irq, eint_func,  IRQF_TRIGGER_FALLING,
                          "mediatek, GSE_1-eint", NULL);
        if (ret != 0)
            printk("%s  EINT IRQ LINE NOT AVAILABLE\n", __func__);
        else
            printk("%s set EINT finished, obj_i2c_data->irq=%d\n", __func__,
                   obj_i2c_data->irq);
    } else
        printk("%s can't find compatible node\n", __func__);
    return 0;
}
#endif
static int ConfigureThd(struct i2c_client *client)
{
    u8 databuf[2];
    int res = 0;
    memset(databuf, 0, sizeof(u8) * 2);
    databuf[0] = 0x24;
    databuf[1] = 0x02;
    res = i2c_master_send(client, databuf, 0x2);
    if(res < 0)
        return -1;
    memset(databuf, 0, sizeof(u8) * 2);
    databuf[0] = 0x25;
    databuf[1] = 0x6a;
    res = i2c_master_send(client, databuf, 0x2);
    if(res < 0)
        return -1;
    memset(databuf, 0, sizeof(u8) * 2);
    databuf[0] = 0x36;
    //Begin modified by yan.gong for 6176060 on 2018-04-09
    databuf[1] = 0x73;
    //End modified by yan.gong for 6176060 on 2018-04-09
    res = i2c_master_send(client, databuf, 0x2);
    if(res < 0)
        return -1;
    memset(databuf, 0, sizeof(u8) * 2);
    databuf[0] = 0x37;
    databuf[1] = 0x02;
    res = i2c_master_send(client, databuf, 0x2);
    if(res < 0)
        return -1;
    GSE_LOG("[%s]  success!\n", __func__);
    return 0;
}

static int setMotionEnable(struct i2c_client *client, u8 enable)
{
    int res = 0;
    u8 databuf[2];
    u8 addr = 0x34;
    memset(databuf, 0, sizeof(u8) * 2);
    databuf[0] = addr;
    if(enable)
        databuf[1] |= 0x2a;
    else
        databuf[1] = 0;
    res = i2c_master_send(client, databuf, 0x2);
    if(res < 0)
        return -1;
    GSE_ERR("Motiondetect  %d!\n", enable);
    msleep(5);
    return 0;
}

static int setIntEnable(struct i2c_client *client, u8 enable)
{
    u8 databuf[2];
    u8 addr = 0x22;
    int res = 0;
    memset(databuf, 0, sizeof(u8) * 2);
    databuf[0] = addr;
    if(enable)
        databuf[1] = 0x20;
    else
        databuf[1] = 0;
    res = i2c_master_send(client, databuf, 0x2);
    if(res < 0)
        return -1;
    if(enable) {
        res = ConfigureThd(client);
        if(res < 0)
            return -1;
    }
    return 0;
}

int n2dm_set_acc_interrupter(bool flag)
{
    int res = 0;
    struct n2dm_i2c_data *obj = obj_i2c_data;
    printk("n2dm_set_acc_interrupter en =%d\n", flag);
    if(flag) {
        significant_enable_status = true;
        N2DM_SetPowerMode(obj->client, false);
        res = setMotionEnable(obj->client, true); //set 0x1b enable wake up function
        if(res != 0)
            GSE_ERR("SetMotionEnable faild\n");
        res = setIntEnable(obj->client,
                           true);  //0x20  physical interrupt pin (7) is enabled
        if(res != 0)
            GSE_ERR("SetIntEnable faild\n");
        N2DM_SetPowerMode(obj->client, true);
        msleep(500);
    } else {
        significant_enable_status = false;
        N2DM_SetPowerMode(obj->client, false);  //set register power must to  false
        res = setIntEnable(obj->client, false);
        res = setMotionEnable(obj->client, false);
        if(res != 0)
            GSE_ERR("setMotionEnable faild\n");
        N2DM_SetPowerMode(obj->client, true);
        if(!acc_enable_status)
            N2DM_SetPowerMode(obj->client, false);
    }
    return res;
}

#endif
/*-----------------------significant end -----------------------------------------------------*/
static int  n2dm_local_init(void)
{
    //struct acc_hw *hw = n2dm_get_cust_acc_hw();
    //GSE_FUN();
    N2DM_power(hw, 1);
    if(i2c_add_driver(&n2dm_i2c_driver)) {
        GSE_ERR("add driver error-1\n");
        return -1;
    }
/*Add-Begin by zhikui.li@tcl.com,Remove unused variables*/
    if(-1 == n2dm_acc_init_flag) {//n2dm_init_flag
/*Add-end by zhikui.li@tcl.com,Remove unused variables*/
        GSE_ERR("add driver error-2\n");
        return -1;
    }
    GSE_ERR("add driver ok\n");
    return 0;
}

/*----------------------------------------------------------------------------*/
static int __init n2dm_init(void)
{
    GSE_FUN();
    acc_driver_add(&n2dm_init_info);
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit n2dm_exit(void)
{
    GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(n2dm_init);
module_exit(n2dm_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("N2DM I2C driver");
MODULE_AUTHOR("jay.huangfu@st.com");
