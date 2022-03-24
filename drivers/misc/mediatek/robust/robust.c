#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

static struct class *robust_i2c_check_class;
static struct device *tp_check_device;
static struct device *accel_check_device;
static struct device *gyro_check_device;
static struct device *mag_check_device;
static struct device *pro_check_device;
static struct device *ls_check_device;
static struct device *ind_led_check_device;

int robust_tp_i2c_check_status = 0;         //tp
EXPORT_SYMBOL(robust_tp_i2c_check_status);

int robust_accel_i2c_check_status = 0;      //accel
EXPORT_SYMBOL(robust_accel_i2c_check_status);

int robust_gyro_i2c_check_status = 0;       //gyro
EXPORT_SYMBOL(robust_gyro_i2c_check_status);

int robust_mag_i2c_check_status = 0;        //mag
EXPORT_SYMBOL(robust_mag_i2c_check_status);

int robust_pro_i2c_check_status = 0;        //pro
EXPORT_SYMBOL(robust_pro_i2c_check_status);

int robust_ls_i2c_check_status = 0;         //light sensor
EXPORT_SYMBOL(robust_ls_i2c_check_status);

int robust_ind_led_i2c_check_status = 1;    //indicator_led
EXPORT_SYMBOL(robust_ind_led_i2c_check_status);

void register_camera_robust_dev(unsigned int sensor_dev);
EXPORT_SYMBOL(register_camera_robust_dev);

static ssize_t dev_attribute_tp_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", robust_tp_i2c_check_status);
}

static ssize_t dev_attribute_accel_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", robust_accel_i2c_check_status);
}

static ssize_t dev_attribute_gyro_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", robust_gyro_i2c_check_status);
}

static ssize_t dev_attribute_mag_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", robust_mag_i2c_check_status);
}

static ssize_t dev_attribute_pro_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", robust_pro_i2c_check_status);
}

static ssize_t dev_attribute_ls_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", robust_ls_i2c_check_status);
}

static ssize_t dev_attribute_ind_led_status_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", robust_ind_led_i2c_check_status);
}

static struct device_attribute dev_tp_status =
    __ATTR(status, S_IRUGO, dev_attribute_tp_status_show, NULL);

static struct device_attribute dev_accel_status =
    __ATTR(status, S_IRUGO, dev_attribute_accel_status_show, NULL);

static struct device_attribute dev_gyro_status =
    __ATTR(status, S_IRUGO, dev_attribute_gyro_status_show, NULL);

static struct device_attribute dev_mag_status =
    __ATTR(status, S_IRUGO, dev_attribute_mag_status_show, NULL);

static struct device_attribute dev_pro_status =
    __ATTR(status, S_IRUGO, dev_attribute_pro_status_show, NULL);

static struct device_attribute dev_ls_status =
    __ATTR(status, S_IRUGO, dev_attribute_ls_status_show, NULL);

static struct device_attribute dev_ind_led_status =
    __ATTR(status, S_IRUGO, dev_attribute_ind_led_status_show, NULL);


static int camera_robust_check_open(struct inode *pnode, struct file *pfile)
{
    return 0;
}

static int camera_robust_check_release(struct inode *pnode, struct file *pfile)
{
    return 0;
}

static const struct file_operations camera_robust_check_fops = {
    .owner = THIS_MODULE,
    .open = camera_robust_check_open,
    .release = camera_robust_check_release,
};

static struct miscdevice main_camera_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "media2",
    .fops = &camera_robust_check_fops,
};

static struct miscdevice sub_camera_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "media1",
    .fops = &camera_robust_check_fops,
};

static struct miscdevice main2_camera_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "media3",
    .fops = &camera_robust_check_fops,
};

void register_camera_robust_dev(unsigned int sensor_dev)
{
    int ret = -1;
    static bool main_camera_dev_registered = false;
    static bool sub_camera_dev_registered = false;
    static bool main2_camera_dev_registered = false;

    switch(sensor_dev) {
    case 1:
        if (main_camera_dev_registered) {
            pr_err("Main camera device node was already registered!\n");
            return;
        }
        ret = misc_register(&main_camera_device);
        if (ret == 0)
            main_camera_dev_registered = true;
        break;
    case 2:
        if (sub_camera_dev_registered) {
            pr_err("Sub camera device node was already registered!\n");
            return;
        }
        ret = misc_register(&sub_camera_device);
        if (ret == 0)
            sub_camera_dev_registered = true;
        break;
    case 4:
        if (main2_camera_dev_registered) {
            pr_err("Main2 camera device node was already registered!\n");
            return;
        }
        ret = misc_register(&main2_camera_device);
        if (ret == 0)
            main2_camera_dev_registered = true;
        break;
    default:
        pr_err("Unknow sensor dev\n");
    }
    if (ret != 0)
        pr_err("Failed to register device node for camera (%d)!\n", sensor_dev);
}

static int __init robust_i2c_check_init(void)
{
    int err = 0;
    printk("%s: entering!\n", __func__);

    robust_i2c_check_class = class_create(THIS_MODULE, "i2c_check");
    if (IS_ERR(robust_i2c_check_class)) {
        pr_err("%s: robust_i2c_check_class create failed!\n", __func__);
        goto class_create_fail;
    }

    tp_check_device = device_create(robust_i2c_check_class, NULL, 0, NULL,
                                    "touch_panel");
    if (IS_ERR(tp_check_device)) {
        pr_err("%s: tp_check_device create failed!\n", __func__);
        goto tp_check_device_create_fail;
    }
    err = device_create_file(tp_check_device, &dev_tp_status);
    if (err) {
        pr_err("%s: tp check status create failed!\n", __func__);
        goto tp_status_file_create_fail;
    }

    accel_check_device = device_create(robust_i2c_check_class, NULL, 0, NULL,
                                       "AccelerometerSensor");
    if (IS_ERR(accel_check_device)) {
        pr_err("%s: accel_check_device_create failed!\n", __func__);
        goto accel_check_device_create_fail;
    }
    err = device_create_file(accel_check_device, &dev_accel_status);
    if (err) {
        pr_err("%s: accel check status create failed!\n", __func__);
        goto accel_status_file_create_fail;
    }

    gyro_check_device = device_create(robust_i2c_check_class, NULL, 0, NULL,
                                      "GyroscopeSensor");
    if (IS_ERR(gyro_check_device)) {
        pr_err("%s: gyro_check_device create failed!\n", __func__);
        goto gyro_check_device_create_fail;
    }
    err = device_create_file(gyro_check_device, &dev_gyro_status);
    if(err) {
        pr_err("%s: gyro check status create failed!\n", __func__);
        goto gyro_status_file_create_fail;
    }

    mag_check_device = device_create(robust_i2c_check_class, NULL, 0, NULL,
                                     "MagnetometerSensor");
    if (IS_ERR(mag_check_device)) {
        pr_err("%s: mag_check_device create failed!\n", __func__);
        goto mag_check_device_create_fail;
    }
    err = device_create_file(mag_check_device, &dev_mag_status);
    if(err) {
        pr_err("%s: mag check status create failed!\n", __func__);
        goto mag_status_file_create_fail;
    }

    pro_check_device = device_create(robust_i2c_check_class, NULL, 0, NULL,
                                     "ProximitySensor");
    if (IS_ERR(pro_check_device)) {
        pr_err("%s: pro_check_device create failed!\n", __func__);
        goto pro_check_device_create_fail;
    }
    err = device_create_file(pro_check_device, &dev_pro_status);
    if(err) {
        pr_err("%s: pro check status create failed!\n", __func__);
        goto pro_status_file_create_fail;
    }

    ls_check_device = device_create(robust_i2c_check_class, NULL, 0, NULL,
                                    "LightSensor");
    if (IS_ERR(ls_check_device)) {
        pr_err("%s: ls_check_device create failed!\n", __func__);
        goto ls_check_device_create_fail;
    }
    err = device_create_file(ls_check_device, &dev_ls_status);
    if(err) {
        pr_err("%s: ls check status create failed!\n", __func__);
        goto ls_status_file_create_fail;
    }

    ind_led_check_device = device_create(robust_i2c_check_class, NULL, 0, NULL,
                                         "indicator_led");
    if (IS_ERR(ind_led_check_device)) {
        pr_err("%s: ind_led_check_device create failed!\n", __func__);
        goto ind_led_check_device_create_fail;
    }
    err = device_create_file(ind_led_check_device, &dev_ind_led_status);
    if(err) {
        pr_err("%s: ind_led check status create failed!\n", __func__);
        goto ind_led_status_file_create_fail;
    }

    printk("%s: creater all node success!\n", __func__);

    return 0;

ind_led_status_file_create_fail:
    device_del(ind_led_check_device);
ind_led_check_device_create_fail:
    device_remove_file(ls_check_device, &dev_ls_status);
ls_status_file_create_fail:
    device_del(ls_check_device);
ls_check_device_create_fail:
    device_remove_file(pro_check_device, &dev_pro_status);
pro_status_file_create_fail:
    device_del(pro_check_device);
pro_check_device_create_fail:
    device_remove_file(mag_check_device, &dev_mag_status);
mag_status_file_create_fail:
    device_del(mag_check_device);
mag_check_device_create_fail:
    device_remove_file(gyro_check_device, &dev_gyro_status);
gyro_status_file_create_fail:
    device_del(gyro_check_device);
gyro_check_device_create_fail:
    device_remove_file(accel_check_device, &dev_accel_status);
accel_status_file_create_fail:
    device_del(accel_check_device);
accel_check_device_create_fail:
    device_remove_file(tp_check_device, &dev_tp_status);
tp_status_file_create_fail:
    device_del(tp_check_device);
tp_check_device_create_fail:
    class_destroy(robust_i2c_check_class);
class_create_fail:
    return 0;
}

static void __exit robust_i2c_check_exit(void)
{
    printk("%s: entering!\n", __func__);
}


module_init(robust_i2c_check_init);
module_exit(robust_i2c_check_exit);


