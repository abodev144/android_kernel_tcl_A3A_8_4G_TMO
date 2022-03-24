#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/seq_file.h>
#include <lcm_drv.h>

/*[BUGFIX]-add-begin,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/
#define AUDIO_PARA_VER_MONITOR
/*[BUGFIX]-add-end,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/

#define BASE_DIR_NAME        "tct_debug"
#define TP_DIR_NAME          "tp"
#define LCD_DIR_NAME         "lcd"
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    #define HALL_DIR_NAME    "hall"
#endif
#ifdef CONFIG_MMC_FFU
    #define FFU_DIR_NAME      "ffu"
#endif
/*[BUGFIX]-add-begin,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/
#ifdef AUDIO_PARA_VER_MONITOR
#define AUDIO_DIR_NAME "audio_para"
#endif
/*[BUGFIX]-add-end,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/

struct ctp_vendor_info {
    char md5[32];
    char name[64];
    u8   id;
};

struct proc_dir_entry *base;
EXPORT_SYMBOL(base);
static struct proc_dir_entry *tp_dir, *lcd_dir;
static struct proc_dir_entry *tp_entry, *lcd_entry, *gesture_entry;
static struct proc_dir_entry *smart_cover_entry, *palm_entry;
static struct proc_dir_entry *gtp_cfg_entry, *gtp_fw_entry, *gtp_id_entry;
/*[BUGFIX]-add-begin,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/
/*add audio parameters monitor function*/
#ifdef AUDIO_PARA_VER_MONITOR
static struct proc_dir_entry *audio_dir,*audio_entry;
#endif
/*[BUGFIX]-add-end,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/

static struct class *device_info_class;
static struct device *device_info_dev;
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    static struct proc_dir_entry *hall_dir, *hall_status_entry;
#endif
#ifdef CONFIG_MMC_FFU
    static struct proc_dir_entry *ffu_dir , *ffu_entry;
#endif


static struct ctp_vendor_info all_ctp_vendor_info[] = {
    {"md5xxxxxx", "LWGB1011010_PIXI5104G_FT5526_FW_20161209", 0x00},
    {"md5xxxxxx", "DPT10112-5A6028ACY_PIXI5104G_GT9271_FW_20161216", 0x00},
#ifdef KERNEL_THIRD_TP
    {KERNEL_THIRD_TP, "CTPXXXXXXXXX", 0xFF},
#endif
};
static char all_disp_lcm_name[256];

char tp_info[512];
EXPORT_SYMBOL(tp_info);

char lcd_info[512];
EXPORT_SYMBOL(lcd_info);

int smart_cover_flag = 0;
EXPORT_SYMBOL(smart_cover_flag);
void (*smart_cover_flag_charge_notify)(int flag) = NULL;
EXPORT_SYMBOL(smart_cover_flag_charge_notify);

int gesture_wakeup_flag = 0;
EXPORT_SYMBOL(gesture_wakeup_flag);

#ifdef CONFIG_MMC_FFU
    int ffu_flag = 1;
    EXPORT_SYMBOL(ffu_flag);
#endif

bool  palm_suspend_flag = false;
EXPORT_SYMBOL(palm_suspend_flag);

unsigned int gtp_cfg_version = 0;
EXPORT_SYMBOL(gtp_cfg_version);

unsigned int gtp_fw_version = 0;
EXPORT_SYMBOL(gtp_fw_version);

u8 gtp_sensor_id = 0;
EXPORT_SYMBOL(gtp_sensor_id);

#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    u16 g_hall_status = 1;
    EXPORT_SYMBOL(g_hall_status);
#endif

extern char mtkfb_lcm_name[256];
extern LCM_DRIVER *lcm_driver_list[];
extern unsigned int lcm_count;

static int tp_info_show(struct seq_file *m, void *v)
{
    static bool is_tp_info_loaded = false;
    int i, count, len;
    count = sizeof(all_ctp_vendor_info) / sizeof(all_ctp_vendor_info[0]);
    if (strlen(tp_info) == 0) {
        for (i = 0; i < count; i++) {
            len = strlen(all_ctp_vendor_info[i].name);
            if (len != 0)
                strncat(tp_info, all_ctp_vendor_info[i].name, len);
            else
                strncat(tp_info, "CTPXXXX", 7);
            strncat(tp_info, "_", 1);
            len = strlen(all_ctp_vendor_info[i].md5);
            if (len != 0 && len >= 8)
                strncat(tp_info, all_ctp_vendor_info[i].md5, 8);
            else
                strncat(tp_info, "MD5XXXX", 7);
            if (gtp_sensor_id == all_ctp_vendor_info[i].id)
                strncat(tp_info, "*", 1);
            strncat(tp_info, "\n", 1);
            is_tp_info_loaded = true;
        }
    }
    if (is_tp_info_loaded == false) {
        strncat(tp_info, "UnKown\n", 7);
        is_tp_info_loaded = true;
    }
    seq_printf(m, "%s", tp_info);
    return 0;
}

static int tp_info_open(struct inode *inode, struct file *file)
{
    return single_open(file, tp_info_show, inode->i_private);
}

static const struct file_operations tp_info_proc_ops = {
    .open = tp_info_open,
    .release = single_release,
    .read = seq_read,
    .llseek = seq_lseek,
};

static int lcd_info_show(struct seq_file *m, void *v)
{
    int i;
    memset(all_disp_lcm_name, 0, sizeof(all_disp_lcm_name));
    for(i = 0; i < lcm_count; i++) {
        strcat(all_disp_lcm_name, lcm_driver_list[i]->name);
        if(!strcmp(mtkfb_lcm_name, lcm_driver_list[i]->name))
            strcat(all_disp_lcm_name, "*");
        	strcat(all_disp_lcm_name, "\n");
    }
    seq_printf(m, "%s", all_disp_lcm_name);
    return 0;
}

static int lcd_info_open(struct inode *inode, struct file *file)
{
    return single_open(file, lcd_info_show, inode->i_private);
}

static const struct file_operations lcd_info_proc_ops = {
    .open = lcd_info_open,
    .release = single_release,
    .read = seq_read,
    .llseek = seq_lseek,
};

static ssize_t gesture_wakeup_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", gesture_wakeup_flag);
    return 0;
}

static ssize_t gesture_wakeup_write(struct file *filp, const char __user *page,
                                    size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;
    if (size >= sizeof(buf))
        return -EINVAL;
    if (copy_from_user(&buf, page, size))
        return -EFAULT;
    buf[size] = 0;
    ret = kstrtoul(buf, 10, (unsigned long *)&val);
    if (val == 0)
        gesture_wakeup_flag = 0;
    else if (val == 1)
        gesture_wakeup_flag = 1;
    if (ret < 0)
        return ret;
    pr_err("%s gesture wakeup function!\n",
           (gesture_wakeup_flag ? "Enable" : "Disable"));
    return size;
}

static int gesture_wakeup_open(struct inode *inode, struct file *file)
{
    return single_open(file, gesture_wakeup_show, inode->i_private);
}

static struct file_operations gesture_wakeup_ops = {
    .open = gesture_wakeup_open,
    .release = single_release,
    .read = seq_read,
    .write = gesture_wakeup_write,
    .llseek = seq_lseek,
};

#ifdef CONFIG_MMC_FFU
static ssize_t ffu_show(struct seq_file *m, void *v)
{
    seq_printf(ptr, "%d\n", ffu_flag);
    return 0;
}

static ssize_t ffu_write(struct file *filp, const char __user *page,
                         size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;
    if (size >= sizeof(buf))
        return -EINVAL;
    if (copy_from_user(&buf, page, size))
        return -EFAULT;
    buf[size] = 0;
    ret = kstrtoul(buf, 10, (unsigned long *)&val);
    if (val == 0)
        ffu_flag = 0;
    else if (val == 1)
        ffu_flag = 1;
    if (ret < 0)
        return ret;
    pr_err("%s ffu_flag function!\n",
           (ffu_flag ? "Enable" : "Disable"));
    return size;
}

static int ffu_open(struct inode *inode, struct file *file)
{
    return single_open(file, ffu_show, inode->i_private);
}

static struct file_operations ffu_ops = {
    .open = ffu_open,
    .release = single_release,
    .read = seq_read,
    .write = ffu_write,
    .llseek = seq_lseek,
};
#endif

static ssize_t smart_cover_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", smart_cover_flag);
    return 0;
}

static ssize_t smart_cover_write(struct file *filp, const char __user *page,
                                 size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;
    if (size >= sizeof(buf))
        return -EINVAL;
    if (copy_from_user(&buf, page, size))
        return -EFAULT;
    buf[size] = 0;
    ret = kstrtoul(buf, 10, (unsigned long *)&val);
    if (val == 0)
        smart_cover_flag = 0;
    else if (val == 1)
        smart_cover_flag = 1;
    if(smart_cover_flag_charge_notify != NULL)
        smart_cover_flag_charge_notify(smart_cover_flag);
    if (ret < 0)
        return ret;
    pr_err("%s smart cover flag function!\n",
           (smart_cover_flag ? "Enable" : "Disable"));
    return size;
}

static int smart_cover_open(struct inode *inode, struct file *file)
{
    return single_open(file, smart_cover_show, inode->i_private);
}

static struct file_operations smart_cover_ops = {
    .open = smart_cover_open,
    .release = single_release,
    .read = seq_read,
    .write = smart_cover_write,
    .llseek = seq_lseek,
};

static ssize_t palm_suspend_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", palm_suspend_flag);
    return 0;
}

static ssize_t palm_suspend_write(struct file *filp, const char __user *page,
                                  size_t size, loff_t *ppos)
{
    char buf[64];
    int val, ret;
    if (size >= sizeof(buf))
        return -EINVAL;
    if (copy_from_user(&buf, page, size))
        return -EFAULT;
    buf[size] = 0;
    ret = kstrtoul(buf, 10, (unsigned long *)&val);
    if (val == 0)
        palm_suspend_flag = false;
    else if (val == 1)
        palm_suspend_flag = true;
    if (ret < 0)
        return ret;
    pr_err("%s palm suspend function!\n",
           (palm_suspend_flag ? "Enable" : "Disable"));
    return size;
}

static int palm_suspend_open(struct inode *inode, struct file *file)
{
    return single_open(file, palm_suspend_show, inode->i_private);
}

static struct file_operations palm_suspend_ops = {
    .open = palm_suspend_open,
    .release = single_release,
    .read = seq_read,
    .write = palm_suspend_write,
    .llseek = seq_lseek,
};

static int gtp_cfg_version_show(struct seq_file *m, void *v)
{
    seq_printf(m, "V%d\n", gtp_cfg_version);
    return 0;
}

static int gtp_cfg_version_open(struct inode *inode, struct file *file)
{
    return single_open(file, gtp_cfg_version_show, inode->i_private);
}

static struct file_operations gtp_cfg_ops = {
    .open = gtp_cfg_version_open,
    .release = single_release,
    .read = seq_read,
    .llseek = seq_lseek,
};

static int gtp_fw_version_show(struct seq_file *m, void *v)
{
    seq_printf(m, "0x%x\n", gtp_fw_version);
    return 0;
}

static int gtp_fw_version_open(struct inode *inode, struct file *file)
{
    return single_open(file, gtp_fw_version_show, inode->i_private);
}

static struct file_operations gtp_fw_ops = {
    .open = gtp_fw_version_open,
    .release = single_release,
    .read = seq_read,
    .llseek = seq_lseek,
};

static int gtp_sensor_id_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%x\n", gtp_sensor_id);
    return 0;
}

static int gtp_sensor_id_open(struct inode *inode, struct file *file)
{
    return single_open(file, gtp_sensor_id_show, inode->i_private);
}

static struct file_operations gtp_id_ops = {
    .open = gtp_sensor_id_open,
    .release = single_release,
    .read = seq_read,
    .llseek = seq_lseek,
};

#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
static int hall_status_show(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", g_hall_status);
}

static int hall_status_open(struct inode *inode, struct file *file)
{
    return single_open(file, hall_status_show, inode->i_private);
}

static struct file_operations hall_status_ops = {
    .open = hall_status_open,
    .release = single_release,
    .read = seq_read,
    .llseek = seq_lseek,
};
#endif

static int create_device_info_class_and_dev(void)
{
    if (!device_info_class) {
        device_info_class = class_create(THIS_MODULE, "deviceinfo");
        if (IS_ERR(device_info_class)) {
            pr_err("Failed to create class for device info!\n");
            return -1;
        }
        device_info_dev = device_create(device_info_class, NULL, 0, NULL,
                                        "device_info");
        if (IS_ERR(device_info_dev)) {
            pr_err("Failed to create dev for device info!\n");
            return -2;
        }
    }
    return 0;
}

struct device *get_deviceinfo_dev(void)
{
    if (device_info_dev)
        return device_info_dev;
    return NULL;
}
EXPORT_SYMBOL(get_deviceinfo_dev);
/*[BUGFIX]-add-begin,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/
#ifdef AUDIO_PARA_VER_MONITOR
#define AUDIO_PARA_VER "v02"
static ssize_t audio_para_ver_read(struct file *filp,char __user *page,size_t size,loff_t *ppos)
{
	char *ptr=page;
	char buf[64]={0};
	int ret=0;
	if(*ppos)
		return 0;
	ptr += sprintf(buf, "%s\n", AUDIO_PARA_VER);
    if (copy_to_user(page, buf, (ptr - page))) {
        ret = -EFAULT;
    } else {
        ret = ptr - page;
        *ppos += (ptr - page);
    }

	return ret;
}
static struct file_operations audio_info_proc_ops={
	.owner=THIS_MODULE,
	.write=NULL,
	.read=audio_para_ver_read,
};
#endif
/*[BUGFIX]-add-end,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/

static int __init tct_debug_init(void)
{
    base = proc_mkdir(BASE_DIR_NAME, NULL);
    tp_dir = proc_mkdir(TP_DIR_NAME, base);
    lcd_dir = proc_mkdir(LCD_DIR_NAME, base);
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    hall_dir = proc_mkdir(HALL_DIR_NAME, base);
#endif
#ifdef CONFIG_MMC_FFU
    ffu_dir = proc_mkdir(FFU_DIR_NAME, base);
#endif
	/*[BUGFIX]-Mod-END by TCTSZ.(gaoxiang.zou@tcl.com),*/
/*[BUGFIX]-add-begin,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/
#ifdef AUDIO_PARA_VER_MONITOR
	audio_dir=proc_mkdir(AUDIO_DIR_NAME,base);
	audio_entry=proc_create("audio_para.ver",0666,audio_dir,&audio_info_proc_ops);
#endif
/*[BUGFIX]-add-end,by scdtablet jinghuang@tcl.com,5354483 on 2017.09.21*/
    tp_entry = proc_create("tp_fw_cfg.ver", 0444, tp_dir, &tp_info_proc_ops);
    if (tp_entry == NULL) {
        pr_err("Create tp proc entry failed!\n");
        goto err_create_tp_info_entry;
    }
    lcd_entry = proc_create("initcode.ver", 0444, lcd_dir, &lcd_info_proc_ops);
    if (lcd_entry == NULL) {
        pr_err("Create lcd proc entry failed!\n");
        goto err_create_lcd_info_entry;
    }
    gesture_entry = proc_create("gesture_enable", 0666, tp_dir,
                                &gesture_wakeup_ops);
    if (gesture_entry == NULL) {
        pr_err("Create gesture enable/disable entry failed!\n");
        goto err_create_gesture_wakeup_entry;
    }
    smart_cover_entry = proc_create("smart_cover_enable", 0666, tp_dir,
                                    &smart_cover_ops);
    if (smart_cover_entry == NULL) {
        pr_err("Create smart cover enable/disable entry failed!\n");
        goto err_create_smart_cover_entry;
    }
    palm_entry = proc_create("palm_suspend_enable", 0666, tp_dir,
                             &palm_suspend_ops);
    if (palm_entry == NULL) {
        pr_err("Create palm suspend enable/disable entry failed!\n");
        goto err_create_palm_suspend_entry;
    }
    gtp_cfg_entry = proc_create("cfg_version", 0444, tp_dir,
                                &gtp_cfg_ops);
    if (gtp_cfg_entry == NULL) {
        pr_err("Create gtp cfg version entery failed!\n");
        goto err_create_gtp_cfg_entry;
    }
    gtp_fw_entry = proc_create("firmware_version", 0444, tp_dir,
                               &gtp_fw_ops);
    if (gtp_fw_entry == NULL) {
        pr_err("Create gtp firmware version entery failed!\n");
        goto err_create_gtp_fw_entry;
    }
    gtp_id_entry = proc_create("sensor_id", 0444, tp_dir,
                               &gtp_id_ops);
    if (gtp_id_entry == NULL) {
        pr_err("Create gtp sensor id entry failed!\n");
        goto err_create_gtp_sensor_id_entry;
    }
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    hall_status_entry = proc_create("hall_status", 0444, hall_dir,
                                    &hall_status_ops);
    if (hall_status_entry == NULL) {
        pr_err("Create hall status entry failed!\n");
        goto err_create_hall_status_entry;
    }
#endif
#ifdef CONFIG_MMC_FFU
    ffu_entry = proc_create("ffu_enable", 0666, ffu_dir,
                            &ffu_ops);
    if (ffu_entry == NULL) {
        pr_err("Create ffu_entry failed!\n");
        goto err_create_ffu_entry;
    }
#endif
    if (create_device_info_class_and_dev())
        pr_err("Failed to create device info class and dev!\n");
    return 0;
#ifdef CONFIG_MMC_FFU
err_create_ffu_entry:
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    proc_remove(hall_status_entry);
#endif
#endif
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
err_create_hall_status_entry:
#endif
    proc_remove(gtp_id_entry);
err_create_gtp_sensor_id_entry:
    proc_remove(gtp_fw_entry);
err_create_gtp_fw_entry:
    proc_remove(gtp_cfg_entry);
err_create_gtp_cfg_entry:
    proc_remove(palm_entry);
err_create_palm_suspend_entry:
    proc_remove(smart_cover_entry);
err_create_smart_cover_entry:
    proc_remove(gesture_entry);
err_create_gesture_wakeup_entry:
    proc_remove(lcd_entry);
err_create_lcd_info_entry:
    proc_remove(tp_entry);
err_create_tp_info_entry:
    proc_remove(lcd_dir);
    proc_remove(tp_dir);
    proc_remove(base);
    return -1;
}

static void __exit tct_debug_exit(void)
{
#ifdef CONFIG_MMC_FFU
    proc_remove(ffu_entry);
#endif
#ifdef CONFIG_CUSTOM_KERNEL_HALL_SENSOR
    proc_remove(hall_status_entry);
#endif
    proc_remove(gtp_id_entry);
    proc_remove(gtp_fw_entry);
    proc_remove(gtp_cfg_entry);
    proc_remove(gesture_entry);
    proc_remove(palm_entry);
    proc_remove(lcd_entry);
    proc_remove(tp_entry);
    proc_remove(lcd_dir);
    proc_remove(tp_dir);
    proc_remove(base);
    return;
}

module_init(tct_debug_init);
module_exit(tct_debug_exit);
MODULE_LICENSE("GPL");

