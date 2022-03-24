#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/upmu_hw.h>
#else
//#include <mt-plat/mt_gpio.h>
#include <mach/upmu_hw.h>
#include <mt-plat/upmu_common.h>
//#include <mach/gpio_const.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>

#include <mt-plat/mtk_gpio_core.h>



#endif
#define LCD_DEBUG(fmt, args...) printf(fmt, ##args)


#define LCD_VCC_EP        			(GPIO28 | 0x80000000)
#define LCD_VCC_EN        			(GPIO122 | 0x80000000)
//#define GPIO_LCD_IO        		    	(GPIO15 | 0x80000000)
#define GPIO_LCD_RST        			(GPIO83 | 0x80000000)


static void lcd_reset(unsigned int enabled)
{
    if (enabled)
    {
		mt_set_gpio_mode(GPIO_LCD_RST, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_RST, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ONE);
    }
    else
    {	
		mt_set_gpio_mode(GPIO_LCD_RST, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_RST, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_RST, GPIO_OUT_ZERO);    
    }
}
static void lcd_vcc_ep(unsigned int enabled)
{
    if (enabled)
    {
		mt_set_gpio_mode(LCD_VCC_EP, GPIO_MODE_00);
		mt_set_gpio_dir(LCD_VCC_EP, GPIO_DIR_OUT);
		mt_set_gpio_out(LCD_VCC_EP, GPIO_OUT_ONE);

    }
    else
    {	
		mt_set_gpio_mode(LCD_VCC_EP, GPIO_MODE_00);
		mt_set_gpio_dir(LCD_VCC_EP, GPIO_DIR_OUT);
		mt_set_gpio_out(LCD_VCC_EP, GPIO_OUT_ZERO);
    }
}
static void lcd_vcc_en(unsigned int enabled)
{
    if (enabled)
    {
		mt_set_gpio_mode(LCD_VCC_EN, GPIO_MODE_00);
		mt_set_gpio_dir(LCD_VCC_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(LCD_VCC_EN, GPIO_OUT_ONE);
    }
    else
    {	
		mt_set_gpio_mode(LCD_VCC_EN, GPIO_MODE_00);
		mt_set_gpio_dir(LCD_VCC_EN, GPIO_DIR_OUT);
		mt_set_gpio_out(LCD_VCC_EN, GPIO_OUT_ZERO);
	
    }
}
#if 0
static void lcd_io_en(unsigned int enabled)
{
    if (enabled)
    {
		mt_set_gpio_mode(GPIO_LCD_IO, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_IO, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_IO, GPIO_OUT_ONE);

    }
    else
    {	
		mt_set_gpio_mode(GPIO_LCD_IO, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_LCD_IO, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_LCD_IO, GPIO_OUT_ZERO);   	
    }
}
#endif
/*[bugfix]-mod-begin,by scdtablet jinghuang@tcl.com,5869406 on 20180111*/
/*modify suspend sequence,reset high and close +-5.8v*/
void lcd_init_power_sequence(unsigned int enable)
	{
	
		if(enable==1){
			/*init power*/
			//lcd_vcc_en(0);
			//lcd_vcc_ep(0);
			//lcd_reset(0);
			//mdelay(5);
			lcd_vcc_ep(1);
			mdelay(5);
			lcd_vcc_en(1);
			mdelay(10);
		}
		else if(enable==2){
			/*esd  power off*/
			mdelay(5);
			lcd_reset(0);
			mdelay(5);
			lcd_vcc_en(0);
			mdelay(1);
			lcd_vcc_ep(0);
			mdelay(5);

		}
		else if(enable==3){
			/*suspend power off*/
			lcd_reset(0);
			mdelay(15);
			lcd_vcc_en(0);
			mdelay(1);
			lcd_vcc_ep(0);
			mdelay(5);
			
		}
		else if(enable==4){
			/*device shutdown power off*/
			mdelay(5);
			lcd_reset(0);
			mdelay(5);
			lcd_vcc_en(0);
			mdelay(1);
			lcd_vcc_ep(0);
			mdelay(5);
			
		}
		else{
			/*default power off*/
			/*[BUGFIX]-add-begin,by scdtabelt jinghuang@tcl.com,5285627 on 20170905*/
			/*modify lcm power off timming*/
			mdelay(5);
			/*[BUGFIX]-add-begin,by scdtabelt jinghuang@tcl.com,5285627 on 20170905*/
			lcd_reset(0);
			mdelay(5);
			lcd_vcc_en(0);
			mdelay(1);
			lcd_vcc_ep(0);
			mdelay(5);
				
			}

}
EXPORT_SYMBOL(lcd_init_power_sequence);
/*[bugfix]-mod-end,by scdtablet jinghuang@tcl.com,5869406 on 20180111*/

#if 0
#ifndef BUILD_LK
#define TPS_I2C_NAME "tps65132"
#define TPS_I2C_BUSNUM 1
#ifndef CONFIG_OF
static static struct i2c_board_info tps65132_board_info __initdata =
{
    I2C_BOARD_INFO(TPS_I2C_NAME, 0x3E)
};
#else
static const struct of_device_id lcm_of_match[] = {
    {.compatible = "mediatek,i2c_lcd_bias"},
    {}
};
#endif
static struct i2c_client *tps65132_i2c_client;
static int tps65132_probe(struct i2c_client *client,
                          const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
static const struct i2c_device_id tps65132_id[] = {
    {TPS_I2C_NAME, 0},
    {}
};
static struct i2c_driver tps65132_i2c_driver = {
    .id_table = tps65132_id,
    .probe    = tps65132_probe,
    .remove   = tps65132_remove,
    .driver   = {
        .owner = THIS_MODULE,
        .name  = TPS_I2C_NAME,
#ifdef CONFIG_OF
        .of_match_table = lcm_of_match,
#endif
    }
};

static int tps65132_probe(struct i2c_client *client,
                          const struct i2c_device_id *id)
{
    pr_debug("tps65132_iic_probe\n");
    pr_debug("TPS: info==>name=%s addr=0x%x\n", client->name, client->addr);
    tps65132_i2c_client  = client;
    return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
    pr_debug("tps65132_remove\n");
    tps65132_i2c_client = NULL;
    i2c_unregister_device(client);
    return 0;
}

int tps65132_write_byte(unsigned char addr, unsigned char value)
{
    int ret = 0;
    struct i2c_client *client = tps65132_i2c_client;
    unsigned char write_data[2] = {0};

    pr_debug("%s:addr = 0x%x, value = 0x%x\n", __func__, addr, value);
    if (client == NULL) {
        pr_err("ERROR!!tps65132_i2c_client is null\n");
        return 0;
    }

    write_data[0] = addr;
    write_data[1] = value;
    ret = i2c_master_send(client, write_data, 2);
    if (ret < 0)
        pr_err("tps65132 write data fail !!\n");
    return ret;
}
EXPORT_SYMBOL_GPL(tps65132_write_byte);

static int __init tps65132_i2c_init(void)
{
    pr_debug("%s\n", __func__);
#ifndef CONFIG_OF
    i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
    pr_debug("%s\n", __func__);
#endif
    i2c_add_driver(&tps65132_i2c_driver);
    pr_debug("%s success\n", __func__);
    return 0;
}

static void __exit tps65132_i2c_exit(void)
{
    pr_debug("%s\n", __func__);
    i2c_del_driver(&tps65132_i2c_driver);
}

module_init(tps65132_i2c_init);
module_exit(tps65132_i2c_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");
#endif
#endif
