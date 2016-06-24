/*
 * leds-sm5701.c
 *
 * Copyright (c) 2014 Silicon Mitus Co., Ltd
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/mfd/sm5701_core.h>
#include <linux/platform_data/leds-sm5701.h>


enum sm5701_oper_mode {
        SUSPEND_MODE = 0,
        CHARGING_OFF_MODE,
        CHARGING_ON_MODE,
        CHARGING_FLASH_BOOST_MODE
};

enum sm5701_flash_mode {
        NONE_MODE = 0,
        FLASH_MODE,
        MOVIE_MODE
};
    
struct sm5701_leds_data {
        struct device *dev;
        struct sm5701_dev *iodev;

        struct led_classdev cdev_flash;
        struct led_classdev cdev_movie;
        
        struct work_struct work_flash;
        struct work_struct work_movie;

        u8 br_flash; //IFLED Current in Flash Mode
        u8 br_movie; //IMLED Current in Movie Mode
        
        //struct sm5701_platform_data *pdata;

        struct mutex lock;
};

static struct i2c_client * leds_sm5701_client = NULL;

#define ENABSTMR_SHIFT  4
void sm5701_set_enabstmr(int abstmr_enable)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s abstmr_enable = %d\n",__func__,abstmr_enable);

    sm5701_reg_read(client, SM5701_FLEDCNTL1, &data);

    data = (data & (~SM5701_FLEDCNTL1_ENABSTMR)) | (abstmr_enable << ENABSTMR_SHIFT);

    sm5701_reg_write(client,SM5701_FLEDCNTL1,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL1, data);
    
}
EXPORT_SYMBOL(sm5701_set_enabstmr);

#define ABSTMR_SHIFT  2
void sm5701_set_abstmr(int abstmr_sec)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s abstmr_sec = %d\n",__func__,abstmr_sec);

    sm5701_reg_read(client, SM5701_FLEDCNTL1, &data);

    data = (data & (~SM5701_FLEDCNTL1_ABSTMR)) | (abstmr_sec << ABSTMR_SHIFT);

    sm5701_reg_write(client,SM5701_FLEDCNTL1,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL1, data);
    
}
EXPORT_SYMBOL(sm5701_set_abstmr);

void sm5701_set_fleden(int fled_enable)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s fled_enable = %d\n",__func__,fled_enable);

    sm5701_reg_read(client, SM5701_FLEDCNTL1, &data);
    
    data = (data & (~SM5701_FLEDCNTL1_FLEDEN)) | fled_enable;

    sm5701_reg_write(client,SM5701_FLEDCNTL1,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL1, data);
    
}
EXPORT_SYMBOL(sm5701_set_fleden);

#define nENSAFET_SHIFT  7
void sm5701_set_nensafet(int nensafet_enable)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s nensafet_enable = %d\n",__func__,nensafet_enable);

    sm5701_reg_read(client, SM5701_FLEDCNTL2, &data);

    data = (data & (~SM5701_FLEDCNTL2_nENSAFET)) | (nensafet_enable << nENSAFET_SHIFT);

    sm5701_reg_write(client,SM5701_FLEDCNTL2,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL2, data);
    
}
EXPORT_SYMBOL(sm5701_set_nensafet);

#define SAFET_SHIFT  5
void sm5701_set_safet(int safet_us)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s safet_us = %d\n",__func__,safet_us);

    sm5701_reg_read(client, SM5701_FLEDCNTL2, &data);

    data = (data & (~SM5701_FLEDCNTL2_SAFET)) | (safet_us << SAFET_SHIFT);

    sm5701_reg_write(client,SM5701_FLEDCNTL2,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL2, data);
    
}
EXPORT_SYMBOL(sm5701_set_safet);

#define nONESHOT_SHIFT  4
void sm5701_set_noneshot(int noneshot_enable)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s noneshot_enable = %d\n",__func__,noneshot_enable);

    sm5701_reg_read(client, SM5701_FLEDCNTL2, &data);

    data = (data & (~SM5701_FLEDCNTL2_nONESHOT)) | (noneshot_enable << nONESHOT_SHIFT);

    sm5701_reg_write(client,SM5701_FLEDCNTL2,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL2, data);
    
}
EXPORT_SYMBOL(sm5701_set_noneshot);

void sm5701_set_onetimer(int onetimer_ms)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s onetimer_ms = %d\n",__func__,onetimer_ms);

    sm5701_reg_read(client, SM5701_FLEDCNTL2, &data);

    data = (data & (~SM5701_FLEDCNTL2_ONETIMER)) | onetimer_ms;

    sm5701_reg_write(client,SM5701_FLEDCNTL2,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL2, data);
    
}
EXPORT_SYMBOL(sm5701_set_onetimer);

void sm5701_set_ifled(int ifled_ma)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s ifled_ma = %d\n",__func__,ifled_ma);

    sm5701_reg_read(client, SM5701_FLEDCNTL3, &data);

    data = (data & (~SM5701_FLEDCNTL3_IFLED)) | ifled_ma;

    sm5701_reg_write(client,SM5701_FLEDCNTL3,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL3, data);
    
}
EXPORT_SYMBOL(sm5701_set_ifled);

void sm5701_set_imled(int imled_ma)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s imled_ma = %d\n",__func__,imled_ma);

    sm5701_reg_read(client, SM5701_FLEDCNTL4, &data);

    data = (data & (~SM5701_FLEDCNTL4_IMLED)) | imled_ma;

    sm5701_reg_write(client,SM5701_FLEDCNTL4,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL4, data);
    
}
EXPORT_SYMBOL(sm5701_set_imled);

#define ENLOWBATT_SHIFT  7
void sm5701_set_enlowbatt(int enlowbatt_enable)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s enlowbatt_enable = %d\n",__func__,enlowbatt_enable);

    sm5701_reg_read(client, SM5701_FLEDCNTL5, &data);

    data = (data & (~SM5701_FLEDCNTL5_ENLOWBATT)) | (enlowbatt_enable << ENLOWBATT_SHIFT);

    sm5701_reg_write(client,SM5701_FLEDCNTL5,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL5, data);
    
}
EXPORT_SYMBOL(sm5701_set_enlowbatt);

#define LBRSTIMER_SHIFT  5
void sm5701_set_lbrstimer(int lbrstimer_us)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s lbrstimer_us = %d\n",__func__,lbrstimer_us);

    sm5701_reg_read(client, SM5701_FLEDCNTL5, &data);

    data = (data & (~SM5701_FLEDCNTL5_LBRSTIMER)) | (lbrstimer_us << LBRSTIMER_SHIFT);

    sm5701_reg_write(client,SM5701_FLEDCNTL5,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL5, data);
    
}
EXPORT_SYMBOL(sm5701_set_lbrstimer);

#define LOWBATT_SHIFT  2
void sm5701_set_lowbatt(int lowbatt_v)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s lowbatt_v = %d\n",__func__,lowbatt_v);

    sm5701_reg_read(client, SM5701_FLEDCNTL5, &data);

    data = (data & (~SM5701_FLEDCNTL5_LOWBATT)) | (lowbatt_v << LOWBATT_SHIFT);

    sm5701_reg_write(client,SM5701_FLEDCNTL5,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL5, data);
    
}
EXPORT_SYMBOL(sm5701_set_lowbatt);

void sm5701_set_lbdhys(int lbdhys_mv)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s lbdhys_mv = %d\n",__func__,lbdhys_mv);

    sm5701_reg_read(client, SM5701_FLEDCNTL5, &data);

    data = (data & (~SM5701_FLEDCNTL5_LBDHYS))| lbdhys_mv;

    sm5701_reg_write(client,SM5701_FLEDCNTL5,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL5, data);
    
}
EXPORT_SYMBOL(sm5701_set_lbdhys);

void sm5701_set_bstout(int bstout_mv)
{
    struct i2c_client * client;
    u8 data = 0;

    client = leds_sm5701_client;
  
    if( !client) return;

    pr_info("%s bstout_mv = %d\n",__func__,bstout_mv);

    sm5701_reg_read(client, SM5701_FLEDCNTL6, &data);

    data = (data & (~SM5701_FLEDCNTL6_BSTOUT))| bstout_mv;

    sm5701_reg_write(client,SM5701_FLEDCNTL6,data);    
  
    pr_info("write SM5701 addr : 0x%02x data : 0x%02x\n", SM5701_FLEDCNTL6, data);
    
}
EXPORT_SYMBOL(sm5701_set_bstout);

/* chip control */
static int sm5701_control(struct sm5701_leds_data *chip,
                          u8 brightness, enum sm5701_oper_mode opmode)
{
        switch (opmode) {
        case NONE_MODE:
                sm5701_led_ready(LED_DISABLE);
                sm5701_set_fleden(SM5701_FLEDEN_DISABLED);
                SM5701_operation_mode_function_control();
                break;

        case FLASH_MODE:            
                sm5701_led_ready(FLASH_MODE);
                sm5701_set_ifled(brightness);
                SM5701_operation_mode_function_control();
                sm5701_set_fleden(SM5701_FLEDEN_ON_FLASH);
                break;

        case MOVIE_MODE:
                sm5701_led_ready(MOVIE_MODE);
                sm5701_set_imled(brightness);
                SM5701_operation_mode_function_control();                
                sm5701_set_fleden(SM5701_FLEDEN_ON_MOVIE);
                break;

        default:
                break;
        }
        
        return opmode;
}

/* movie */

/* movie config for sm5701*/
static ssize_t sm5701_movie_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf, size_t size)
{
        ssize_t ret;
        struct led_classdev *led_cdev = dev_get_drvdata(dev);
        struct sm5701_leds_data *chip =
            container_of(led_cdev, struct sm5701_leds_data, work_movie);
        unsigned int state;

        ret = kstrtouint(buf, 10, &state);
        if (ret)
                goto out_strtoint;
        
        if (state == 0)
                sm5701_set_fleden(SM5701_FLEDEN_DISABLED);
        else if (state == 1) 
                sm5701_set_fleden(SM5701_FLEDEN_ON_MOVIE);
        else 
                sm5701_set_fleden(SM5701_FLEDEN_DISABLED);

        //sm5701_dump_register();
        
        return size;

out_strtoint:
        dev_err(chip->dev, "%s: fail to change str to int\n", __func__);
        return ret;
}

static DEVICE_ATTR(movie, S_IWUSR, NULL, sm5701_movie_store);

static void sm5701_deferred_movie_brightness_set(struct work_struct *work)
{
    struct sm5701_leds_data *chip =
        container_of(work, struct sm5701_leds_data, work_movie);

    if(chip->br_movie == 0)
    {
        mutex_lock(&chip->lock);
        sm5701_control(chip, chip->br_movie, NONE_MODE);
        mutex_unlock(&chip->lock);
    }
    else
    {
        mutex_lock(&chip->lock);
        sm5701_control(chip, chip->br_movie, MOVIE_MODE);
        mutex_unlock(&chip->lock);
    }

}

static void sm5701_movie_brightness_set(struct led_classdev *cdev,
                                        enum led_brightness brightness)
{
        struct sm5701_leds_data *chip =
            container_of(cdev, struct sm5701_leds_data, cdev_movie);

        chip->br_movie = brightness;
        schedule_work(&chip->work_movie);
}

/* flash */

/* flash config for sm5701*/
static ssize_t sm5701_flash_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t size)
{
        ssize_t ret;
        struct led_classdev *led_cdev = dev_get_drvdata(dev);
        struct sm5701_leds_data *chip =
            container_of(led_cdev, struct sm5701_leds_data, work_flash);
        unsigned int state;

        ret = kstrtouint(buf, 10, &state);
        if (ret)
                goto out_strtoint;
        
        if (state == 0)
                sm5701_set_fleden(SM5701_FLEDEN_DISABLED);
        else if (state == 1) 
                sm5701_set_fleden(SM5701_FLEDEN_ON_FLASH);
        else 
                sm5701_set_fleden(SM5701_FLEDEN_DISABLED);

        //sm5701_dump_register();

        return size;
        
out_strtoint:
        dev_err(chip->dev, "%s: fail to change str to int\n", __func__);
        return ret;
}

static DEVICE_ATTR(flash, S_IWUSR, NULL, sm5701_flash_store);

static void sm5701_deferred_flash_brightness_set(struct work_struct *work)
{
    struct sm5701_leds_data *chip =
        container_of(work, struct sm5701_leds_data, work_flash);
    
    if(chip->br_flash == 0)
    {
        mutex_lock(&chip->lock);
        sm5701_control(chip, chip->br_flash, NONE_MODE);
        mutex_unlock(&chip->lock);          
    }
    else
    {
        mutex_lock(&chip->lock);
        sm5701_control(chip, chip->br_flash, FLASH_MODE);
        mutex_unlock(&chip->lock);
    }

}

static void sm5701_flash_brightness_set(struct led_classdev *cdev,
                                         enum led_brightness brightness)
{
        struct sm5701_leds_data *chip =
            container_of(cdev, struct sm5701_leds_data, cdev_flash);

        chip->br_flash = brightness;
        schedule_work(&chip->work_flash);
}

/* chip initialize */
static int sm5701_chip_init(struct sm5701_leds_data *chip)
{
        int ret = 0;

        return ret;
}


static __devinit int leds_sm5701_probe(struct platform_device *pdev)
{
    	struct sm5701_dev *iodev = dev_get_drvdata(pdev->dev.parent);
    	struct sm5701_platform_data *pdata = dev_get_platdata(iodev->dev);
        struct sm5701_leds_data *chip;

        int err;

        printk("******* %s *******\n",__func__);

        if (!pdata) {
            dev_err(pdev->dev.parent, "Platform data not supplied\n");
            return -ENODEV;
        }

    	chip = kzalloc(sizeof(struct sm5701_leds_data), GFP_KERNEL);        
        if (!chip)
           return -ENOMEM;

        chip->dev = &pdev->dev;
        chip->iodev = iodev;
        platform_set_drvdata(pdev, chip);
        
        if (!(leds_sm5701_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
            return -ENOMEM;
        }    
        memset(leds_sm5701_client, 0, sizeof(struct i2c_client));

        leds_sm5701_client = chip->iodev->i2c;    

        mutex_init(&chip->lock);

        err = sm5701_chip_init(chip);
        if (err < 0)
                goto err_out;

        /* flash */
        INIT_WORK(&chip->work_flash, sm5701_deferred_flash_brightness_set);
        chip->cdev_flash.name = "flash";
        chip->cdev_flash.max_brightness = 32;
        chip->cdev_flash.brightness_set = sm5701_flash_brightness_set;
        chip->cdev_flash.default_trigger = "flash";
        err = led_classdev_register((struct device *)
                                    chip->dev, &chip->cdev_flash);

        if (err < 0) {
                dev_err(chip->dev, "failed to register flash\n");
                goto err_create_flash_file;
        }
        err = device_create_file(chip->cdev_flash.dev, &dev_attr_flash);
        if (err < 0) {
                dev_err(chip->dev, "failed to create flash file\n");
                goto err_create_flash_pin_file;
        }

        /* movie */
        INIT_WORK(&chip->work_movie, sm5701_deferred_movie_brightness_set);
        chip->cdev_movie.name = "movie";
        chip->cdev_movie.max_brightness = 32;
        chip->cdev_movie.brightness_set = sm5701_movie_brightness_set;
        chip->cdev_movie.default_trigger = "movie";
        err = led_classdev_register((struct device *)
                                    chip->dev, &chip->cdev_movie);
        if (err < 0) {
                dev_err(chip->dev, "failed to register movie\n");
                goto err_create_movie_file;
        }
        err = device_create_file(chip->cdev_movie.dev, &dev_attr_movie);
        if (err < 0) {
                dev_err(chip->dev, "failed to create movie file\n");
                goto err_create_movie_pin_file;
        }

        dev_info(chip->dev, "LEDs_SM5701 Probe Done\n");
      return 0;

err_create_movie_file:
        device_remove_file(chip->cdev_movie.dev, &dev_attr_movie);
err_create_movie_pin_file:
        led_classdev_unregister(&chip->cdev_movie);
err_create_flash_file:
        device_remove_file(chip->cdev_flash.dev, &dev_attr_flash);
err_create_flash_pin_file:
        led_classdev_unregister(&chip->cdev_flash);
err_out:
        return err;
}

static int __devexit leds_sm5701_remove(struct platform_device *pdev)
{
        struct sm5701_leds_data *chip = platform_get_drvdata(pdev);

        device_remove_file(chip->cdev_movie.dev, &dev_attr_movie);
        led_classdev_unregister(&chip->cdev_movie);
        flush_work(&chip->work_movie);
        device_remove_file(chip->cdev_flash.dev, &dev_attr_flash);
        led_classdev_unregister(&chip->cdev_flash);
        flush_work(&chip->work_flash);
        return 0;
}

static const struct platform_device_id leds_sm5701_id[] = {
        {"leds_sm5701", 0},
        {}
};

MODULE_DEVICE_TABLE(platform, leds_sm5701_id);

static struct platform_driver leds_sm5701_driver = {
        .driver = {
                   .name = "leds_sm5701",
                   .owner = THIS_MODULE,
                   },
        .probe = leds_sm5701_probe,
        .remove = leds_sm5701_remove,
        .id_table = leds_sm5701_id,
};

static int __init leds_sm5701_init(void)
{
    printk("******* %s *******\n",__func__);

	return platform_driver_register(&leds_sm5701_driver);
}

subsys_initcall(leds_sm5701_init);

static void __exit leds_sm5701_exit(void)
{
	platform_driver_unregister(&leds_sm5701_driver);
}
module_exit(leds_sm5701_exit);

/* Module information */
MODULE_DESCRIPTION("SILICONMITUS SM5701 Flash LED Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-sm5701");

