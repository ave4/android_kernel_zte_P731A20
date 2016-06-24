/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : mxc400x_acc.c
 * Description        : MXC400X accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *

 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include <linux/module.h>
#include	<linux/input.h>
#include	<linux/input-polldev.h>
#include	<linux/miscdevice.h>
#include	<linux/uaccess.h>
#include  <linux/slab.h>

#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include        <linux/earlysuspend.h>
#endif

#include        <linux/i2c/lis3dh.h>
#define	G_MAX		16000	/** Maximum polled-device-reported g value */
#include        "mxc400x.h"

#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5

#define DEVICE_INFO         "ST, LIS3DH"
#define DEVICE_INFO_LEN     32

/* end RESUME STATE INDICES */

#define DEBUG    1
#define MXC400X_DEBUG  1 

#define	MAX_INTERVAL	50

#ifdef __KERNEL__
static struct mxc400x_acc_platform_data mxc400x_plat_data = {
    .poll_interval = 20,
    .min_interval = 10,
};
#endif

#ifdef I2C_BUS_NUM_STATIC_ALLOC
static struct i2c_board_info  mxc400x_i2c_boardinfo = {
        I2C_BOARD_INFO(MXC400X_ACC_DEV_NAME, MXC400X_ACC_I2C_ADDR),
#ifdef __KERNEL__
        .platform_data = &mxc400x_plat_data
#endif
};
#endif

struct mxc400x_acc_data {
	struct i2c_client *client;
	struct mxc400x_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;
#endif
};

/*
 * Because misc devices can not carry a mxc400x from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct mxc400x_acc_data *mxc400x_acc_misc_data;
struct i2c_client      *mxc400x_i2c_client;

static int mxc400x_acc_i2c_read(struct mxc400x_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf, },
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf, },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	
	return err;
}

static int mxc400x_acc_i2c_write(struct mxc400x_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = { { .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1, .buf = buf, }, };
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int mxc400x_acc_hw_init(struct mxc400x_acc_data *acc)
{
	int err = -1;
	u8 buf[7] = {0};

	printk(KERN_INFO "%s: hw init start\n", MXC400X_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = mxc400x_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		acc->hw_working = 1;
	if ((buf[0] & 0x3F) != WHOAMI_MXC400X_ACC) {
		err = -1; /* choose the right coded error */
		goto error_unknown_device;
	}

	acc->hw_initialized = 1;
	printk(KERN_INFO "%s: hw init done\n", MXC400X_ACC_DEV_NAME);
	return 0;

error_firstread:
	acc->hw_working = 0;
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
	goto error1;
error_unknown_device:
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_MXC400X_ACC, buf[0]);
error1:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void mxc400x_acc_device_power_off(struct mxc400x_acc_data *acc)
{
	int err;
	u8 buf[2] = { MXC400X_REG_CTRL, MXC400X_CTRL_PWRDN };

	err = mxc400x_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);
}

static int mxc400x_acc_device_power_on(struct mxc400x_acc_data *acc)
{
    
	int err = -1;
	u8 buf[2] = {MXC400X_REG_FAC, MXC400X_PASSWORD};
	printk(KERN_INFO "%s: %s entry\n",MXC400X_ACC_DEV_NAME, __func__);
	//+/-2G
/*	
	err = mxc400x_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft factory password write failed: %d\n", err);
		
	buf[0] = MXC400X_REG_FSRE;
	buf[1] = MXC400X_RANGE_8G;
	err = mxc400x_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft change to +/8g failed: %d\n", err);
*/		
	buf[0] = MXC400X_REG_CTRL;
	buf[1] = MXC400X_CTRL_PWRON;
	err = mxc400x_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power on failed: %d\n", err);
	msleep(300);
	if (!acc->hw_initialized) {
		err = mxc400x_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			mxc400x_acc_device_power_off(acc);
			return err;
		}
	}

	return 0;
}

static int mxc400x_acc_register_read(struct mxc400x_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = mxc400x_acc_i2c_read(acc, buf, 1);
	return err;
}

/* */

static int mxc400x_acc_get_acceleration_data(struct mxc400x_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware x, y,z */
	unsigned char acc_data[MXC400X_DATA_LENGTH] = {0, 0, 0, 0, 0, 0};
    s16 hw_d[3] = { 0 };
   
	acc_data[0] = MXC400X_REG_X;
	err = mxc400x_acc_i2c_read(acc, acc_data, MXC400X_DATA_LENGTH);
	//printk("acc[0] = %x, acc[1] = %x, acc[2] = %x, acc[3] = %x, acc[4] = %x, acc[5] = %x\n", acc_data[0], acc_data[1], acc_data[2], 																								acc_data[3], acc_data[4], acc_data[5]);
    																		
	if (err < 0)
    {
        #ifdef MXC400X_DEBUG
        printk(KERN_INFO "%s I2C read xy error %d\n", MXC400X_ACC_DEV_NAME, err);
        #endif
		return err;
    }
    
	
    xyz[0] = (signed short)(acc_data[0] << 8 | acc_data[1]) >> 4;
	xyz[1] = (signed short)(acc_data[2] << 8 | acc_data[3]) >> 4;
	xyz[2] = (signed short)(acc_data[4] << 8 | acc_data[5]) >> 4;
	#if 0//def MXC400X_DEBUG
	hw_d[0] = hw_d[0] * 4;//acc->sensitivity;
	hw_d[1] = hw_d[1] * 4;// acc->sensitivity;
	hw_d[2] = hw_d[2] * 4;//acc->sensitivity;
	
	printk("=====%s read x=%d, y=%d, z=%d\n",MXC400X_ACC_DEV_NAME, hw_d[0], hw_d[1], hw_d[2]);
	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		  : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		  : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		  : (hw_d[acc->pdata->axis_map_z]));
	printk("=====%s read x=%d, y=%d, z=%d\n",MXC400X_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);

	#endif
	return err;
}

static void mxc400x_acc_report_values(struct mxc400x_acc_data *acc, int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, -xyz[1]);
	input_report_abs(acc->input_dev, ABS_Y, -xyz[0]);
	input_report_abs(acc->input_dev, ABS_Z, -xyz[2]);
	input_sync(acc->input_dev);
}

static int mxc400x_acc_enable(struct mxc400x_acc_data *acc)
{
	int err;
    printk("mxc400x_acc_enable\n");
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = mxc400x_acc_device_power_on(acc);
		
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}

  //  schedule_delayed_work(&acc->input_work,
	//			      msecs_to_jiffies(acc->pdata->
	//					       poll_interval));
	}

	return 0;
}

static int mxc400x_acc_disable(struct mxc400x_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
//        cancel_delayed_work_sync(&acc->input_work);
		mxc400x_acc_device_power_off(acc);
	}

	return 0;
}

static int mxc400x_acc_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = mxc400x_acc_misc_data;

	return 0;
}
static atomic_t	a_flag;
static atomic_t	m_flag;
static atomic_t	o_flag;
static short ptacc_delay = 20;
short flag;
static long mxc400x_acc_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short delay;
	int err;
	int interval;
    	int xyz[3] = {0,0,0};
	int reg;
	int parms[3] = {0, 0, 0};
	struct mxc400x_acc_data *acc = file->private_data;

//	printk(KERN_INFO "%s: %s call with cmd 0x%x and arg 0x%x\n",
//			MXC400X_ACC_DEV_NAME, __func__, cmd, (unsigned int)arg);
	mutex_lock(&acc->lock);
    //printk("mxc400x_acc_misc_ioctl  cmd=%d",cmd);
	switch (cmd) {
	case LIS3DH_ACC_IOCTL_GET_DELAY:
		interval = acc->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		break;

	case LIS3DH_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		if (interval < 0 || interval > 1000){
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
		if(interval > MAX_INTERVAL)
			interval = MAX_INTERVAL;
		ptacc_delay = delay;
		acc->pdata->poll_interval = max(interval,
				acc->pdata->min_interval);
		break;

	case LIS3DH_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		if (interval > 1){
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
		if (interval)
			err = mxc400x_acc_enable(acc);
		else
			err = mxc400x_acc_disable(acc);

		atomic_set(&a_flag, interval);
		mutex_unlock(&acc->lock);
		return err;
		break;

	case LIS3DH_ACC_IOCTL_GET_ENABLE:
		interval = atomic_read(&acc->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}	
		break;
    case LIS3DH_ACC_IOCTL_SET_G_RANGE:

		break;
	case LIS3DH_ACC_IOCTL_GET_COOR_XYZ:    
		err = mxc400x_acc_get_acceleration_data(acc, xyz);                
		if (err < 0){
			mutex_unlock(&acc->lock);                    
			return err;
		}	
               
		if (copy_to_user(argp, xyz, sizeof(xyz))) {			
			printk(KERN_ERR " %s %d error in copy_to_user \n",					 
				__func__, __LINE__);
			mutex_unlock(&acc->lock);			
			return -EINVAL;                
		}                
		break;
#if 0
	case MXC400X_IOCTL_GET_TEMP:    
	{	
		u8  tempture = 0; 
		
		err = mxc400x_acc_register_read(acc, &tempture,MXC400X_REG_TEMP);                
		if (err < 0)  
		{
			printk("%s, error read register MXC400X_REG_TEMP\n", __func__);
			mutex_unlock(&acc->lock);
			return err;
		}   
		     	
		if (copy_to_user(argp, &tempture, sizeof(tempture))) {			
			printk(KERN_ERR " %s %d error in copy_to_user \n",					 
				__func__, __LINE__);
			mutex_unlock(&acc->lock);			
			return -EINVAL;                
		}    
	}            
		break;
#endif
	case LIS3DH_ACC_IOCTL_GET_CHIP_ID:
	{
		u8 devid = 0;
		err = mxc400x_acc_register_read(acc, &devid, WHO_AM_I);
		if (err < 0) {
			printk("%s, error read register WHO_AM_I\n", __func__);
			mutex_unlock(&acc->lock);
			return -EAGAIN;
		}
		
		if (copy_to_user(argp, &devid, sizeof(devid))) {
			printk("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
	}
            break;
  case ACC_IOC_SET_MODE:
		break;
	case ACC_IOC_SET_DELAY:
		if (copy_from_user(&delay, argp, sizeof(delay)))
		{
			  mutex_unlock(&acc->lock);	
				return -EFAULT;
		}
		ptacc_delay = delay;
		break;
	case ACC_IOC_GET_DELAY:
		delay = ptacc_delay;
		if (copy_to_user(argp, &delay, sizeof(delay)))
		{
			  mutex_unlock(&acc->lock);	
			  return -EFAULT;
		}
		break;

	case ACC_IOC_SET_AFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
		{
			  mutex_unlock(&acc->lock);	
				return -EFAULT;
		}
		if (flag < 0 || flag > 1)
		{
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		atomic_set(&a_flag, flag);
		break;
	case ACC_IOC_GET_AFLAG:
		flag = atomic_read(&a_flag);
		if (copy_to_user(argp, &flag, sizeof(flag)))
	  {
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		break;
	case ACC_IOC_SET_MFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
		{
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		if (flag < 0 || flag > 1)
		{
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		atomic_set(&m_flag, flag);
		break;
	case ACC_IOC_GET_MFLAG:
		flag = atomic_read(&m_flag);
		if (copy_to_user(argp, &flag, sizeof(flag)))
		{
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		break;
	case ACC_IOC_SET_OFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
		{
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		if (flag < 0 || flag > 1)
		{
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		atomic_set(&o_flag, flag);
		break;
	case ACC_IOC_GET_OFLAG:
		flag = atomic_read(&o_flag);
		if (copy_to_user(argp, &flag, sizeof(flag)))
		{
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		break;

	case ACC_IOC_SET_APARMS:
		if (copy_from_user(parms, argp, sizeof(parms)))
		{
			  mutex_unlock(&acc->lock);	
				return -EINVAL;
		}
		//printk("input hal data %d %d %d\n", parms[0], parms[1],parms[2]);
	    
		mxc400x_acc_report_values(acc,parms);
		break;
		case MXC400X_ACC_IOCTL_GET_DELAY:
		interval = acc->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		break;

	case MXC400X_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		if (interval < 0 || interval > 1000){
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
		if(interval > MAX_INTERVAL)
			interval = MAX_INTERVAL;
		acc->pdata->poll_interval = max(interval,
				acc->pdata->min_interval);
		break;

	case MXC400X_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		if (interval > 1){
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
		if (interval)
			err = mxc400x_acc_enable(acc);
		else
			err = mxc400x_acc_disable(acc);
		
		mutex_unlock(&acc->lock);
		return err;
		break;

	case MXC400X_ACC_IOCTL_GET_ENABLE:
		interval = atomic_read(&acc->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval))){
      mutex_unlock(&acc->lock);
			return -EINVAL;
		}	
		break;
	case MXC400X_ACC_IOCTL_GET_COOR_XYZ:    
		err = mxc400x_acc_get_acceleration_data(acc, xyz);                
		if (err < 0){
			mutex_unlock(&acc->lock);                    
			return err;
		}	
               
		if (copy_to_user(argp, xyz, sizeof(xyz))) {			
			printk(KERN_ERR " %s %d error in copy_to_user \n",					 
				__func__, __LINE__);
			mutex_unlock(&acc->lock);			
			return -EINVAL;                
		}                
		break;
	case MXC400X_ACC_IOCTL_GET_TEMP:    
	{	
		u8  tempture = 0; 
		
		err = mxc400x_acc_register_read(acc, &tempture,MXC400X_REG_TEMP);                
		if (err < 0)  
		{
			printk("%s, error read register MXC400X_REG_TEMP\n", __func__);
			mutex_unlock(&acc->lock);
			return err;
		}   
		     	
		if (copy_to_user(argp, &tempture, sizeof(tempture))) {			
			printk(KERN_ERR " %s %d error in copy_to_user \n",					 
				__func__, __LINE__);
			mutex_unlock(&acc->lock);			
			return -EINVAL;                
		}    
	}            
		break;
	case MXC400X_ACC_IOCTL_GET_CHIP_ID:
	{
		u8 devid = 0;
		err = mxc400x_acc_register_read(acc, &devid, WHO_AM_I);
		if (err < 0) {
			printk("%s, error read register WHO_AM_I\n", __func__);
   		mutex_unlock(&acc->lock);
			return -EAGAIN;
		}
		
		if (copy_to_user(argp, &devid, sizeof(devid))) {
			printk("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
	}
            break;
	case MXC400X_ACC_IOCTL_READ_REG:
	{
		u8 content = 0;
		if (copy_from_user(&reg, argp, sizeof(reg))){
		  mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		err = mxc400x_acc_register_read(acc, &content, reg);
		if (err < 0) {
			printk("%s, error read register %x\n", __func__, reg);
		  mutex_unlock(&acc->lock);
			return -EAGAIN;
		}
		if (copy_to_user(argp, &content, sizeof(content))) {
			printk("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
		  mutex_unlock(&acc->lock);
			return -EINVAL;
		}
		
	}
	break;
	case MXC400X_ACC_IOCTL_WRITE_REG:
	{
		if (copy_from_user(&reg, argp, sizeof(reg))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}	//mxc400x_acc_i2c_write(struct mxc400x_acc_data *acc, u8 * buf, int len
		err = mxc400x_acc_i2c_write(acc, &reg, 1);
		if (err < 0) {
			printk("%s, error write register %x\n", __func__, reg);
			mutex_unlock(&acc->lock);
			return -EAGAIN;
		}
	}
	break;
	default:
	 //   printk("%s  %d no cmd error\n", __func__,cmd);
	  mutex_unlock(&acc->lock);
		return -EINVAL;
	}
	
	mutex_unlock(&acc->lock);
	return 0;
}

static const struct file_operations mxc400x_acc_misc_fops = {
		.owner = THIS_MODULE,
		.open = mxc400x_acc_misc_open,
		.unlocked_ioctl = mxc400x_acc_misc_ioctl,
};

static struct miscdevice mxc400x_acc_misc_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = MXC400X_ACC_DEV_NAME,
		.fops = &mxc400x_acc_misc_fops,
};

static void mxc400x_acc_input_work_func(struct work_struct *work)
{
	struct mxc400x_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			   struct mxc400x_acc_data, input_work);

	mutex_lock(&acc->lock);
	err = mxc400x_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		mxc400x_acc_report_values(acc, xyz);

//	schedule_delayed_work(&acc->input_work,
		//	      msecs_to_jiffies(acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}
static int mxc400x_acc_validate_pdata(struct mxc400x_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int mxc400x_acc_input_init(struct mxc400x_acc_data *acc)
{
	int err;
	/* Polling rx data when the interrupt is not used.*/
	if (1 /*acc->irq1 == 0 && acc->irq1 == 0 */ ) {
	//	INIT_DELAYED_WORK(&acc->input_work, mxc400x_acc_input_work_func);
	}

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}
#ifdef MXC400X_ACC_OPEN_ENABLE
	acc->input_dev->open = mxc400x_acc_input_open;
	acc->input_dev->close = mxc400x_acc_input_close;
#endif

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*      next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*      next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, 0, 0);
	/*      next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*      next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

	acc->input_dev->name = "accelerometer";

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
			"unable to register input polled device %s\n",
			acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void mxc400x_acc_input_cleanup(struct mxc400x_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}
#ifdef CONFIG_OF
static struct mxc400x_acc_platform_data *mxc400x_acc_parse_dt(struct device *dev)
{
	struct mxc400x_acc_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct mxc400x_acc_platform_data");
		return NULL;
	}
	ret = of_property_read_u32(np, "poll_interval", &pdata->poll_interval);
	if(ret){
		dev_err(dev, "fail to get poll_interval\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "min_interval", &pdata->min_interval);
	if(ret){
		dev_err(dev, "fail to get min_interval\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "g_range", &pdata->g_range);
	if(ret){
		dev_err(dev, "fail to get g_range\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_x", &pdata->axis_map_x);
	if(ret){
		dev_err(dev, "fail to get axis_map_x\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_y", &pdata->axis_map_y);
	if(ret){
		dev_err(dev, "fail to get axis_map_y\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "axis_map_z", &pdata->axis_map_z);
	if(ret){
		dev_err(dev, "fail to get axis_map_z\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_x", &pdata->negate_x);
	if(ret){
		dev_err(dev, "fail to get negate_x\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_y", &pdata->negate_y);
	if(ret){
		dev_err(dev, "fail to get negate_y\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "negate_z", &pdata->negate_z);
	if(ret){
		dev_err(dev, "fail to get negate_z\n");
		goto fail;
	}
	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif




#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxc400x_early_suspend (struct early_suspend* es);
static void mxc400x_early_resume (struct early_suspend* es);
#endif


static int mxc400x_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct mxc400x_acc_data *acc;
	struct mxc400x_acc_platform_data *pdata = client->dev.platform_data;
	int err = -1;
	int tempvalue;


	 printk("mxc400x_acc_probe \n");
	#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
	if (np && !pdata){
		pdata = mxc400x_acc_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		if(!pdata){
			err = -ENOMEM;
			goto exit_kfree_pdata;
		}
	}
	#endif

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE |
					I2C_FUNC_SMBUS_BYTE_DATA |
					I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}

	
	printk("mxc400x_acc_probe i2c_check_functionality  \n");
	if (!i2c_check_functionality(client->adapter,
						I2C_FUNC_SMBUS_I2C_BLOCK)){
		dev_err(&client->dev, "client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	
	printk("mxc400x_acc_probe tempvalue=0x%x \n",tempvalue);
	if ((tempvalue & 0x003F) == WHOAMI_MXC400X_ACC) {
		printk(KERN_INFO "%s I2C driver registered!\n",
							MXC400X_ACC_DEV_NAME);
	} else {
		acc->client = NULL;
		printk(KERN_INFO "I2C driver not registered!"
				" Device unknown 0x%x\n", tempvalue);
		goto exit_check_functionality_failed;
	}
	/*
	 * OK. From now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */

	acc = kzalloc(sizeof(struct mxc400x_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_alloc_data_failed;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
    mxc400x_i2c_client = client;
	i2c_set_clientdata(client, acc);


	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto exit_kfree_pdata;
	}

	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));

	err = mxc400x_acc_validate_pdata(acc);
	
	printk("mxc400x_acc_probe validate_pdata  err=%d \n",err);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	i2c_set_clientdata(client, acc);


	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err2;
		}
	}
//	cancel_delayed_work_sync(&acc->input_work);
	err = mxc400x_acc_device_power_on(acc);
	
	printk("mxc400x_acc_probe power on  err=%d \n",err);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}
    err = mxc400x_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err2;
	}
    //INIT_DELAYED_WORK(&acc->input_work, mxc400x_acc_input_work_func);
	atomic_set(&acc->enabled, 1);

	mxc400x_acc_misc_data = acc;

	err = misc_register(&mxc400x_acc_misc_device);

	printk("mxc400x_acc_probe err=%d \n",err);
	if (err < 0) {
		dev_err(&client->dev,
				"misc MXC400X_ACC_DEV_NAME register failed\n");
		goto err_input_cleanup;
	}

	mxc400x_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

    acc->on_before_suspend = 0;

 #ifdef CONFIG_HAS_EARLYSUSPEND
    acc->early_suspend.suspend = mxc400x_early_suspend;
    acc->early_suspend.resume  = mxc400x_early_resume;
    acc->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    register_early_suspend(&acc->early_suspend);
#endif

	mutex_unlock(&acc->lock);

	dev_info(&client->dev, "%s: probed\n", MXC400X_ACC_DEV_NAME);


	printk("mxc400x_acc_probe over \n");
	return 0;

err_input_cleanup:
	mxc400x_acc_input_cleanup(acc);

err2:
	if (acc->pdata->exit) acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_mutexunlockfreedata:
	kfree(acc);
 	mutex_unlock(&acc->lock);
    i2c_set_clientdata(client, NULL);
    mxc400x_acc_misc_data = NULL;
exit_alloc_data_failed:
exit_check_functionality_failed:
	printk(KERN_ERR "%s: Driver Init failed\n", MXC400X_ACC_DEV_NAME);
	return err;
}

static int __exit  mxc400x_acc_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct mxc400x_acc_data *acc = i2c_get_clientdata(client);
	
    	misc_deregister(&mxc400x_acc_misc_device);
    	mxc400x_acc_device_power_off(acc);
    	if (acc->pdata->exit)
    		acc->pdata->exit();
    	kfree(acc->pdata);
    	kfree(acc);
    
	return 0;
}

static int mxc400x_acc_resume(struct i2c_client *client)
{
	struct mxc400x_acc_data *acc = i2c_get_clientdata(client);
#ifdef MXC400X_DEBUG
    printk("%s.\n", __func__);
#endif

	if (acc != NULL && acc->on_before_suspend) {
        acc->on_before_suspend = 0;
		return mxc400x_acc_enable(acc);
    }

	return 0;
}

static int mxc400x_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mxc400x_acc_data *acc = i2c_get_clientdata(client);
#ifdef MXC400X_DEBUG
    printk("%s.\n", __func__);
#endif
    if (acc != NULL) {
        if (atomic_read(&acc->enabled)) {
            acc->on_before_suspend = 1;
            return mxc400x_acc_disable(acc);
        }
    }
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND

static void mxc400x_early_suspend (struct early_suspend* es)
{
#ifdef MXC400X_DEBUG
    printk("%s.\n", __func__);
#endif
    mxc400x_acc_suspend(mxc400x_i2c_client,
         (pm_message_t){.event=0});
}

static void mxc400x_early_resume (struct early_suspend* es)
{
#ifdef MXC400X_DEBUG
    printk("%s.\n", __func__);
#endif
    mxc400x_acc_resume(mxc400x_i2c_client);
}

#endif /* CONFIG_HAS_EARLYSUSPEND */



static const struct i2c_device_id mxc400x_acc_id[]
				= { { MXC400X_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, mxc400x_acc_id);

static const struct of_device_id mxc400x_acc_of_match[] = {
	{ .compatible = "Memsic,mxc400x_acc", },
	{}
};
MODULE_DEVICE_TABLE(of, mxc400x_acc_of_match);
static struct i2c_driver mxc400x_acc_driver = {
	.driver = {
			.name = MXC400X_ACC_I2C_NAME,
			.of_match_table = mxc400x_acc_of_match,
		  },
	.probe = mxc400x_acc_probe,
	.remove = __exit_p(mxc400x_acc_remove),
	.resume = mxc400x_acc_resume,
	.suspend = mxc400x_acc_suspend,
	.id_table = mxc400x_acc_id,
};


#ifdef I2C_BUS_NUM_STATIC_ALLOC

int i2c_static_add_device(struct i2c_board_info *info)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int err;

	adapter = i2c_get_adapter(I2C_STATIC_BUS_NUM);
	if (!adapter) {
		pr_err("%s: can't get i2c adapter\n", __FUNCTION__);
		err = -ENODEV;
		goto i2c_err;
	}

	client = i2c_new_device(adapter, info);
	if (!client) {
		pr_err("%s:  can't add i2c device at 0x%x\n",
			__FUNCTION__, (unsigned int)info->addr);
		err = -ENODEV;
		goto i2c_err;
	}

	i2c_put_adapter(adapter);

	return 0;

i2c_err:
	return err;
}

#endif /*I2C_BUS_NUM_STATIC_ALLOC*/

static int __init mxc400x_acc_init(void)
{
        int  ret = 0;

    	printk(KERN_INFO "%s accelerometer driver: init\n",
						MXC400X_ACC_I2C_NAME);
#ifdef I2C_BUS_NUM_STATIC_ALLOC
        ret = i2c_static_add_device(&mxc400x_i2c_boardinfo);
        if (ret < 0) {
            pr_err("%s: add i2c device error %d\n", __FUNCTION__, ret);
            goto init_err;
        }
#endif
        ret = i2c_add_driver(&mxc400x_acc_driver);
        printk(KERN_INFO "%s add driver: %d\n",
						MXC400X_ACC_I2C_NAME,ret);
        return ret;

init_err:
        return ret;
}

static void __exit mxc400x_acc_exit(void)
{
	printk(KERN_INFO "%s accelerometer driver exit\n", MXC400X_ACC_DEV_NAME);

	#ifdef I2C_BUS_NUM_STATIC_ALLOC
        i2c_unregister_device(mxc400x_i2c_client);
	#endif

	i2c_del_driver(&mxc400x_acc_driver);
	return;
}

module_init(mxc400x_acc_init);
module_exit(mxc400x_acc_exit);


MODULE_AUTHOR("Robbie Cao<hjcao@memsic.com>");
MODULE_DESCRIPTION("MEMSIC MXC400X (DTOS) Accelerometer Sensor Driver");
MODULE_LICENSE("GPL");
