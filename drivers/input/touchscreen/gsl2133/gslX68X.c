/* drivers/input/touchscreen/gslX68X.h
 * 
 * 2010 - 2013 SLIEAD Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the SLIEAD's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 */
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/byteorder/generic.h>

#include <linux/timer.h>

#include <linux/jiffies.h>

#include <linux/irq.h>

#include <linux/platform_device.h>

#include <linux/earlysuspend.h>
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/wakelock.h>

#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/device.h>



#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include "gslX68X.h"

#ifdef GSL_REPORT_POINT_SLOT
    #include <linux/input/mt.h>
#endif

static struct mutex gsl_i2c_lock;

#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk("[tp-gsl][%s]"fmt,__func__, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)   //
#endif

//#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

static struct wake_lock gsl_wake_lock;

/*define golbal variable*/
static struct gsl_ts_data *ddata=NULL;




#ifdef TOUCH_VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{


#if  0//def CONFIG_OF
	struct gsl_ts_data *data = i2c_get_clientdata(ddata);
	struct gsl_ts_platform_data *pdata = data->platform_data;
	return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
	,__stringify(EV_KEY), __stringify(KEY_MENU),pdata ->virtualkeys[0],pdata ->virtualkeys[1],pdata ->virtualkeys[2],pdata ->virtualkeys[3]
	,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),pdata ->virtualkeys[4],pdata ->virtualkeys[5],pdata ->virtualkeys[6],pdata ->virtualkeys[7]
	,__stringify(EV_KEY), __stringify(KEY_BACK),pdata ->virtualkeys[8],pdata ->virtualkeys[9],pdata ->virtualkeys[10],pdata ->virtualkeys[11]);


#else
	return sprintf(buf,
         __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":100:1020:80:65"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":280:1020:80:65"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) 	 ":470:1020:80:65"
	 "\n");

#endif
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.gsl2133_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void gsl_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;

   printk("gsl_ts_virtual_keys_init \n");
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        printk(" gsl_ts_virtual_keys_init failed to create board_properties\n");
}

#endif




static int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{

	int err = 0;
	u8 temp = reg;
	mutex_lock(&gsl_i2c_lock);
	if(temp < 0x80)
	{
		if(temp==0x7c){
			temp =0x78;
			i2c_master_send(client,&temp,1);	
			err = i2c_master_recv(client,&buf[0],4);
			temp = 0x7c;
		}else{
			i2c_master_send(client,&temp,1);	
			err = i2c_master_recv(client,&buf[0],4);
		}
	}
	i2c_master_send(client,&temp,1);
	err = i2c_master_recv(client,&buf[0],num);
	mutex_unlock(&gsl_i2c_lock);
	return err;
}

static int gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];
	int err;
	u8 tmp_buf[num+1];
	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);
	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	//xfer_msg[0].timing = 400;

	mutex_lock(&gsl_i2c_lock);
	err= i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);
	return err;	
//	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}
#if 0
#define DMA_TRANS_LEN 0x20
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[DMA_TRANS_LEN*4] = {0};
	u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		if (0xf0 == GSL_DOWNLOAD_DATA[source_line].offset)
		{
			memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);	
			gsl_write_interface(client, 0xf0, buf, 4);
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
	    			addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;

			memcpy((buf+send_flag*4 -4),&GSL_DOWNLOAD_DATA[source_line].val,4);	

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
			{
	    		gsl_write_interface(client, addr, buf, DMA_TRANS_LEN * 4);
				send_flag = 0;
			}

			send_flag++;
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#else 
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[4] = {0};
	//u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

//	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
	    addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;
		memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);
	    gsl_write_interface(client, addr, buf, 4);	
	}
//	printk("=============gsl_load_fw end==============\n");
}
#endif
static void gsl_start_core(struct i2c_client *client)
{
	//u8 tmp = 0x00;
	u8 buf[4] = {0};
#if 0
	buf[0]=0;
	buf[1]=0x10;
	buf[2]=0xfe;
	buf[3]=0x1;
	gsl_write_interface(client,0xf0,buf,4);
	buf[0]=0xf;
	buf[1]=0;
	buf[2]=0;
	buf[3]=0;
	gsl_write_interface(client,0x4,buf,4);
	msleep(20);
#endif
	buf[0]=0;
	gsl_write_interface(client,0xe0,buf,4);
#ifdef GSL_ALG_ID
	{
		gsl_DataInit(gsl_config_data_id);
	}
#endif	
}

static void gsl_reset_core(struct i2c_client *client)
{
	u8 buf[4] = {0x00};
	
	buf[0] = 0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(5);

	buf[0] = 0x04;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	
	buf[0] = 0;
	gsl_write_interface(client,0xbc,buf,4);
	msleep(5);
}

static void gsl_clear_reg(struct i2c_client *client)
{
	u8 buf[4]={0};
	//clear reg
	buf[0]=0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	buf[0]=0x3;
	gsl_write_interface(client,0x80,buf,4);
	msleep(5);
	buf[0]=0x4;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	buf[0]=0x0;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	//clear reg

}
static int gsl_compatible_id(struct i2c_client *client)
{
	u8 buf[4];
	int i,err;
	for(i=0;i<5;i++)
	{
		err = gsl_read_interface(client,0xfc,buf,4);
		printk("[tp-gsl] 0xfc = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],
			buf[1],buf[0]);
		if(!(err<0))
		{
			err = 1;		
			break;	
		}
	}
	return err;	
}
static void gsl_hw_init(void)
{
	//add power
	//GSL_POWER_ON();	
	//
	//gpio_request(GSL_IRQ_GPIO_NUM,GSL_IRQ_NAME);	
	//gpio_request(GSL_RST_GPIO_NUM,GSL_RST_NAME);



	gpio_request(sprd_3rdparty_gpio_tp_irq, "ts_irq_pin");

	gpio_request(sprd_3rdparty_gpio_tp_rst, "ts_rst_pin");
	
	gpio_direction_output(sprd_3rdparty_gpio_tp_rst,1);	
	gpio_direction_input(sprd_3rdparty_gpio_tp_irq);

	msleep(5);
	gpio_set_value(sprd_3rdparty_gpio_tp_rst, 0);
	msleep(10); 	
	gpio_set_value(sprd_3rdparty_gpio_tp_rst, 1);
	msleep(20); 
	//	
}
static void gsl_sw_init(struct i2c_client *client)
{
	int temp;
	if(1==ddata->gsl_sw_flag)
		return;
	ddata->gsl_sw_flag = 1;
	
	gpio_set_value(sprd_3rdparty_gpio_tp_rst, 0);
	msleep(20);
	gpio_set_value(sprd_3rdparty_gpio_tp_rst, 1);
	msleep(20);	

	gsl_clear_reg(client);
	gsl_reset_core(client);
	{
		temp = ARRAY_SIZE(GSLX68X_FW);
		gsl_load_fw(client,GSLX68X_FW,temp);	
	}
	gsl_start_core(client);

	ddata->gsl_sw_flag = 0;
}

static void check_mem_data(struct i2c_client *client)
{

	u8 read_buf[4]  = {0};
	msleep(30);
	gsl_read_interface(client,0xb0,read_buf,4);
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a 
		|| read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		printk(" gls check_mem_data   0xb4 ={0x%02x%02x%02x%02x}\n",
			read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		gsl_sw_init(client);
	}
}
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}
//
#ifdef TPD_PROC_DEBUG
static int gsl_config_read_proc(struct seq_file *m,void *v)
//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	//char *ptr = page;
	char temp_data[5] = {0};
	unsigned int tmp=0;
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_ALG_ID
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_ALG_ID 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<ARRAY_SIZE(gsl_config_data_id))
				seq_printf(m,"%d\n",gsl_config_data_id[tmp]); 
#endif
		}
		else 
		{
			gsl_write_interface(ddata->client,0xf0,&gsl_data_proc[4],4);
			gsl_read_interface(ddata->client,gsl_data_proc[0],temp_data,4);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
	//*eof = 1;
	return 0;
}
static int gsl_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%ld]\n", CONFIG_LEN, count);
        	return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}	
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
	
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
		gsl_proc_flag = 1;
		gsl_reset_core(ddata->client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		gsl_reset_core(ddata->client);
		gsl_start_core(ddata->client);
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		gsl_write_interface(ddata->client,buf[4],buf,4);
	}
	
#ifdef GSL_ALG_ID
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<ARRAY_SIZE(gsl_config_data_id))
		{
			gsl_config_data_id[tmp1] = tmp;
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
static void *gsl_seq_start(struct seq_file *seq,loff_t *pos)
{
	char *buf;
	print_info("\n");
	if(*pos>0){
		return NULL;
	}
	buf = kzalloc(16,GFP_KERNEL);
	if(buf==NULL)
		return NULL;
	return buf; 
}
static void *gsl_seq_next(struct seq_file *seq, void *v,loff_t *pos)
{
	print_info("\n");
	return NULL;
}
static void gsl_seq_stop(struct seq_file *seq,void *v)
{
	print_info("\n");
	if(v!=NULL)
		kfree(v);
	return;
}
static const struct seq_operations gsl_server_list_ops = {
	.start	= gsl_seq_start,
	.next	= gsl_seq_next,
	.stop	= gsl_seq_stop,
	.show	= gsl_config_read_proc,
};

static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	int ret;
	print_info("\n");
	ret = seq_open(file,&gsl_server_list_ops);
	if(ret<0){
		return ret;
	}
	return 0;
}
static const struct file_operations gsl_seq_fops = {
	.open 		= gsl_server_list_open,
	.read		= seq_read,
	.release	= seq_release,
	.write		= gsl_config_write_proc,
	.owner		= THIS_MODULE,
};
#endif

#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{	
	u8 buf[4] = {0};
	u32 tmp;
	int i,flag=0;
	static int timer_count;
	if(1==ddata->gsl_halt_flag){
		return;
	}
	gsl_read_interface(ddata->client, 0xb4, buf, 4);
	tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);

	print_info("[pre] 0xb4 = %x \n",ddata->gsl_timer_data);
	print_info("[cur] 0xb4 = %x \n",tmp);
	print_info("gsl_timer_flag=%d\n",ddata->gsl_timer_flag);
	if(0 == ddata->gsl_timer_flag)
	{
		if(tmp== ddata->gsl_timer_data)
		{
			ddata->gsl_timer_flag = 1;
			if(0==ddata->gsl_halt_flag)
			{
				queue_delayed_work(ddata->timer_wq, &ddata->timer_work, 25);
			}
		}
		else
		{
			for(i=0;i<5;i++){
				gsl_read_interface(ddata->client,0xbc,buf,4);
				if(buf[0]==0&&buf[1]==0&&buf[2]==0&&buf[3]==0)
				{
					flag = 1;
					break;
				}
				flag =0;
			}
			if(flag == 0){
				gsl_reset_core(ddata->client);
				gsl_start_core(ddata->client);
			}
			ddata->gsl_timer_flag = 0;
			timer_count = 0;
			if(0 == ddata->gsl_halt_flag)
			{
				queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
	}
	else if(1==ddata->gsl_timer_flag){
		if(tmp==ddata->gsl_timer_data)
		{
			if(0==ddata->gsl_halt_flag)
			{
				timer_count++;
				ddata->gsl_timer_flag = 2;
				gsl_sw_init(ddata->client);
				ddata->gsl_timer_flag = 1;
			}
			if(0 == ddata->gsl_halt_flag && timer_count < 20)
			{
				queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
		else{
			timer_count = 0;
			if(0 == ddata->gsl_halt_flag && timer_count < 20)
			{
				queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
		ddata->gsl_timer_flag = 0;
	}
	ddata->gsl_timer_data = tmp;
}
#endif

//
#if GSL_HAVE_TOUCH_KEY
static int gsl_report_key(struct input_dev *idev,int x,int y)
{
	int i;
	for(i=0;i<GSL_KEY_NUM;i++)
	{
		if(x > gsl_key_data[i].x_min &&
			x < gsl_key_data[i].x_max &&
			y > gsl_key_data[i].y_min &&
			y < gsl_key_data[i].y_max)
		{
			ddata->gsl_key_state = i+1;
			input_report_key(idev,gsl_key_data[i].key,1);
			input_sync(idev);
			return 1;
		}
	}
	return 0;
}
#endif
static void gsl_report_point(struct input_dev *idev, struct gsl_touch_info *cinfo)
{
	int i; 
	u32 gsl_point_state = 0;
	u32 temp=0;
	if(cinfo->finger_num>0 && cinfo->finger_num<6)
	{
		ddata->gsl_up_flag = 0;
		gsl_point_state = 0;
	#if GSL_HAVE_TOUCH_KEY
		if(1==cinfo->finger_num)
		{
			if(cinfo->x[0] > GSL_MAX_X || cinfo->y[0] > GSL_MAX_Y)
			{
				gsl_report_key(idev,cinfo->x[0],cinfo->y[0]);
				return;		
			}
		}
	#endif
		for(i=0;i<cinfo->finger_num;i++)
		{
			gsl_point_state |= (0x1<<cinfo->id[i]);	
			print_info("id = %d, x = %d, y = %d \n",cinfo->id[i], 
				cinfo->x[i],cinfo->y[i]);
		#ifdef GSL_REPORT_POINT_SLOT
			input_mt_slot(idev, cinfo->id[i] - 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);


		
		#else 
			input_report_key(idev, BTN_TOUCH, 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(idev);		
		#endif
		}
	}
	else if(cinfo->finger_num == 0)
	{
		gsl_point_state = 0;
	//	ddata->gsl_point_state = 0;
		if(1 == ddata->gsl_up_flag)
		{
			return;
		}
		ddata->gsl_up_flag = 1;
	#if GSL_HAVE_TOUCH_KEY
		if(ddata->gsl_key_state > 0)
		{
			if(ddata->gsl_key_state < GSL_KEY_NUM+1)
			{
				input_report_key(idev,gsl_key_data[ddata->gsl_key_state - 1].key,0);
				input_sync(idev);
			}
		}
	#endif
	#ifndef GSL_REPORT_POINT_SLOT
		input_report_key(idev, BTN_TOUCH, 0);
		input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(idev);
	#endif
	}

	temp = gsl_point_state & ddata->gsl_point_state;
	temp = (~temp) & ddata->gsl_point_state;
#ifdef GSL_REPORT_POINT_SLOT
	for(i=1;i<6;i++)
	{
		if(temp & (0x1<<i))
		{
			input_mt_slot(idev, i-1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(idev, MT_TOOL_FINGER, false);
		}
	}
#endif	
	ddata->gsl_point_state = gsl_point_state;
	input_sync(idev);
}


static void gsl_report_work(struct work_struct *work)
{
	int rc,tmp;
	u8 buf[44] = {0};
	int tmp1=0;
	struct gsl_touch_info *cinfo = ddata->cinfo;
	struct i2c_client *client = ddata->client;
	struct input_dev *idev = ddata->idev;
	
	if(1 == ddata->gsl_sw_flag)
		goto schedule;

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		goto schedule;
	}
#endif
#ifdef GSL_TIMER 
	if(2==ddata->gsl_timer_flag){
		goto schedule;
	}
#endif

	/* read data from DATA_REG */
	rc = gsl_read_interface(client, 0x80, buf, 44);
	if (rc < 0) 
	{
		dev_err(&client->dev, "[gsl] I2C read failed\n");
		goto schedule;
	}

	if (buf[0] == 0xff) {
		goto schedule;
	}

	cinfo->finger_num = buf[0];
	for(tmp=0;tmp<(cinfo->finger_num>10 ? 10:cinfo->finger_num);tmp++)
	{
		cinfo->y[tmp] = (buf[tmp*4+4] | ((buf[tmp*4+5])<<8));
		cinfo->x[tmp] = (buf[tmp*4+6] | ((buf[tmp*4+7] & 0x0f)<<8));
		cinfo->id[tmp] = buf[tmp*4+7] >> 4;
		print_info("tp-gsl  x = %d y = %d \n",cinfo->x[tmp],cinfo->y[tmp]);
	}
	
	print_info("111 finger_num= %d\n",cinfo->finger_num);
#ifdef GSL_ALG_ID
	cinfo->finger_num = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
	gsl_alg_id_main(cinfo);
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;
		buf[1]=0;
		buf[2]=0;
		buf[3]=0;
		gsl_write_interface(client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",
			tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_write_interface(client,0x8,buf,4);
	}
#endif


	print_info("222 finger_num= %d\n",cinfo->finger_num);
	gsl_report_point(idev,cinfo);
	
schedule:
	enable_irq(client->irq);

}

static int gsl_request_input_dev(struct gsl_ts_data *ddata)

{
	struct input_dev *input_dev;
	int err;
#if GSL_HAVE_TOUCH_KEY
	int i;
#endif
	/*allocate input device*/
	print_info("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_allocate_input_device_fail;
	}
	ddata->idev = input_dev;
	/*set input parameter*/	
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
#ifdef GSL_REPORT_POINT_SLOT
	__set_bit(EV_REP, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,input_dev->propbit);
	input_mt_init_slots(input_dev,5);
#else 
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0,5,0,0);
#endif
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
#ifdef TOUCH_VIRTUAL_KEYS
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOME,  input_dev->keybit);    
#endif
#if GSL_HAVE_TOUCH_KEY
	for(i=0;i<GSL_KEY_NUM;i++)
	{
		__set_bit(gsl_key_data[i].key, input_dev->keybit);
	}
#endif
	
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, GSL_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, GSL_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	input_dev->name = GSL_TS_NAME;		//dev_name(&client->dev)

	/*register input device*/
	err = input_register_device(input_dev);
	if (err) {
		goto err_register_input_device_fail;
	}
	return 0;
err_register_input_device_fail:
	input_free_device(input_dev);			
err_allocate_input_device_fail:
	return err;
}

static irqreturn_t gsl_ts_interrupt(int irq, void *dev_id)
{
	struct i2c_client *client = ddata->client;
	print_info("gslX68X_ts_interrupt\n");
	
	disable_irq_nosync(client->irq);
	if (!work_pending(&ddata->work)) {
		queue_work(ddata->wq, &ddata->work);
	}
	
	return IRQ_HANDLED;
}

static void gsl_early_suspend(struct early_suspend *handler)
{
	u32 tmp;
	struct i2c_client *client = ddata->client;
	print_info("==gslX68X_ts_suspend=\n");


	
	//version info
	printk("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);

	if(1==ddata->gsl_sw_flag)
		return;

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();	
	printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif
	//version info
	ddata->gsl_halt_flag = 1;
#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&ddata->timer_work);
	if(2==ddata->gsl_timer_flag){
		return;
	}
#endif
	disable_irq_nosync(client->irq);
	gpio_set_value(sprd_3rdparty_gpio_tp_rst, 0);


}

static void gsl_early_resume(struct early_suspend *handler)
{	

	
	struct i2c_client *client = ddata->client;
	print_info("==gslX68X_ts_resume=\n");
	if(1==ddata->gsl_sw_flag){
		ddata->gsl_halt_flag = 0;
		return;
	}

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		ddata->gsl_halt_flag = 0;
		return;
	}
#endif

#ifdef GSL_TIMER
	if(2==ddata->gsl_timer_flag)
	{
		ddata->gsl_halt_flag=0;
		enable_irq(client->irq);
		return;
	}
#endif
	gpio_set_value(sprd_3rdparty_gpio_tp_rst, 1);
	msleep(20);
	gsl_reset_core(client);
	gsl_start_core(client);
	msleep(20);
	check_mem_data(client);
	enable_irq(client->irq);
#ifdef GSL_TIMER
	queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
	ddata->gsl_timer_flag = 0;
#endif
	ddata->gsl_halt_flag = 0;




}


#ifdef CONFIG_OF
static struct gsl_ts_platform_data *gls2133_ts_parse_dt(struct device *dev)
{
	struct gsl_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
	//	dev_err(dev, "Could not allocate struct ft5x0x_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
	//	dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		//dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		//dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", &pdata->virtualkeys,12);
	if(ret){
		//dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		//dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		//dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

static int gsl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	 struct gsl_ts_platform_data *pdata = client->dev.platform_data;

	printk("gsl_ts_probe  start \n");
	
	#ifdef CONFIG_OF
				struct device_node *np = client->dev.of_node;
				if (np && !pdata){
					pdata = gls2133_ts_parse_dt(&client->dev);
					if(pdata){
						client->dev.platform_data = pdata;
					}
					else{
						err = -ENOMEM;
						return -1;
					}
				}
	#endif
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
        printk("%s: I2c doesn't work\n", __func__);
		goto exit_check_functionality_failed;
	}

	print_info("==kzalloc=");
	ddata = kzalloc(sizeof(struct gsl_ts_data), GFP_KERNEL);
	if (ddata==NULL)
	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	
	ddata->gsl_halt_flag = 0;
	ddata->gsl_sw_flag = 0;
	ddata->gsl_up_flag = 0;
	ddata->gsl_point_state = 0;
#if GSL_HAVE_TOUCH_KEY
	ddata->gsl_key_state = 0;
#endif	
	ddata->cinfo = kzalloc(sizeof(struct gsl_touch_info),GFP_KERNEL);
	if(ddata->cinfo == NULL)
	{
		err = -ENOMEM;
		goto exit_alloc_cinfo_failed;
	}
	mutex_init(&gsl_i2c_lock);

	ddata->client = client;
	i2c_set_clientdata(client, ddata);
	print_info("I2C addr=%x\n", client->addr);	
	gsl_hw_init();
	err = gsl_compatible_id(client);
	if(err < 0)
	{
		goto exit_i2c_transfer_fail; 
	}
	/*request input system*/	
	err = gsl_request_input_dev(ddata);
	if(err < 0)
	{
		goto exit_i2c_transfer_fail;	
	}
	
	/*register early suspend*/
	print_info("==register_early_suspend =\n");
	ddata->pm.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ddata->pm.suspend = gsl_early_suspend;
	ddata->pm.resume	= gsl_early_resume;
	register_early_suspend(&ddata->pm);

	/*init work queue*/
	wake_lock_init(&gsl_wake_lock, WAKE_LOCK_SUSPEND, "gsl_wake_lock");
	INIT_WORK(&ddata->work, gsl_report_work);
	ddata->wq = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ddata->wq) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	/*request irq */
	client->irq = GSL_IRQ_NUM;
	print_info("%s: ==request_irq=\n",__func__);
	print_info("%s IRQ number is %d\n", client->name, client->irq);
	err = request_irq(client->irq, gsl_ts_interrupt, IRQF_TRIGGER_RISING| IRQF_NO_SUSPEND, client->name, ddata);
	if (err < 0) {
		dev_err(&client->dev, "gslX68X_probe: request irq failed\n");
		printk("gsl_ts_probe request_irq %d \n",err);
		goto exit_irq_request_failed;
	}
	disable_irq_nosync(client->irq);
	/*gsl of software init*/
	printk("gsl_ts_probe  gsl_sw_init\n");
	gsl_sw_init(client);
	msleep(20);
	printk("gsl_ts_probe  check mem data\n");
	check_mem_data(client);
	printk("gsl_ts_probe  create proc\n");
#ifdef TPD_PROC_DEBUG
	proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
/*
	gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
	if (gsl_config_proc == NULL)
	{
		print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
	}
	else
	{
		gsl_config_proc->read_proc = gsl_config_read_proc;
		gsl_config_proc->write_proc = gsl_config_write_proc;
	}
*/
	gsl_proc_flag = 0;
#endif

#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&ddata->timer_work, gsl_timer_check_func);
	ddata->timer_wq = create_workqueue("gsl_timer_wq");
	queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
#endif

#ifdef TOUCH_VIRTUAL_KEYS
	gsl_ts_virtual_keys_init();
#endif
	enable_irq(client->irq);
	print_info("%s: ==probe over =\n",__func__);
	return 0;


exit_irq_request_failed:
	cancel_work_sync(&ddata->work);
	destroy_workqueue(ddata->wq);
exit_create_singlethread:
	unregister_early_suspend(&ddata->pm);
	input_unregister_device(ddata->idev);
	input_free_device(ddata->idev);
exit_i2c_transfer_fail:
	gpio_free(sprd_3rdparty_gpio_tp_rst);
	gpio_free(sprd_3rdparty_gpio_tp_irq);
	i2c_set_clientdata(client, NULL);
	kfree(ddata->cinfo);
exit_alloc_cinfo_failed:
	kfree(ddata);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int gsl_ts_remove(struct i2c_client *client)
{

	print_info("==gslX68X_ts_remove=\n");
	
#ifdef TPD_PROC_DEBUG
	if(gsl_config_proc!=NULL)
		remove_proc_entry(GSL_CONFIG_PROC_FILE, NULL);
#endif

#ifdef GSL_TIMER
	cancel_delayed_work_sync(&ddata->timer_work);
	destroy_workqueue(ddata->timer_wq);
#endif
	
	unregister_early_suspend(&ddata->pm);
	free_irq(client->irq,ddata);
	input_unregister_device(ddata->idev);
	input_free_device(ddata->idev);
	gpio_free(sprd_3rdparty_gpio_tp_rst);
	gpio_free(sprd_3rdparty_gpio_tp_irq);
	
	cancel_work_sync(&ddata->work);
	destroy_workqueue(ddata->wq);
	i2c_set_clientdata(client, NULL);
	//sprd_free_gpio_irq(client->irq);
	kfree(ddata->cinfo);
	kfree(ddata);

	return 0;
}




static const struct i2c_device_id gsl_ts_id[] = {
	{ GSL_TS_NAME, 0 },
	{ }
};
static const struct of_device_id gsl2133_of_match[] = {
       { .compatible = "gsl2133,gsl2133_ts", },
       { }
};


MODULE_DEVICE_TABLE(i2c, gsl_ts_id);

MODULE_DEVICE_TABLE(of, gsl2133_of_match);

static struct i2c_driver gsl_ts_driver = {

		.probe		= gsl_ts_probe,
	.remove		= gsl_ts_remove,
	.id_table	= gsl_ts_id,

	.driver	= {
		.name	= GSL_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = gsl2133_of_match,
	},

		
};

#if I2C_BOARD_INFO_METHOD
static int __init gsl_ts_init(void)
{
	print_info();	
	i2c_add_driver(&gsl_ts_driver);
	return 0;
}

static void __exit gsl_ts_exit(void)
{
	print_info();
	i2c_del_driver(&gsl_ts_driver);	
}
#else
static struct i2c_board_info gsl_i2c_info = {
	.type = GSL_TS_NAME,
	.addr = GSL_TS_ADDR,
}; 
static int __init gsl_ts_init(void)
{
	int ret;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	adapter = i2c_get_adapter(GSL_ADAPTER_INDEX);
	if(!adapter)
	{
		return -1;
	}
	client = i2c_new_device(adapter,&gsl_i2c_info);
	if(!client)
	{
		ret = -1;
		goto err_put_adapter;
	}
	i2c_put_adapter(adapter);

	ret = i2c_add_driver(&gsl_ts_driver);
	if(ret < 0 )
	{
		goto err_add_driver;
	}
	return 0;

err_add_driver:
	kfree(client);
err_put_adapter:
	kfree(adapter);
	return ret;
}
static void __exit gsl_ts_exit(void)
{
	print_info();
	i2c_unregister_device(ddata->client);
	i2c_del_driver(&gsl_ts_driver);	
}
#endif
module_init(gsl_ts_init);
module_exit(gsl_ts_exit);

MODULE_AUTHOR("sileadinc");
MODULE_DESCRIPTION("GSL Series Driver");
MODULE_LICENSE("GPL");

