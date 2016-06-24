/* drivers/video/sc8825/lcd_otm8019a_mipi.c
 *
 * Support for otm8019a mipi LCD device
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include "../sprdfb_panel.h"

//#define LCD_Delay(ms)  uDelay(ms*1000)

#define MAX_DATA   48

typedef struct LCM_Init_Code_tag {
	unsigned int tag;
	unsigned char data[MAX_DATA];
}LCM_Init_Code;

#define LCM_TAG_SHIFT 24
#define LCM_TAG_MASK  ((1 << 24) -1)
#define LCM_SEND(len) ((1 << LCM_TAG_SHIFT)| len)
#define LCM_SLEEP(ms) ((2 << LCM_TAG_SHIFT)| ms)
//#define ARRAY_SIZE(array) ( sizeof(array) / sizeof(array[0]))

#define LCM_TAG_SEND  (1<< 0)
#define LCM_TAG_SLEEP (1 << 1)

static LCM_Init_Code init_data[] = {


{LCM_SEND(2), {0x00,0x00}}, 
{LCM_SEND(6), {4, 0, 0xff,0x80,0x19,0x01}}, 
{LCM_SEND(2), {0x00,0x80}},
{LCM_SEND(5), {3, 0, 0xff,0x80,0x19}},
{LCM_SEND(2), {0x00,0x8A}},
{LCM_SEND(2), {0xC4,0x40}},
{LCM_SEND(2), {0x00,0xA6}},
{LCM_SEND(5), {3, 0, 0xB3,0x20,0x01}},
{LCM_SEND(2), {0x00,0x90}},
{LCM_SEND(9), {7, 0, 0xC0,0x00,0x15,0x00,0x00,0x00,0x03}},
{LCM_SEND(2), {0x00,0x8A}},
{LCM_SEND(2), {0x00,0xb4}},
{LCM_SEND(2), {0xc0,0x20}},
{LCM_SEND(2), {0x00,0x81}},
{LCM_SEND(2), {0xc1,0x33}},



{LCM_SEND(2), {0x00,0x81}},
{LCM_SEND(2), {0xc4,0x81}},

{LCM_SEND(2), {0x00,0x87}},
{LCM_SEND(2), {0xc4,0x00}},

{LCM_SEND(2), {0x00,0x89}},
{LCM_SEND(2), {0xc4,0x00}},

{LCM_SEND(2), {0x00,0x82}},
{LCM_SEND(2), {0xc5,0xb0}},


{LCM_SEND(2), {0x00,0x90}},
{LCM_SEND(10), {8, 0, 0xC5,0x4e,0x34,0x06,0x91,0x33,0x34,0x23}},


{LCM_SEND(2), {0x00,0xb1}},
{LCM_SEND(2), {0xc5,0xA8}},
{LCM_SEND(2), {0x00,0x00}},
{LCM_SEND(5), {3, 0, 0xd8,0x7f,0x7f}},

{LCM_SEND(2), {0x00,0x00}},
{LCM_SEND(2), {0xd9,0x4c}},
{LCM_SEND(2), {0x00,0x80}},

{LCM_SEND(15), {13, 0, 0xCE,0x86,0x01,0x00,0x85,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},


{LCM_SEND(2), {0x00,0xA0}},
{LCM_SEND(15), {13, 0, 0xCE,0x18,0x05,0x83,0x39,0x00,0x00,0x00,0x18,0x04,0x83,0x3A,0x00,0x00,0x00}},


{LCM_SEND(2), {0x00,0xB0}},

{LCM_SEND(17), {15, 0, 0xCE,0x18,0x03,0x83,0x3B,0x86,0x00,0x00,0x18,0x02,0x83,0x3C,0x88,0x00,0x00}},

{LCM_SEND(2), {0x00,0xC0}},

{LCM_SEND(13), {11, 0, 0xCF,0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x02,0x00,0x000}},

{LCM_SEND(2), {0x00,0xD0}},
{LCM_SEND(2), {0xCF,0x00}},

{LCM_SEND(2), {0x00,0xC0}},

{LCM_SEND(18), {16, 0,0xCB,0x00,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},


{LCM_SEND(2), {0x00,0xD0}},

{LCM_SEND(2), {0xCB,0x00}},

{LCM_SEND(2), {0x00,0xD5}},

{LCM_SEND(13), {11, 0, 0xCB,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{LCM_SEND(2), {0x00,0xE0}},
{LCM_SEND(9), {7, 0, 0xCB,0x01,0x01,0x01,0x01,0x01,0x00}},

{LCM_SEND(2), {0x00,0x80}},
{LCM_SEND(13), {11, 0,0xCC,0x00,0x26,0x09,0x0b,0x01,0x25,0x00,0x00,0x00,0x00}},

{LCM_SEND(2), {0x00,0x90}},
{LCM_SEND(9), {7, 0, 0xCC,0x00,0x00,0x00,0x00,0x00,0x00}},

{LCM_SEND(2), {0x00,0x9A}},
{LCM_SEND(8), {6, 0, 0xCC,0x00,0x00,0x00,0x00,0x00}},


{LCM_SEND(2), {0x00,0xA0}},
{LCM_SEND(14), {12, 0, 0xCC,0x00,0x00,0x00,0x00,0x00,0x25,0x02,0x0c,0x0a,0x26,0x00}},



{LCM_SEND(2), {0x00,0xB0}},
{LCM_SEND(13), {11, 0, 0xCC,0x00,0x25,0xc0,0x0a,0x02,0x26,0x00,0x00,0x00,0x00}},


{LCM_SEND(2), {0x00,0xc0}},
{LCM_SEND(9), {7, 0, 0xCC,0x00,0x00,0x00,0x00,0x00,0x00}},


{LCM_SEND(2), {0x00,0xcA}},
{LCM_SEND(8), {6, 0, 0xCC,0x00,0x00,0x00,0x00,0x00}},


{LCM_SEND(2), {0x00,0xD0}},
{LCM_SEND(14), {12, 0, 0xCC,0x00,0x00,0x00,0x00,0x00,0x26,0x01,0x09,0x0b,0x25,0x00}},

{LCM_SEND(2), {0x00,0x00}},
{LCM_SEND(23), {21, 0, 0xE1,0x00,0x07,0x10,0x22,0x40,0x56,0x66,0x99,0x88,0x9f,0x6a,0x5a,0x74,0x61,0x68,0x61,0x5b,0x4e,0x44,0x00}},



{LCM_SEND(2), {0x00,0x00}},
{LCM_SEND(23), {21, 0, 0xE2,0x00,0x07,0x10,0x22,0x40,0x56,0x66,0x99,0x88,0x9f,0x6a,0x5a,0x74,0x61,0x68,0x61,0x5b,0x4e,0x44,0x00}},


                                   
{LCM_SEND(2), {0x00,0x80}},									   
{LCM_SEND(2), {0xc4,0x30}},

{LCM_SEND(2), {0x00,0x98}},									   
{LCM_SEND(2), {0xc0,0x00}},


{LCM_SEND(2), {0x00,0xa9}}, 								   
{LCM_SEND(2), {0xc0,0x0A}},

{LCM_SEND(2), {0x00,0xb0}}, 								   
{LCM_SEND(6), {4, 0, 0xC1,0x20,0x00,0x00}},

{LCM_SEND(2), {0x00,0xe1}},
{LCM_SEND(5), {3, 0, 0xC0,0x40,0x30}},

{LCM_SEND(2), {0x00,0x80}},
{LCM_SEND(5), {3, 0, 0xC1,0x03,0x33}},

{LCM_SEND(2), {0x00,0xA0}},
{LCM_SEND(2), {0xc1,0xe8}},

{LCM_SEND(2), {0x00,0x90}},
{LCM_SEND(2), {0xb6,0xb4}},

{LCM_SEND(2), {0x35,0x00}},


 {LCM_SLEEP(10),},
{LCM_SEND(2), {0x00,0x00}},
{LCM_SEND(2), {0xfb,0x01}},
/////////////////////////////////////////////////

{LCM_SEND(2), {0x00,0x00}},
{LCM_SEND(6), {4, 0, 0xff,0xff,0xff,0xff}},

{LCM_SEND(2), {0x11,0x00}},
{LCM_SLEEP(120),},
{LCM_SEND(2), {0x29,0x00}},



};

//static LCM_Init_Code disp_on =  {LCM_SEND(1), {0x29}};

static LCM_Init_Code sleep_in[] =  {
{LCM_SEND(1), {0x28}},
{LCM_SLEEP(10)},
{LCM_SEND(1), {0x10}},
{LCM_SLEEP(120)},
};

static LCM_Init_Code sleep_out[] =  {
{LCM_SEND(1), {0x11}},
{LCM_SLEEP(120)},
{LCM_SEND(1), {0x29}},
{LCM_SLEEP(20)},
};

static int32_t otm8019a_mipi_init(struct panel_spec *self)
{
	int32_t i = 0;
	LCM_Init_Code *init = init_data;
	unsigned int tag;

	mipi_set_cmd_mode_t mipi_set_cmd_mode = self->info.mipi->ops->mipi_set_cmd_mode;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk(KERN_DEBUG "otm8019a_mipi_init\n");

	mipi_set_cmd_mode();

	for(i = 0; i < ARRAY_SIZE(init_data); i++){
		tag = (init->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(init->data, (init->tag & LCM_TAG_MASK));
			udelay(10);
		}else if(tag & LCM_TAG_SLEEP){
			msleep((init->tag & LCM_TAG_MASK));
		}
		init++;
	}
	return 0;
}

static uint32_t otm8019a_readid(struct panel_spec *self)
{
	/*Jessica TODO: need read id*/
	

	
	return 0x8019;
}


static int32_t  otm8019a_check_esd(struct panel_spec *self)
{

	return 1;

}

static int32_t otm8019a_enter_sleep(struct panel_spec *self, uint8_t is_sleep)
{
	int32_t i = 0;
	LCM_Init_Code *sleep_in_out = NULL;
	unsigned int tag;
	int32_t size = 0;

	mipi_dcs_write_t mipi_dcs_write = self->info.mipi->ops->mipi_dcs_write;
	mipi_gen_write_t mipi_gen_write = self->info.mipi->ops->mipi_gen_write;

	printk(KERN_DEBUG "otm8019a_enter_sleep, is_sleep = %d\n", is_sleep);

	if(is_sleep){
		sleep_in_out = sleep_in;
		size = ARRAY_SIZE(sleep_in);
	}else{
		sleep_in_out = sleep_out;
		size = ARRAY_SIZE(sleep_out);
	}

	for(i = 0; i <size ; i++){
		tag = (sleep_in_out->tag >>24);
		if(tag & LCM_TAG_SEND){
			mipi_gen_write(sleep_in_out->data, (sleep_in_out->tag & LCM_TAG_MASK));
		}else if(tag & LCM_TAG_SLEEP){
			msleep((sleep_in_out->tag & LCM_TAG_MASK));
		}
		sleep_in_out++;
	}
	return 0;
}

static struct panel_operations lcd_otm8019a_mipi_operations = {
	.panel_init = otm8019a_mipi_init,
	.panel_readid = otm8019a_readid,
	.panel_enter_sleep = otm8019a_enter_sleep,
	.panel_esd_check = otm8019a_check_esd,
};

static struct timing_rgb lcd_otm8019a_mipi_timing = {
	.hfp = 80,  /* unit: pixel */
	.hbp = 80,
	.hsync = 6,
	.vfp = 10, /*unit: line*/
	.vbp = 10,
	.vsync = 6,
};

static struct info_mipi lcd_otm8019a_mipi_info = {
	.work_mode  = SPRDFB_MIPI_MODE_VIDEO,
	.video_bus_width = 24, /*18,16*/
	.lan_number = 2,
	.phy_feq = 480*1000,
	.h_sync_pol = SPRDFB_POLARITY_POS,
	.v_sync_pol = SPRDFB_POLARITY_POS,
	.de_pol = SPRDFB_POLARITY_POS,
	.te_pol = SPRDFB_POLARITY_POS,
	.color_mode_pol = SPRDFB_POLARITY_NEG,
	.shut_down_pol = SPRDFB_POLARITY_NEG,
	.timing = &lcd_otm8019a_mipi_timing,
	.ops = NULL,
};

struct panel_spec lcd_otm8019a_mipi_spec = {
	.width = 480,
	.height = 800,
    .fps = 60,
	.type = LCD_MODE_DSI,
	.direction = LCD_DIRECT_NORMAL,
	.is_clean_lcd = true,
	.info = {
		.mipi = &lcd_otm8019a_mipi_info
	},
	.ops = &lcd_otm8019a_mipi_operations,
};

struct panel_cfg lcd_otm8019a_mipi = {
	/* this panel can only be main lcd */
	.dev_id = SPRDFB_MAINLCD_ID,
	.lcd_id = 0x8019,
	.lcd_name = "lcd_otm8019a_mipi",
	.panel = &lcd_otm8019a_mipi_spec,
};

static int __init lcd_otm8019a_mipi_init(void)
{
	return sprdfb_panel_register(&lcd_otm8019a_mipi);
}

subsys_initcall(lcd_otm8019a_mipi_init);
