/*
 *
 * $Id: stk3x1x.h
 *
 * Copyright (C) 2012~2014 Lex Hsieh     <lex_hsieh@sensortek.com.tw> 
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef __STK3X1X_H__
#define __STK3X1X_H__

/* platform data */
struct stk3x1x_platform_data
{
	uint8_t state_reg;
	uint8_t psctrl_reg;
	uint8_t alsctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t	wait_reg;	
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
	int int_pin;
	uint32_t transmittance;
	uint32_t int_flags;	
};

#define STK_IOCTL_MAGIC         0x1C
#define STK_IOCTL_GET_PFLAG     _IOR(STK_IOCTL_MAGIC, 1, int)
#define STK_IOCTL_GET_LFLAG     _IOR(STK_IOCTL_MAGIC, 2, int)
#define STK_IOCTL_SET_PFLAG     _IOW(STK_IOCTL_MAGIC, 3, int)
#define STK_IOCTL_SET_LFLAG     _IOW(STK_IOCTL_MAGIC, 4, int)
#define STK_IOCTL_GET_DATA      _IOW(STK_IOCTL_MAGIC, 5, unsigned char)

#define STK3X1X_DEV_NAME      "ltr_558als"
#define STK3X1X_INPUT_DEV     "alps_pxy"

#endif // __STK3X1X_H__
