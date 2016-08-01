/*
 * arch/arm/mach-sun4i/board-ep1000m.c
 *
 * (C) Copyright 2007-2012
 * SIMIT
 * threewater <threewaterL@163.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <mach/hardware.h>

#include <plat/i2c.h>
#include <plat/platform.h>
#include <plat/platform_data.h>
#include <linux/input/matrix_keypad.h>
#include <linux/regulator/consumer.h>
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#define BUZZER_GPIO		3

static struct resource buzzer_resources = {
	.start	= BUZZER_GPIO,
	.end	= BUZZER_GPIO,
	.flags	= IORESOURCE_IO,
};

static struct platform_device device_buzzer = {
		.name			= "buzzer",
		.num_resources	= 1,
		.resource		= &buzzer_resources,
};

static struct platform_device *ep1k_pdevs[] __initdata = {
	&device_buzzer,
};

static struct i2c_board_info __initdata ep1000_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("rtc-pcf8563", 0x51),
		.type = "pcf8563",
	},
};

void __init sunxi_board_init(void)
{
	platform_add_devices(ep1k_pdevs, ARRAY_SIZE(ep1k_pdevs));
	i2c_register_board_info(1, ep1000_i2c1_boardinfo, ARRAY_SIZE(ep1000_i2c1_boardinfo));
}

#define RFID_GPIO_RESET		2

int __devinit sunxi_device_init(void)
{
	if(gpio_request(RFID_GPIO_RESET, "rfid_reset")<0){
		printk("request gpio %d failed!\n", RFID_GPIO_RESET);
	}
	else{
		gpio_direction_output(RFID_GPIO_RESET, 0);
		mdelay(1);
		gpio_set_value(RFID_GPIO_RESET, 1);
	}
	return 0;
}

subsys_initcall_sync(sunxi_device_init);

int __devinit axp152_init_power(void)
{
	struct regulator *avcc = regulator_get(NULL, "axp15_analog/fm2");
	if (IS_ERR(avcc)) {
		pr_info("Didn't find axp152 analog fm1 regulator\n");
		return -1;
	}

	return regulator_set_voltage(avcc, 2500000, 2500000);
}

device_initcall_sync(axp152_init_power);

/******************************************************************************\
	backlight
\******************************************************************************/
extern __s32 BSP_disp_lcd_set_bright(__u32 sel, __u32 bright);
extern __s32 BSP_disp_lcd_get_bright(__u32 sel);

#define MAX_BACKLIGHT_TO		100
#define MAX_BACKLIGHT_ORIG		255

static void sunxi_bl_set_intensity(int intensity)
{
	BSP_disp_lcd_set_bright(0, intensity*MAX_BACKLIGHT_ORIG/MAX_BACKLIGHT_TO);
}

static struct generic_bl_info sunxi_bl_info = {
	.name			= "backlight",
	.max_intensity		= MAX_BACKLIGHT_TO,
	.default_intensity	= 0,
	.set_bl_intensity	= sunxi_bl_set_intensity,
};

static struct platform_device sunxi_bl_dev = {
	.name			= "generic-bl",
	.id			= 1,
	.dev = {
		.platform_data	= &sunxi_bl_info,
	},
};

/*
	This function must be called after BSP_disp initialized. 
	sunxi_bl_init is defined as "__attribute__ ((weak))" 
	in drivers/video/sunxi/disp/dev_disp.c and calle in 
	function DRV_DISP_Init.
	
*/
int __init sunxi_bl_init(void)
{
	sunxi_bl_info.default_intensity = BSP_disp_lcd_get_bright(0);

	return platform_device_register(&sunxi_bl_dev);
}

