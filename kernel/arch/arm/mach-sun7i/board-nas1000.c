/*
 * arch/arm/mach-sun7i/board-nas1000.c
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
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>
#include <mach/hardware.h>

#include <plat/sys_config.h>
#include <plat/i2c.h>
#include <plat/platform.h>
#include <plat/platform_data.h>
#include <linux/wlan_plat.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

/******************************************************************************\
	bcmhd
\******************************************************************************/

extern void sunximmc_rescan_card(unsigned id, unsigned insert);

static u32 wifi_pio_hdle;

static int bcmdhd_set_power(int val)
{
	int ret;

	if(!wifi_pio_hdle)
		return -1;

	// 1.power on, 0.power off
	ret = gpio_write_one_pin_value(wifi_pio_hdle, val, "sdio_wifi_power");

	msleep(1);
	sunximmc_rescan_card(3, val);

	return 0;
}

static struct wifi_platform_data bcmdhd_data = {
	.set_power = bcmdhd_set_power,
};

static struct resource wifi_resources[] = {
		[0] = {
				.start	= 0,
				.end	= 0,
				.flags	= IORESOURCE_IRQ|IORESOURCE_IRQ_HIGHLEVEL,
				.name = "bcmdhd_wlan_irq",
		},
};

static struct platform_device device_wifi_bcmdhd = {
		.name			= "bcmdhd_wlan",
		.id 			= -1,
		.num_resources	= ARRAY_SIZE(wifi_resources),
		.resource		= wifi_resources,
		.dev.platform_data = &bcmdhd_data,
};

/******************************************************************************\
	GPIO Buttons
\******************************************************************************/
static struct gpio_keys_button buttons[] = {
	{
		.gpio		= 5,
		.code		= KEY_UP,
		.desc		= "up",
		.type		= EV_KEY,
		.active_low	= 1,
	},
	{
		.gpio		= 3,
		.code		= KEY_ENTER,
		.desc		= "ok",
		.type		= EV_KEY,
		.active_low	= 1,
	},
	{
		.gpio		= 4,
		.code		= KEY_DOWN,
		.desc		= "down",
		.type		= EV_KEY,
		.active_low	= 1,
	},
};

static struct gpio_keys_platform_data button_data = {
	.buttons	= buttons,
	.nbuttons	= ARRAY_SIZE(buttons),
};

static struct platform_device button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &button_data,
	}
};

#define BUZZER_GPIO		6

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

static struct platform_device *nas_pdevs[] __initdata = {
	&device_wifi_bcmdhd,
	&button_device,
	&device_buzzer,
};

static struct i2c_board_info __initdata nas_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("rtc-pcf8563", 0x51),
		.type = "pcf8563",
	},
};

void __init sunxi_board_init(void)
{
	i2c_register_board_info(1, nas_i2c1_boardinfo, ARRAY_SIZE(nas_i2c1_boardinfo));

	//enable NMI Source
	#define NMI_IRQ_ENABLE_REG  (SW_VA_IO_BASE + 0x38)
	writel(1, NMI_IRQ_ENABLE_REG);
}

#define WIFI_GPIO_INT	1

int __devinit sunxi_device_init(void)
{
	if(gpio_request(WIFI_GPIO_INT, "wifi_irq")<0)
		printk("request gpio %d failed!\n", WIFI_GPIO_INT);

	wifi_resources[0].start = wifi_resources[0].end = __gpio_to_irq(WIFI_GPIO_INT);
	printk("wifi irq = %d\n", wifi_resources[0].end);

	platform_add_devices(nas_pdevs, ARRAY_SIZE(nas_pdevs));
	wifi_pio_hdle = gpio_request_ex("sdio_wifi_para", NULL);
	return 0;
}

subsys_initcall_sync(sunxi_device_init);



#if 0
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
#endif

