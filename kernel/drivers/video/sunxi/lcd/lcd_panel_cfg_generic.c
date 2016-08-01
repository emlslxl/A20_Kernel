/*
 * Copyright (C) 2007-2012 Allwinner Technology Co., Ltd.
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

#include "lcd_panel_cfg.h"
#include "../disp/disp_lcd.h"

#define CMD_WIRTE_DELAY 2

#define SPI_DATA_PRINT

#ifdef SPI_DATA_PRINT
#define lcd_spi_dbg(msg, ...) pr_info("[LCD_SPI]" msg, ##__VA_ARGS__)
#else
#define lcd_spi_dbg(msg, ...)
#endif

static __s32 lcd_spi_cs;
static __s32 lcd_spi_clk;
static __s32 lcd_spi_mosi;
static __s32 lcd_spi_used;
static __s32 lcd_spi_module = -1;

static void
check_spi_used_value(void)
{
	if (SCRIPT_PARSER_OK !=
	    script_parser_fetch("lcd_spi_para", "lcd_spi_used",
				&lcd_spi_used, 1))
		__inf("LCD SPI doesn't use.\n");

	if (!lcd_spi_used)
		__inf("LCD SPI doesn't use.\n");
}

static void
LCD_SPI_Init(__u32 sel)
{
	if (SCRIPT_PARSER_OK !=
	    script_parser_fetch("lcd_spi_para", "lcd_spi_module",
				&lcd_spi_module, 1)) {
		__wrn("There is no LCD SPI module input.\n");
		return;
	}

	//set enable gpio to correct level
	gpio_request_ex("lcd_spi_para", "lcd_spi_enable");

	lcd_spi_cs = gpio_request_ex("lcd_spi_para", "lcd_spi_cs");
	if (!lcd_spi_cs) {
		__wrn("request gpio lcd_spi_cs error.\n");
		goto ERR1;
	}
	lcd_spi_clk = gpio_request_ex("lcd_spi_para", "lcd_spi_clk");
	if (!lcd_spi_clk) {
		__wrn("request gpio lcd_spi_clk error.\n");
		goto ERR2;
	}
	lcd_spi_mosi = gpio_request_ex("lcd_spi_para", "lcd_spi_mosi");
	if (!lcd_spi_mosi) {
		__wrn("request gpio lcd_spi_mosi error.\n");
		goto ERR3;
	}
	return;

	lcd_spi_dbg("release GPIO src : lcd_spi_mosi\n");
	gpio_release(lcd_spi_mosi, 2);
ERR3:
	lcd_spi_dbg("release GPIO src : lcd_spi_clk\n");
	gpio_release(lcd_spi_clk, 2);
ERR2:
	lcd_spi_dbg("release GPIO src : lcd_spi_cs\n");
	gpio_release(lcd_spi_cs, 2);
ERR1:
	return;
}

#define PULSE_DATA(d)	do{	\
		gpio_write_one_pin_value(lcd_spi_mosi, (d)&1, "lcd_spi_mosi");\
		gpio_write_one_pin_value(lcd_spi_clk, 0, "lcd_spi_clk");\
		udelay(CMD_WIRTE_DELAY);\
		gpio_write_one_pin_value(lcd_spi_clk, 1, "lcd_spi_clk");\
		udelay(CMD_WIRTE_DELAY);}while(0)

static void SPI_write_lq035q3_data(u8 dc, u8 data)
{
	int i;

	gpio_write_one_pin_value(lcd_spi_cs, 0, "lcd_spi_cs");

	PULSE_DATA(dc);
	
	for(i=0;i<8;i++){
		PULSE_DATA(data>>7);
		data<<=1;
	}

	gpio_write_one_pin_value(lcd_spi_cs, 1, "lcd_spi_cs");
	udelay(CMD_WIRTE_DELAY);
}

static void SPI_write_lq035q3_reg(u16 reg, u16 data)
{
	SPI_write_lq035q3_data(0, reg);
	SPI_write_lq035q3_data(1, data>>8);
	SPI_write_lq035q3_data(1, data);
}

static void SPI_write_lq035q3(void)
{
	struct spi_data{
		u16 reg;
		u16 data;
	};

	const struct spi_data spidata[]={
			{0x01, 0x2AEF},
			{0x02, 0x0300},
			{0x03, 0x787E},
			{0x0B, 0xDC00},
			{0x0C, 0x0005},
			{0x0D, 0x0002},
			{0x0E, 0x2900},
			{0x0F, 0x0000},
			{0x0F, 0x0000},
			{0x16, 0x9F86},
			{0x17, 0x0002},
			{0x1E, 0x0000},
			{0x2E, 0xB945},
			{0x30, 0x0301},
			{0x31, 0x0107},
			{0x32, 0x0000},
			{0x33, 0x0100},
			{0x34, 0x0707},
			{0x35, 0x0006},
			{0x36, 0x0604},
			{0x37, 0x0103},
			{0x3A, 0x0D0F},
			{0x3B, 0x0D04},
			{0x28, 0x0006},
			{0x2C, 0xC88C},
	};
	
	int i;
	const struct spi_data *p = spidata;

	for(i=0;i<ARRAY_SIZE(spidata);i++,p++){
		SPI_write_lq035q3_reg(p->reg, p->data);
	}
}

static void
LCD_SPI_Write(__u32 sel)
{
	int i = 0, j = 0, offset = 0, bit_val = 0, ret = 0;
	u16 data[9] = {		/* module 0 data */
		0x0029,		/* reset */
		0x0025,		/* standby */
		0x0840,		/* enable normally black */
		0x0430,		/* enable FRC/dither */
		0x385f,		/* enter test mode(1) */
		0x3ca4,		/* enter test mode(2) */
		0x3409,		/* enable SDRRS, enlarge OE width */
		0x4041,		/* adopt 2 line / 1 dot */
		/* wait 100ms */
		0x00ad,		/* display on */
	};

	lcd_spi_dbg
	    ("============ start LCD SPI data write, module = %d============\n",
	     lcd_spi_module);

	switch (lcd_spi_module) {
	case 0: /* rili 7inch */
		for (i = 0; i < 8; i++) {
			gpio_write_one_pin_value(lcd_spi_cs, 0, "lcd_spi_cs");
			lcd_spi_dbg("write data[%d]:", i);

			for (j = 0; j < 16; j++) {
				gpio_write_one_pin_value(lcd_spi_clk, 0,
							 "lcd_spi_clk");
				offset = 15 - j;
				bit_val = 0x0001 & (data[i] >> offset);
				ret = gpio_write_one_pin_value
					(lcd_spi_mosi, bit_val, "lcd_spi_mosi");
#ifdef SPI_DATA_PRINT
				if (ret == 0)
					lcd_spi_dbg("%d-", bit_val);
				else
					lcd_spi_dbg("write[bit:%d]ERR", j);
#endif
				udelay(CMD_WIRTE_DELAY);
				gpio_write_one_pin_value(lcd_spi_clk, 1,
							 "lcd_spi_clk");
				udelay(CMD_WIRTE_DELAY);
			}

			lcd_spi_dbg("\n");
			gpio_write_one_pin_value(lcd_spi_cs, 1, "lcd_spi_cs");
			gpio_write_one_pin_value(lcd_spi_clk, 1, "lcd_spi_clk");
			udelay(CMD_WIRTE_DELAY);
		}
		msleep(50);
		gpio_write_one_pin_value(lcd_spi_cs, 0, "lcd_spi_cs");

		lcd_spi_dbg("write data[8]:");

		for (j = 0; j < 16; j++) {
			gpio_write_one_pin_value(lcd_spi_clk, 0, "lcd_spi_clk");
			offset = 15 - j;
			bit_val = (0x0001 & (data[i] >> offset));
			ret = gpio_write_one_pin_value(lcd_spi_mosi, bit_val,
						       "lcd_spi_mosi");
#ifdef SPI_DATA_PRINT
			if (ret == 0)
				lcd_spi_dbg("%d-", bit_val);
			else
				lcd_spi_dbg("write[bit:%d]ERR", j);
#endif
			udelay(CMD_WIRTE_DELAY);
			gpio_write_one_pin_value(lcd_spi_clk, 1, "lcd_spi_clk");
			udelay(CMD_WIRTE_DELAY);
		}

		lcd_spi_dbg("\n");
		gpio_write_one_pin_value(lcd_spi_cs, 1, "lcd_spi_cs");
		gpio_write_one_pin_value(lcd_spi_clk, 1, "lcd_spi_clk");
		udelay(CMD_WIRTE_DELAY);
		lcd_spi_dbg("LCD SPI data translation finished\n");
		break;
	case 1: /* lq035q3 */
		SPI_write_lq035q3();
		break;
		
	default:
		lcd_spi_dbg("%s Unknow lcd_spi_module\n", __func__);
		break;
	}
}

static void
LCD_SPI_Dinit(__u32 sel)
{
	lcd_spi_dbg("release GPIO src : lcd_spi_mosi\n");
	if (lcd_spi_mosi)
		gpio_release(lcd_spi_mosi, 2);

	lcd_spi_dbg("release GPIO src : lcd_spi_clk\n");
	if (lcd_spi_clk)
		gpio_release(lcd_spi_clk, 2);

	lcd_spi_dbg("release GPIO src : lcd_spi_cs\n");
	if (lcd_spi_cs)
		gpio_release(lcd_spi_cs, 2);
}

void LCD_power_on_generic(__u32 sel)
{
	LCD_POWER_EN(sel, 1); /* config lcd_power pin to open lcd power */
}

void LCD_power_off_generic(__u32 sel)
{
	LCD_POWER_EN(sel, 0); /* config lcd_power pin to close lcd power */
}

void LCD_bl_open_generic(__u32 sel)
{
	LCD_PWM_EN(sel, 1); /* open pwm module */
	LCD_BL_EN(sel, 1); /* config lcd_bl_en pin to open lcd backlight */
}

void LCD_bl_close_generic(__u32 sel)
{
	LCD_BL_EN(sel, 0); /* config lcd_bl_en pin to close lcd backlight */
	LCD_PWM_EN(sel, 0); /* close pwm module */
}

__s32 LCD_open_flow_generic(__u32 sel)
{
	check_spi_used_value();
	/* open lcd power, and delay 50ms */
	LCD_OPEN_FUNC(sel, LCD_power_on_generic, 5);
	if (lcd_spi_used) {
		/* request and init gpio, and delay 20ms */
		LCD_OPEN_FUNC(sel, LCD_SPI_Init, 0);
		/*
		 * use gpio to config lcd module to the work mode,
		 * and delay 10ms
		 */
		LCD_OPEN_FUNC(sel, LCD_SPI_Write, 0);
	}
	/* open lcd controller, and delay 500ms */
	LCD_OPEN_FUNC(sel, TCON_open, 5);
	/* open lcd backlight, and delay 0ms */
	LCD_OPEN_FUNC(sel, LCD_bl_open_generic, 0);

	return 0;
}

__s32 LCD_close_flow_generic(__u32 sel)
{
	/* close lcd backlight, and delay 0ms */
	LCD_CLOSE_FUNC(sel, LCD_bl_close_generic, 0);
	/* close lcd controller, and delay 0ms */
	LCD_CLOSE_FUNC(sel, TCON_close, 0);
	if (lcd_spi_used)
		/* release gpio, and delay 0ms */
		LCD_CLOSE_FUNC(sel, LCD_SPI_Dinit, 0);

	/* close lcd power, and delay 1000ms */
	LCD_CLOSE_FUNC(sel, LCD_power_off_generic, 1000);

	return 0;
}

/* sel: 0:lcd0; 1:lcd1 */
__s32 LCD_user_defined_func_generic(__u32 sel, __u32 para1, __u32 para2,
				    __u32 para3)
{
	return 0;
}

void LCD_get_panel_funs_generic(__lcd_panel_fun_t *fun)
{
	/* lcd panel info defined in sys_config1.fex */
	fun->cfg_panel_info = NULL;
	fun->cfg_open_flow = LCD_open_flow_generic;
	fun->cfg_close_flow = LCD_close_flow_generic;
	fun->lcd_user_defined_func = LCD_user_defined_func_generic;
}
