/*
 * st7735fb.c -- FB driver for ST7735 SPI controller
 *
 * Copyright (C) 2014, threewater
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Layout is based on skeletonfb.c by James Simmons and Geert Uytterhoeven.
 *
 * This driver is written to be used with the ST7735 display controller.
 *
 * It is intended to be architecture independent. A board specific driver
 * must be used to perform all the physical IO interactions.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/splashfb.h>

//#define DEBUG
#ifdef DEBUG
#define DPRINTF(x...) printk("st7735: "x)
#else
#define DPRINTF(x...)
#endif

#define BIT_PER_PIXEL	24
#define DPY_W 128
#define DPY_H 128

#define BYTE_PER_PIXEL	(BIT_PER_PIXEL>>3)

#define ST7735_NOP		(0x0)
#define ST7735_SWRESET	(0x01)
#define ST7735_SLPIN	(0x10)
#define ST7735_SLPOUT	(0x11)
#define ST7735_PTLON	(0x12)
#define ST7735_NORON	(0x13)
#define ST7735_INVOFF	(0x20)
#define ST7735_INVON	(0x21)
#define ST7735_GAMSET	(0x26)
#define ST7735_DISPON	(0x29)
#define ST7735_CASET	(0x2A)
#define ST7735_RASET	(0x2B)
#define ST7735_RAMWR	(0x2C)
#define ST7735_COLMOD	(0x3A)
#define ST7735_MADCTL	(0x36)
#define ST7735_FRMCTR1	(0xB1)
#define ST7735_INVCTR	(0xB4)
#define ST7735_DISSET5	(0xB6)
#define ST7735_PWCTR1	(0xC0)
#define ST7735_PWCTR2	(0xC1)
#define ST7735_PWCTR3	(0xC2)
#define ST7735_VMCTR1	(0xC5)
#define ST7735_VMOFCTR	(0xC7)
#define ST7735_PWCTR6	(0xFC)
#define ST7735_GMCTRP1	(0xE0)
#define ST7735_GMCTRN1	(0xE1)
#define ST7735_GAM2		(0xF2)

struct st7735fb_par {
	struct fb_info *info;
	struct spi_device* spi;
	unsigned int cmd_io;
//	wait_queue_head_t waitq;
	struct mutex io_lock;
};

/* main st7735fb functions */
static int st7735_write_byte(struct st7735fb_par *par, int iscmd, uint8_t data)
{
	struct spi_device* spi = par->spi;

	struct spi_message msg;
	struct spi_transfer xfers;

	gpio_set_value(par->cmd_io, iscmd?0:1);

	spi_message_init(&msg);
	memset(&xfers, 0, sizeof(xfers));

	xfers.tx_buf = &data;
	xfers.rx_buf = NULL;
	xfers.len = 1;
	spi_message_add_tail(&xfers, &msg);

	return spi_sync(spi, &msg);
}

static int st7735_burst_write_data(struct st7735fb_par *par, void* buf, int size)
{
	static char tx_buf[DPY_W*DPY_H*BYTE_PER_PIXEL];
	struct spi_device* spi = par->spi;

	struct spi_message msg;
	struct spi_transfer xfers;

	gpio_set_value(par->cmd_io, 1);

	spi_message_init(&msg);
	memset(&xfers, 0, sizeof(xfers));
	memcpy(tx_buf, buf, size);

	xfers.tx_buf = tx_buf;
	xfers.rx_buf = NULL;
	xfers.len = size;
	spi_message_add_tail(&xfers, &msg);

	return spi_sync(spi, &msg);
}

static void st7735_write_cmd(struct st7735fb_par *par, uint8_t command)
{
	st7735_write_byte(par, 1, command);
}

static void st7735_write_data(struct st7735fb_par *par, uint8_t data)
{
	st7735_write_byte(par, 0, data);
}

static int st7735_init_display(struct st7735fb_par *par)
{
	int ret;
	ret = gpio_request(par->cmd_io, "st7735 cmd");
	if(ret<0){
		printk("st7735 cmd gpio request failed\n");
		return ret;
	}

	gpio_direction_output(par->cmd_io, 0);

	st7735_write_cmd(par, ST7735_SLPOUT);  // out of sleep mode
	udelay(500);

	st7735_write_cmd(par, ST7735_GAMSET); //Set Default Gamma 
	st7735_write_data(par,0x04);
	st7735_write_cmd(par, ST7735_FRMCTR1);//Set Frame Rate 
	st7735_write_data(par,0x08);
	st7735_write_data(par,0x14);
	st7735_write_cmd(par, ST7735_PWCTR1); //Set VRH1[4:0] & VC[2:0] for VCI1 & G
	st7735_write_data(par,0x08);
	st7735_write_data(par,0x00);
	st7735_write_cmd(par, ST7735_PWCTR2); //Set BT[2:0] for AVDD & VCL & VGH &
	st7735_write_data(par,0x05);
	st7735_write_cmd(par, ST7735_VMCTR1); //Set VMH[6:0] & VML[6:0] for VOMH &
	st7735_write_data(par,0x46);
	st7735_write_data(par,0x40);
	st7735_write_cmd(par, ST7735_VMOFCTR);// Set VMF 
	st7735_write_data(par,0xC2);
	st7735_write_cmd(par, ST7735_COLMOD); //Set Color Format 18bit
	st7735_write_data(par,0x06);
	st7735_write_cmd(par, ST7735_CASET); //Set Column Address 
	st7735_write_data(par,0x00);
	st7735_write_data(par,0x00);
	st7735_write_data(par,0x00);
	st7735_write_data(par,0x80);
	st7735_write_cmd(par, ST7735_RASET); //Set Page Address 
	st7735_write_data(par,0x00);
	st7735_write_data(par,0x00);
	st7735_write_data(par,0x00);
	st7735_write_data(par,0xA0);
	st7735_write_cmd(par, ST7735_INVCTR);
	st7735_write_data(par,0x00);
	st7735_write_cmd(par, ST7735_MADCTL); ////RGB/BGR
	st7735_write_data(par,0xC8);//08

#if 0
	st7735_write_cmd(par, ST7735_GAM2); //Enable Gamma b
	st7735_write_data(par,0x01);
	st7735_write_cmd(par, ST7735_GMCTRP1);
	st7735_write_data(par,0x3F);//p1 
	st7735_write_data(par,0x26);//p2 
	st7735_write_data(par,0x23);//p3 
	st7735_write_data(par,0x30);//p4 
	st7735_write_data(par,0x28);//p5 
	st7735_write_data(par,0x10);//p6 
	st7735_write_data(par,0x55);//p7 
	st7735_write_data(par,0xB7);//p8 
	st7735_write_data(par,0x40);//p9 
	st7735_write_data(par,0x19);//p10 
	st7735_write_data(par,0x10);//p11 
	st7735_write_data(par,0x1E);//p12 
	st7735_write_data(par,0x02);//p13 
	st7735_write_data(par,0x01);//p14 
	st7735_write_data(par,0x00);//p15 
	st7735_write_cmd(par, ST7735_GMCTRN1);
	st7735_write_data(par,0x00);//p1 
	st7735_write_data(par,0x19);//p2 
	st7735_write_data(par,0x1C);//p3 
	st7735_write_data(par,0x0F);//p4 
	st7735_write_data(par,0x14);//p5 
	st7735_write_data(par,0x0F);//p6 
	st7735_write_data(par,0x2A);//p7 
	st7735_write_data(par,0x48);//p8 
	st7735_write_data(par,0x3F);//p9 
	st7735_write_data(par,0x06);//p10 
	st7735_write_data(par,0x1D);//p11 
	st7735_write_data(par,0x21);//p12 
	st7735_write_data(par,0x3d);//p13 
	st7735_write_data(par,0x3e);//p14 
	st7735_write_data(par,0x3f);//p15 
#endif
	st7735_write_cmd(par, ST7735_DISPON); // Display On 

	return 0;
}

static void st7735_set_addr_window(struct st7735fb_par *par, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	st7735_write_cmd(par, ST7735_CASET);   // column addr set
	st7735_write_data(par, 0x00);
	st7735_write_data(par, x0+2);		   // XSTART 
	st7735_write_data(par, 0x00);
	st7735_write_data(par, x1+2);		   // XEND

	st7735_write_cmd(par, ST7735_RASET);   // row addr set
	st7735_write_data(par, 0x00);
	st7735_write_data(par, y0+3);		   // YSTART
	st7735_write_data(par, 0x00);
	st7735_write_data(par, y1+3);		   // YEND
}

static struct fb_fix_screeninfo st7735fb_fix __devinitdata = {
	.id =		"st7735fb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,
	.line_length =	DPY_W*BYTE_PER_PIXEL,
	.accel =	FB_ACCEL_NONE,
};

static struct fb_var_screeninfo st7735fb_var __devinitdata = {
	.xres		= DPY_W,
	.yres		= DPY_H,
	.xres_virtual	= DPY_W,
	.yres_virtual	= DPY_H,
	.bits_per_pixel	= BIT_PER_PIXEL,
	.red =		{ 0, 8, 0 },
	.green =	{ 8, 8, 0 },
	.blue =		{ 16, 8, 0 },
	.transp =	{ 0, 0, 0 },
};

static void st7735fb_dpy_update_pages(struct st7735fb_par *par,
						u16 y1, u16 y2)
{
	unsigned char *buf = (unsigned char *)par->info->screen_base;

	DPRINTF("%s:y %d to %d\n", __FUNCTION__, y1, y2);
	buf += y1 * par->info->var.xres * BYTE_PER_PIXEL;

	mutex_lock(&(par->io_lock));

	st7735_set_addr_window(par, 0,y1,DPY_W-1,y2-1);
	st7735_write_cmd(par, ST7735_RAMWR);  // write to RAM

	st7735_burst_write_data(par, buf, DPY_W*(y2-y1)*BYTE_PER_PIXEL);
	st7735_write_cmd(par, ST7735_NOP);

	mutex_unlock(&(par->io_lock));
}

static void st7735fb_dpy_update(struct st7735fb_par *par)
{
	uint16_t *buf = (uint16_t *)par->info->screen_base;

	DPRINTF("%s\n", __FUNCTION__);

	mutex_lock(&(par->io_lock));
	st7735_set_addr_window(par, 0,0,DPY_W-1,DPY_H-1);
	st7735_write_cmd(par, ST7735_RAMWR);  // write to RAM

	st7735_burst_write_data(par, buf, DPY_W*DPY_H*BYTE_PER_PIXEL);
	st7735_write_cmd(par, ST7735_NOP);

	mutex_unlock(&(par->io_lock));
}

/* this is called back from the deferred io workqueue */
static void st7735fb_dpy_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	u16 y1 = 0, h = 0;
	int prev_index = -1;
	struct page *cur;
	struct fb_deferred_io *fbdefio = info->fbdefio;
	int h_inc;
	u16 yres = info->var.yres;
	u16 xres = info->fix.line_length;;

	/* height increment is fixed per page */
	h_inc = DIV_ROUND_UP(PAGE_SIZE , xres);

	/* walk the written page list and swizzle the data */
	list_for_each_entry(cur, &fbdefio->pagelist, lru) {
		if (prev_index < 0) {
			/* just starting so assign first page */
			y1 = (cur->index << PAGE_SHIFT) / xres;
			h = h_inc;
		} else if ((prev_index + 1) == cur->index) {
			/* this page is consecutive so increase our height */
			h += h_inc;
		} else {
			/* page not consecutive, issue previous update first */
			st7735fb_dpy_update_pages(info->par, y1, y1 + h);
			/* start over with our non consecutive page */
			y1 = (cur->index << PAGE_SHIFT) / xres;
			h = h_inc;
		}
		prev_index = cur->index;
	}

	/* if we still have any pages to update we do so now */
	if (h >= yres) {
		/* its a full screen update, just do it */
		st7735fb_dpy_update(info->par);
	} else {
		st7735fb_dpy_update_pages(info->par, y1,
						min((u16) (y1 + h), yres));
	}
}

static void st7735fb_fillrect(struct fb_info *info,
				   const struct fb_fillrect *rect)
{
	struct st7735fb_par *par = info->par;

	DPRINTF("%s\n", __FUNCTION__);

	sys_fillrect(info, rect);

	st7735fb_dpy_update(par);
}

static void st7735fb_copyarea(struct fb_info *info,
				   const struct fb_copyarea *area)
{
	struct st7735fb_par *par = info->par;

	DPRINTF("%s\n", __FUNCTION__);

	sys_copyarea(info, area);

	st7735fb_dpy_update(par);
}

static void st7735fb_imageblit(struct fb_info *info,
				const struct fb_image *image)
{
	struct st7735fb_par *par = info->par;

	DPRINTF("%s\n", __FUNCTION__);

	sys_imageblit(info, image);

	st7735fb_dpy_update(par);
}

/*
 * this is the slow path from userspace. they can seek and write to
 * the fb. it's inefficient to do anything less than a full screen draw
 */
static ssize_t st7735fb_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct st7735fb_par *par = info->par;
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long total_size;

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	dst = (void *)(info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		err = -EFAULT;

	if	(!err)
		*ppos += count;

	st7735fb_dpy_update(par);

	return (err) ? err : count;
}

static struct fb_ops st7735fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read		= fb_sys_read,
	.fb_write	= st7735fb_write,
	.fb_fillrect	= st7735fb_fillrect,
	.fb_copyarea	= st7735fb_copyarea,
	.fb_imageblit	= st7735fb_imageblit,
};

static struct fb_deferred_io st7735fb_defio = {
	.delay		= HZ/4,
	.deferred_io	= st7735fb_dpy_deferred_io,
};

static int __devinit st7735_spi_probe(struct spi_device *spi)
{
	int err, size;
	struct fb_info *info;
	struct st7735fb_par *par;

	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err) {
		dev_dbg(&spi->dev, "spi master setup failed\n");
		return err;
	}

	info = framebuffer_alloc(sizeof(struct st7735fb_par), &spi->dev);
	if (info == NULL) {
		dev_err(&spi->dev, "no memory for device state\n");
		return -ENOMEM;
	}

	size = roundup(DPY_W*DPY_H*BYTE_PER_PIXEL, PAGE_SIZE);

//	info->screen_base = kmalloc(size, GFP_KERNEL);
	info->screen_base = vzalloc(size);

	if (!info->screen_base){
		err = -ENOMEM;
		goto err_fb_rel;
	}
	
	info->fbops = &st7735fb_ops;

	st7735fb_var.xres = DPY_W;
	st7735fb_var.yres = DPY_H;
	st7735fb_var.xres_virtual = DPY_W;
	st7735fb_var.yres_virtual = DPY_H;
	info->var = st7735fb_var;

	st7735fb_fix.line_length = DPY_W*BYTE_PER_PIXEL;
	info->fix = st7735fb_fix;
	info->fix.smem_len = size;

	par = info->par;
	par->spi = spi;
	par->cmd_io = 2;

	par->info = info;
	mutex_init(&par->io_lock);

	info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;

	info->fbdefio = &st7735fb_defio;
	fb_deferred_io_init(info);

	err = st7735_init_display(par);
	if (err < 0){
		goto err_free;
	}

	show_splash(info);
	st7735fb_dpy_update(par);

	err = register_framebuffer(info);
	if (err < 0){
		goto err_gpiofree;
	}

	spi_set_drvdata(spi, info);

	printk(KERN_INFO
		"fb%d: ST7735 frame buffer, using %dK of video memory\n",
		info->node, size >> 10);

	return 0;

	unregister_framebuffer(info);
err_gpiofree:
	gpio_free(par->cmd_io);
err_free:
	vfree(info->screen_base);
err_fb_rel:
	framebuffer_release(info);
	return err;
}

static int __devexit st7735_spi_remove(struct spi_device *spi)
{
	struct fb_info *info = spi_get_drvdata(spi);

	if (info){
		struct st7735fb_par *par = info->par;

		unregister_framebuffer(info);
		fb_deferred_io_cleanup(info);
		gpio_free(par->cmd_io);
		vfree((void *)info->screen_base);
		framebuffer_release(info);
	}
	return 0;
}

static struct spi_driver st7735_spi_driver = {
	.driver = {
		.name	= "st7735",
		.owner	= THIS_MODULE,
	},
	.probe		= st7735_spi_probe,
	.remove		= __devexit_p(st7735_spi_remove),
};

module_spi_driver(st7735_spi_driver);

MODULE_DESCRIPTION("fbdev driver for st7735 controller");
MODULE_AUTHOR("threewater");
MODULE_LICENSE("GPL");
