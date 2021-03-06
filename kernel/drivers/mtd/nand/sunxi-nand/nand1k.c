/*
 * nand1k.c
 *
 * Copyright (C) 2013 Qiang Yu <yuq825@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "nfc.h"


//////////////////////////////////////////////////////////////////////////////
// Linux charactor device interface
//////////////////////////////////////////////////////////////////////////////

#define DEV_CLASS_NAME "nand1k"
#define CHAR_DEV_NAME "nand1k"

static struct class *dev_class;
static int nand1k_major;
static char *read_buff;

static int nand1k_open(struct inode *inode, struct file *file)
{
    return 0;
}

static int nand1k_close(struct inode *inode, struct file *file)
{
    return 0;
}

static int nand1k_read(struct file *filp, char __user *buff, size_t count, loff_t *f_pos)
{
	loff_t offs = *f_pos;
	uint32_t len, ret, page, offset;
	size_t size = 0;
	printk(KERN_INFO "nand1k read off=%llx count=%x\n", offs, count);
	while (size < count) {
		page = offs / 1024;
		offset = offs % 1024;
		len = 1024 - offset;
		if (len > count - size)
			len = count - size;
		nfc_read_page1k(page, read_buff);
		ret = copy_to_user(buff, read_buff + offset, len);
		printk(KERN_INFO "nand1k read page=%x offset=%x len=%x\n", page, offset, len);
		printk(KERN_INFO "nand1k %x %x %x %x %x %x %x %x\n", 
			   read_buff[0], read_buff[1], read_buff[2], read_buff[3],
			   read_buff[4], read_buff[5], read_buff[6], read_buff[7]);
		size += len - ret;
		offs += len - ret;
		buff += len - ret;
		if (ret)
			break;
	}
	*f_pos += size;
    return size;
}

static int nand1k_write(struct file *filp, const char __user *buff, size_t count, loff_t *f_pos)
{
    return 0;
}

static long nand1k_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {
    default:
		return -ENOTTY;
    }

    return 0;
}

struct file_operations nand1k_fops = {
	.read = nand1k_read,
	.write = nand1k_write,
	.open = nand1k_open,
	.release = nand1k_close,
	.unlocked_ioctl = nand1k_ioctl,
};

int nand1k_init(void)
{
	int err;
	struct device *dev;

	read_buff = kmalloc(1024, GFP_KERNEL);
	if (!read_buff) {
		printk(KERN_ERR "allocate read buffer fail\n");
		err = -ENOMEM;
		goto error0;
	}

	dev_class = class_create(THIS_MODULE, DEV_CLASS_NAME);
    if (IS_ERR(dev_class)) {
		printk(KERN_ERR "Create device class error\n");
		err = PTR_ERR(dev_class);
		goto error1;
    }

	nand1k_major = register_chrdev(0, CHAR_DEV_NAME, &nand1k_fops);
    if (nand1k_major < 0) {
		printk(KERN_ERR "register_chrdev fail\n");
		err = nand1k_major;
		goto error2;
    }

    // Send uevents to udev, so it'll create /dev nodes
    dev = device_create(dev_class, NULL, MKDEV(nand1k_major, 0), NULL, CHAR_DEV_NAME);
    if (IS_ERR(dev)) {
		printk(KERN_ERR "device_create fail\n");
		err = PTR_ERR(dev);
		goto error3;
    }

	return 0;

error3:
	unregister_chrdev(nand1k_major, CHAR_DEV_NAME);
error2:
	class_destroy(dev_class);
error1:
	kfree(read_buff);
error0:
	return err;
}

void nand1k_exit(void)
{
	device_destroy(dev_class, MKDEV(nand1k_major, 0));
    unregister_chrdev(nand1k_major, CHAR_DEV_NAME);
	class_destroy(dev_class);
	kfree(read_buff);
}


