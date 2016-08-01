/* driver/misc/buzzer.c
 *
 *  Copyright (C) 2014 
 *   threewater <threewaterL@163.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <asm/uaccess.h>

#define DEVICE_NAME	"buzzer"

struct buzzer_info{
	unsigned int gpio;
	struct timer_list timer;
};

static struct buzzer_info buzzer;

#define Buzzer_off()	gpio_set_value(buzzer.gpio,0)
#define Buzzer_on()		gpio_set_value(buzzer.gpio,1)

static void buzzer_timer_handler(unsigned long data)
{
	Buzzer_off();
	del_timer(&buzzer.timer);
}

static ssize_t buzzer_read (struct file *file, 
		char __user *buf, size_t count, loff_t *offset)
{
	if(count<2)
		return -1;

	if(gpio_get_value(buzzer.gpio))
		copy_to_user(buf, "B\n", 2);
	else
		copy_to_user(buf, "b\n", 2);

	return 2;
}

static ssize_t buzzer_write (struct file *file, 
		const char __user *buf, size_t count, loff_t *offset)
{
	char data[count];
	unsigned long time;

	copy_from_user(data, buf, count);

	if(buf[0]=='B'){
		Buzzer_on();
		return count;
	}

	if(buf[0]=='0' || buf[0]=='b'){
		Buzzer_off();
		return count;
	}

	time = simple_strtoul(buf, NULL, 0);
	if(time==0){
		Buzzer_off();
		return count;
	}
	time = HZ*time/1000;

	Buzzer_on();
	mod_timer(&buzzer.timer, jiffies + time);

	return count;
}

static const struct file_operations buzzer_fops = {
	.read		= buzzer_read,
	.write		= buzzer_write,
};

//Initialize driver.
static struct miscdevice misc_buzzer = {
	.minor = MISC_DYNAMIC_MINOR, 
	.name = DEVICE_NAME, 
	.fops = &buzzer_fops,
};

static int __devinit buzzer_probe(struct platform_device *pdev)
{
	unsigned int gpio;
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res)
		return -ENXIO;

	gpio = res->start;
	if(gpio_request(gpio, "buzzer_gpio")<0){
		printk("buzzer request gpio %d failed!\n", gpio);
		return -ENXIO;
	}

	gpio_direction_output(gpio,0);

	platform_set_drvdata(pdev, (void*)gpio);
	buzzer.gpio = gpio;

	init_timer(&buzzer.timer);
	buzzer.timer.function = buzzer_timer_handler;
	
	if(misc_register(&misc_buzzer))
		return -1;

	printk(DEVICE_NAME " initialized\n");
	return 0;
}

static int __devexit buzzer_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver buzzer_driver = {
	.probe		= buzzer_probe,
	.remove		= __devexit_p(buzzer_remove),
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init buzzer_init(void)
{
	return platform_driver_register(&buzzer_driver);
}

static void __exit buzzer_exit(void)
{
	platform_driver_unregister(&buzzer_driver);
}

module_init(buzzer_init);
module_exit(buzzer_exit);
MODULE_AUTHOR("threewater");

