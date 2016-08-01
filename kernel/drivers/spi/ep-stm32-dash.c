/*
 * EP1000 platform STM32 drivers (SPI bus)
 *
 * Copyright (C) 2013-2015 threewater<threewaterl@163.com>
 *
 * Licensed under the GPL-2 or later.
 */
#define DEBUG
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>

#define SPI_READ			0
#define SPI_WRITE			(1<<7)
#define CHIP_ID				0x55

#define CMD2ADDRESS(cmd)	(cmd&(~SPI_WRITE))

// Commands
#define CMD_CTRL_BASE		0x0					//
#define CMD_STATUS			(CMD_CTRL_BASE+0)	//控制状态字，长度8bit
#define CMD_TEMPERATURE		(CMD_CTRL_BASE+1)	//temerpature 
#define CMD_DIN_KEY			(CMD_CTRL_BASE+2)	//键盘队列中断状态字，写入无效，长度8bit，由DI的低8bit控制

#define CMD_TEST1			(CMD_CTRL_BASE+0xa)	//测试寄存器，长度8bit
#define CMD_CHIPID			(CMD_CTRL_BASE+0xf)	//长度8bit

#define CMD_DIN_BASE		0x10			//数据输入基地址，写入无效，每地址长度16bit

#define CMD_DOUT_BASE		0x20			//数据输出基地址，除SETB/CLRB DOUT外，每地址长度16bit
#define CMD_DOUT_GEAR		(CMD_DOUT_BASE+0)	//档位：0-P;1-R;2-N;3-D
#define CMD_DOUT_DIRECT		(CMD_DOUT_BASE+1)	//方向：0,全灭；1.left; 2.right; 3.all，闪烁
#define CMD_DOUT_FUEL		(CMD_DOUT_BASE+2)	//燃料0-100

#define CMD_SEC_BASE		0x30				//加密，保留
#define CMD_SEC_DESCRYPT	(CMD_SEC_BASE+0)

#define CMD_EEPROM_BASE		0x40				//保留
#define CMD_TIME_BASE		0x50				//保留

#define MAX_SPI_FREQ_HZ		2000000

struct mcu_data{
	struct spi_device* spi;
};

/**********************mcu spi access ***************************/
static int epstm32_spi_xfer(struct spi_device *spi,
				u8 cmd, u8 count, void *tx_buf, void *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer xfers[2];

	spi_message_init(&msg);
	memset(xfers, 0, sizeof(xfers));

	xfers[0].tx_buf = &cmd;
	xfers[0].rx_buf = NULL;
	xfers[0].len = 1;
	xfers[0].delay_usecs = 30;	//at lest 5 us
	spi_message_add_tail(&xfers[0], &msg);

	xfers[1].tx_buf = tx_buf;
	xfers[1].rx_buf = rx_buf;
	xfers[1].len = count;
	xfers[1].interbyte_usecs = 30;	//at lest 5 us

	spi_message_add_tail(&xfers[1], &msg);

	return spi_sync(spi, &msg);
}

static int epstm32_spi_read_word(struct spi_device *spi, u8 cmd)
{
	u16 ret;

	return epstm32_spi_xfer(spi, cmd|SPI_READ, 2, NULL, &ret) ? : ret;
}

static int epstm32_spi_write_word(struct spi_device *spi, u8 cmd, u16 val)
{
	return epstm32_spi_xfer(spi, cmd|SPI_WRITE, 2, &val, NULL);
}

static int epstm32_spi_read_byte(struct spi_device *spi, u8 cmd)
{
	u8 ret;

	return epstm32_spi_xfer(spi, cmd|SPI_READ, 1, NULL, &ret) ? : ret;
}

static int epstm32_spi_write_byte(struct spi_device *spi, u8 cmd, u8 val)
{
	return epstm32_spi_xfer(spi, cmd|SPI_WRITE, 1, &val, NULL);
}

/**********************mcu dout***************************/
static ssize_t mcu_show_gear(struct device *dev, struct device_attribute *attr, char *buf){
		struct mcu_data *mcu = dev_get_drvdata(dev);
		return sprintf(buf, "%d\n", epstm32_spi_read_byte(mcu->spi, CMD_DOUT_GEAR));
}

static ssize_t mcu_store_gear(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size){

	struct mcu_data *mcu = dev_get_drvdata(dev);
	char *end;
	unsigned long v = simple_strtoul(buf, &end, 0);
	if (end == buf)
		return -EINVAL;
	epstm32_spi_write_byte(mcu->spi, CMD_DOUT_GEAR, v);
	return size;
}

static DEVICE_ATTR(gear, 0644,mcu_show_gear, mcu_store_gear);


static const struct attribute *mcu_attrs[] = {
	&dev_attr_gear.attr,
	NULL, 
}; 

static const struct attribute_group mcu_attr_group = { 
	.name = "mcu",		/* put in device directory */
	.attrs = (struct attribute **) mcu_attrs
}; 

static int __devinit epstm32_spi_probe(struct spi_device *spi)
{
	int err;
	struct mcu_data *mcudata;

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
		return -EINVAL;
	}

	mcudata = kzalloc(sizeof(struct mcu_data), GFP_KERNEL);
	if (mcudata == NULL) {
		dev_err(&spi->dev, "no memory for device state\n");
		return -ENOMEM;
	}
	mcudata->spi = spi;

	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err) {
		dev_dbg(&spi->dev, "spi master setup failed\n");
		kfree(mcudata);
		return err;
	}

	err = epstm32_spi_read_byte(spi, CMD_CHIPID);
	if(err!=CHIP_ID){
		dev_dbg(&spi->dev, "wrong stm32 spi chip id 0x%x\n", err);
		kfree(mcudata);
		return -1;
	}

	sysfs_create_group(&spi->dev.kobj, &mcu_attr_group);
	spi_set_drvdata(spi, mcudata);

	return 0;
}

static int __devexit epstm32_spi_remove(struct spi_device *spi)
{
	struct mcu_data *mcu = spi_get_drvdata(spi);

	sysfs_remove_group(&spi->dev.kobj, &mcu_attr_group);

	spi_set_drvdata(spi, NULL);
	kfree(mcu);
	return 0;
}

static struct spi_driver epstm32_spi_driver = {
	.driver = {
		.name	= "stm32dash",
		.owner	= THIS_MODULE,
	},
	.probe		= epstm32_spi_probe,
	.remove		= __devexit_p(epstm32_spi_remove),
};

module_spi_driver(epstm32_spi_driver);
MODULE_AUTHOR("threewater <threewaterl@163.com>");
MODULE_LICENSE("GPL v2");
