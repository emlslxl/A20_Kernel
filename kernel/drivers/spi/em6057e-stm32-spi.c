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

#define MCUKEY_DOWN			0x80

#define SPI_READ			0
#define SPI_WRITE			(1<<7)
#define CHIP_ID				0x57

#define CMD2ADDRESS(cmd)	(cmd&(~SPI_WRITE))

// Commands
#define CMD_CTRL_BASE		0x0					//
#define CMD_STATUS			(CMD_CTRL_BASE+0)	//控制状态字，长度8bit
#define CMD_TEMPERATURE		(CMD_CTRL_BASE+1)	//temerpature 
#define CMD_DIN_KEY			(CMD_CTRL_BASE+2)	//键盘队列中断状态字，写入无效，长度8bit，由DI的低8bit控制

#define CMD_TEST1			(CMD_CTRL_BASE+0xa)	//测试寄存器，长度8bit
#define CMD_CHIPID			(CMD_CTRL_BASE+0xf)	//长度8bit

#define CMD_DIN_BASE		0x10			//数据输入基地址，写入无效，每地址长度16bit
#define CMD_DIN				(CMD_DIN_BASE+0)	//IO数据输入
#define CMD_POWER			(CMD_DIN_BASE+1)	//power
#define CMD_RI0				(CMD_DIN_BASE+2)	//RI0
#define CMD_RI1				(CMD_DIN_BASE+3)	//RI1

#define CMD_ANALOG_IN(n)	(CMD_POWER+(n))

#define CMD_DOUT_BASE		0x20			//数据输出基地址，除SETB/CLRB DOUT外，每地址长度16bit
#define CMD_DOUT			(CMD_DOUT_BASE+0)	//IO数据输出
#define CMD_SETB_DOUT		(CMD_DOUT_BASE+1)	//IO数据位高输出，长度8bit，读取无效
#define CMD_CLRB_DOUT		(CMD_DOUT_BASE+2)	//IO数据位低输出，长度8bit，读取无效
#define CMD_PWM0			(CMD_DOUT_BASE+3)
#define CMD_ANALOG_OUT(n)	(CMD_PWM0+(n))

#define CMD_SEC_BASE		0x30				//加密，保留
#define CMD_SEC_DESCRYPT	(CMD_SEC_BASE+0)

#define CMD_EEPROM_BASE		0x40				//保留
#define CMD_TIME_BASE		0x50				//保留

#define MAX_SPI_FREQ_HZ		2000000

struct keymcu_platform_data {
	unsigned short *keyMapTable;
	unsigned int keyMapTable_size;
};

struct mcu_data{
	struct spi_device* spi;
	struct input_dev *input;
	struct keymcu_platform_data* keymcu_data;
	struct delayed_work	work;
	int irq;
};

/**********************mcu spi access ***************************/
static int em6kstm32_spi_xfer(struct spi_device *spi,
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

static int em6kstm32_spi_read_word(struct spi_device *spi, u8 cmd)
{
	u16 ret;

	return em6kstm32_spi_xfer(spi, cmd|SPI_READ, 2, NULL, &ret) ? : ret;
}

static int em6kstm32_spi_write_word(struct spi_device *spi, u8 cmd, u16 val)
{
	return em6kstm32_spi_xfer(spi, cmd|SPI_WRITE, 2, &val, NULL);
}

static int em6kstm32_spi_read_byte(struct spi_device *spi, u8 cmd)
{
	u8 ret;

	return em6kstm32_spi_xfer(spi, cmd|SPI_READ, 1, NULL, &ret) ? : ret;
}

static int em6kstm32_spi_write_byte(struct spi_device *spi, u8 cmd, u8 val)
{
	return em6kstm32_spi_xfer(spi, cmd|SPI_WRITE, 1, &val, NULL);
}

/**********************mcu key***************************/
static int set_keybit(struct keymcu_platform_data *pd, struct input_dev *input_dev)
{
	if(pd){
		int i;
		for(i=0;i<pd->keyMapTable_size;i++){
			if(pd->keyMapTable[i])
				set_bit(pd->keyMapTable[i], input_dev->keybit);
		}
		return 0;
	}
	return -1;
}

static int clear_keybit(struct keymcu_platform_data *pd, struct input_dev *input_dev)
{
	if(pd){
		int i;
		for(i=0;i<pd->keyMapTable_size;i++){
			if(pd->keyMapTable[i])
				clear_bit(pd->keyMapTable[i], input_dev->keybit);
		}
		return 0;
	}
	return -1;
}

static ssize_t keymap_show_map(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	ssize_t n=0;
	struct keymcu_platform_data* pd;
	struct mcu_data *mcu = dev_get_drvdata(dev);

	pd = mcu->keymcu_data;
	
	for(i=0; i<pd->keyMapTable_size; i++){

		if(!pd->keyMapTable[i])
			continue;
		
		n+=sprintf(buf+n, "0x%02x=%d\n", i, pd->keyMapTable[i]);
	}
	
	return n;
}

#define Skip_Space(c)	do{for(;*c==' ' || *c=='\t'; c++) if(c == end) return size; }while(0)

static ssize_t keymap_store_map(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{ 
	struct mcu_data *mcu = dev_get_drvdata(dev);
	struct keymcu_platform_data* pd;
	unsigned int rawcode, keycode;
	const char *p, *end;
	p = buf;
	end = buf+size;

	pd = mcu->keymcu_data;

	Skip_Space(p);
	if(*p=='\n')	//skip a line
		return p-buf+1;
	
	rawcode = simple_strtoul(p, (char**)&p, 0);
	Skip_Space(p);
	if(*p!='=' || rawcode >= pd->keyMapTable_size){
		printk("Wrong key map format 1!\n");
		return size;
	}
	p++;
	Skip_Space(p);
	keycode = simple_strtoul(p, (char**)&p, 0);
	Skip_Space(p);
	if(*p!='\n' || keycode >= KEY_WIMAX){
		printk("Wrong key map format 2!\n");
		return size;
	}
	p++;

	pd->keyMapTable[rawcode] = keycode;

	if(keycode)
		set_bit(keycode, mcu->input->keybit);

	return p-buf;
}

static ssize_t keymap_show_clean(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Write \'clean\' for clean all key map table.\n");
}

static ssize_t keymap_store_clean(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct mcu_data *mcu = dev_get_drvdata(dev);

	if(strcmp(buf, "clean\n")==0 && mcu){
		struct keymcu_platform_data* pd = mcu->keymcu_data;
		clear_keybit(pd, mcu->input);
		memset(pd->keyMapTable, 0, pd->keyMapTable_size);
	}
	return size;
}
static DEVICE_ATTR(map, 0644,keymap_show_map, keymap_store_map);
static DEVICE_ATTR(clean, 0644,keymap_show_clean, keymap_store_clean);

/**********************mcu dout***************************/
static int get_mcu_dout(struct mcu_data *mcu, unsigned int n)
{
	int d;
	d = em6kstm32_spi_read_word(mcu->spi, CMD_DOUT);
	if(d<0){
		printk("%s: failed to get %d\n", __FUNCTION__, n);
		return 0;
	}

	return (d>>n)&1;
}

static int set_mcu_dout(struct mcu_data *mcu, unsigned int n, unsigned int v)
{
	int d;
	if(v)
		d = em6kstm32_spi_write_word(mcu->spi, CMD_SETB_DOUT, n);
	else
		d = em6kstm32_spi_write_word(mcu->spi, CMD_CLRB_DOUT, n);

	if(d<0){
		printk("%s: failed to set %d\n", __FUNCTION__, n);
		return d;
	}

	return 0;
}

#define DEFINE_MCU_DOUT(n)	static ssize_t mcu_show_dout##n(struct device *dev, struct device_attribute *attr, char *buf){\
		struct mcu_data *mcu = dev_get_drvdata(dev);\
		return sprintf(buf, "%d\n", get_mcu_dout(mcu,n));\
	}\
	static ssize_t mcu_store_dout##n(struct device *dev, struct device_attribute *attr,\
			      const char *buf, size_t size){\
		struct mcu_data *mcu = dev_get_drvdata(dev);\
		char *end;\
		unsigned long v = simple_strtoul(buf, &end, 0);\
		if (end == buf)			return -EINVAL;\
	set_mcu_dout(mcu, n, v);	return size;}\
	static DEVICE_ATTR(DOUT##n, 0644,mcu_show_dout##n, mcu_store_dout##n);


DEFINE_MCU_DOUT(0)
DEFINE_MCU_DOUT(1)

/**********************mcu din***************************/
	static int get_mcu_din(struct mcu_data *mcu, unsigned int n)
{
	int d;
	d = em6kstm32_spi_read_word(mcu->spi, CMD_DIN);
	if(d<0){
		printk("%s: failed to get %d\n", __FUNCTION__, n);
		return 0;
	}

	return (d>>n)&1;
}

#define DEFINE_MCU_DIN(n)	static ssize_t mcu_show_din##n(struct device *dev, struct device_attribute *attr, char *buf){\
			struct mcu_data *mcu = dev_get_drvdata(dev);\
			return sprintf(buf, "%d\n", get_mcu_din(mcu,n));\
		}\
	static DEVICE_ATTR(DIN##n, 0644,mcu_show_din##n, NULL);

DEFINE_MCU_DIN(0)
DEFINE_MCU_DIN(1)

/**************************mcu version************************************/
static int get_mcu_ver(struct mcu_data *mcu, unsigned int n)
{
	u8 d;
	d = em6kstm32_spi_read_byte(mcu->spi, CMD_TEST1);
	if(d<0){
		printk("%s: failed to get %d\n", __FUNCTION__, n);
		return 0;
	}

	return d;
}

#define DEFINE_MCU_VER(n)	static ssize_t mcu_show_ver(struct device *dev, struct device_attribute *attr, char *buf){\
			struct mcu_data *mcu = dev_get_drvdata(dev);\
			return sprintf(buf, "%d\n", get_mcu_ver(mcu,n));\
		}\
	static DEVICE_ATTR(ver, 0644,mcu_show_ver, NULL);

DEFINE_MCU_VER(0)

/**********************mcu pwm out***************************/
static int get_mcu_aout(struct mcu_data *mcu, unsigned int n)
{
	int d;
	d = em6kstm32_spi_read_word(mcu->spi, CMD_ANALOG_OUT(n));
	if(d<0){
		printk("%s: failed to get %d\n", __FUNCTION__, n);
		return 0;
	}

	return d;
}

static int set_mcu_aout(struct mcu_data *mcu, unsigned int n, unsigned int v)
{
	int ret;
	ret = em6kstm32_spi_write_word(mcu->spi, CMD_ANALOG_OUT(n), v);
	if(ret<0){
		printk("%s: failed to set %d\n", __FUNCTION__, n);
		return ret;
	}

	return 0;
}

#define DEFINE_MCU_AOUT(n, name)	static ssize_t mcu_show_aout##n(struct device *dev, struct device_attribute *attr, char *buf){\
			struct mcu_data *mcu = dev_get_drvdata(dev);\
			return sprintf(buf, "%d\n", get_mcu_aout(mcu,n));\
		}\
	static ssize_t mcu_store_aout##n(struct device *dev, struct device_attribute *attr,\
						      const char *buf, size_t size){\
				struct mcu_data *mcu = dev_get_drvdata(dev);\
				char *end;\
				unsigned long v = simple_strtoul(buf, &end, 0);\
				if (end == buf)			return -EINVAL;\
			set_mcu_aout(mcu, n, v);	return size;}\
	static DEVICE_ATTR(name, 0644,mcu_show_aout##n, mcu_store_aout##n);

DEFINE_MCU_AOUT(0, buzzer)

/**********************mcu analog input for PI, AI, RI***************************/
static int get_mcu_ain(struct mcu_data *mcu, unsigned int n)
{
	int d;
	d = em6kstm32_spi_read_word(mcu->spi, CMD_ANALOG_IN(n));
	if(d<0){
		printk("%s: failed to get %d\n", __FUNCTION__, n);
		return 0;
	}

	return d;
}

#define DEFINE_MCU_AIN(n, name)	static ssize_t mcu_show_ain##n(struct device *dev, struct device_attribute *attr, char *buf){\
		struct mcu_data *mcu = dev_get_drvdata(dev);\
		return sprintf(buf, "%d\n", get_mcu_ain(mcu,n));}\
		static DEVICE_ATTR(name, 0644,mcu_show_ain##n, NULL);

DEFINE_MCU_AIN(0, power)
DEFINE_MCU_AIN(1, RI0)
DEFINE_MCU_AIN(2, RI1)

static const struct attribute *mcu_attrs[] = {
	&dev_attr_map.attr,
	&dev_attr_clean.attr,

	&dev_attr_DOUT0.attr,
	&dev_attr_DOUT1.attr,

	&dev_attr_DIN0.attr,
	&dev_attr_DIN1.attr,
	
	&dev_attr_ver.attr,
	&dev_attr_buzzer.attr,

	&dev_attr_power.attr,
	&dev_attr_RI0.attr,
	&dev_attr_RI1.attr,
	NULL, 
};

static const struct attribute_group mcu_attr_group = { 
	.name = "mcu",		/* put in device directory */
	.attrs = (struct attribute **) mcu_attrs
}; 

static irqreturn_t mcu_irq(int irq, void *handle)
{
	struct mcu_data *mcu = handle;
	s32 key;
	unsigned int code;

	while((key = em6kstm32_spi_read_byte(mcu->spi, CMD_DIN_KEY))!=0xff){
		if(key<0){
			printk("Failed to get key code\n");
			break;
		}

		code = key&(~MCUKEY_DOWN);

		if(!mcu->keymcu_data || (mcu->keymcu_data->keyMapTable_size == 0)){
			printk("Keycode: 0x%x[%s]\n", code, (key&MCUKEY_DOWN)?"down":"up");
		}
		else{
			unsigned short *keyMapTable = mcu->keymcu_data->keyMapTable;
			unsigned int keyMapTable_size = mcu->keymcu_data->keyMapTable_size;

			if(code < keyMapTable_size){
				if(keyMapTable[code]==0)
					printk("Keycode: 0x%x[%s]\n", code, (key&MCUKEY_DOWN)?"down":"up");
				else{
					input_event(mcu->input, EV_MSC, MSC_SCAN, code);
					input_report_key(mcu->input, keyMapTable[code], key&MCUKEY_DOWN);
					input_sync(mcu->input);
				}
			}
			else
				printk("Wrong key scancode 0x%x[%s]\n\n", code, (key&MCUKEY_DOWN)?"down":"up");
		}
	}

	return IRQ_HANDLED;
}

static void mcu_work(struct work_struct *work)
{
	struct mcu_data *mcu =
		container_of(to_delayed_work(work), struct mcu_data, work);
	s32 key;
	unsigned int code;

	while((key = em6kstm32_spi_read_byte(mcu->spi, CMD_DIN_KEY))!=0xff){
		if(key<0){
			printk("Failed to get key code\n");
			goto out;
		}

		code = key&(~MCUKEY_DOWN);

		if(!mcu->keymcu_data || (mcu->keymcu_data->keyMapTable_size == 0)){
			printk("Keycode: 0x%x[%s]\n", code, (key&MCUKEY_DOWN)?"down":"up");
		}
		else{
			unsigned short *keyMapTable = mcu->keymcu_data->keyMapTable;
			unsigned int keyMapTable_size = mcu->keymcu_data->keyMapTable_size;

			if(code < keyMapTable_size){
				if(keyMapTable[code]==0)
					printk("Keycode: 0x%x[%s]\n", code, (key&MCUKEY_DOWN)?"down":"up");
				else
					input_report_key(mcu->input, keyMapTable[code], key&MCUKEY_DOWN);
			}
			else
				printk("Wrong key scancode 0x%x[%s]\n\n", code, (key&MCUKEY_DOWN)?"down":"up");
		}
	}

out:
	enable_irq(mcu->irq);
}

static void mcu_key_deinit(struct spi_device *spi)
{
	struct mcu_data *mcu = spi_get_drvdata(spi);

	input_unregister_device(mcu->input);
	input_free_device(mcu->input);
	free_irq(mcu->irq, mcu);
}

static int mcu_key_init(struct spi_device *spi, struct mcu_data *mcu)
{
	int err;
	struct input_dev *input_dev;
	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	mcu->input = input_dev;
	INIT_DELAYED_WORK(&mcu->work, mcu_work);

	input_dev->name = "MCU Keypad";
	input_dev->id.bustype = BUS_SPI;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	set_keybit(mcu->keymcu_data, input_dev);

	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	input_dev->keycode = mcu->keymcu_data->keyMapTable;
	input_dev->keycodesize = sizeof(mcu->keymcu_data->keyMapTable[0]);
	input_dev->keycodemax = mcu->keymcu_data->keyMapTable_size;

	printk("spi request irq %d\n", mcu->irq);
	err = request_threaded_irq(mcu->irq, NULL, mcu_irq, IRQF_TRIGGER_HIGH|IRQF_ONESHOT,
			spi->dev.driver->name, mcu);
	if (err < 0) {
		dev_err(&spi->dev, "irq %d busy?\n", mcu->irq);
		goto err_free_mem;
	}

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(mcu->irq, mcu);
err_free_mem:
	input_free_device(input_dev);
	return err;

}

static int __devinit em6kstm32_spi_probe(struct spi_device *spi)
{
	static unsigned short keyMapTable[128]={KEY_F1};	//must have one key for VT(keyboard.c)
	static struct keymcu_platform_data keymcu_data= {
		.keyMapTable = keyMapTable,
		.keyMapTable_size = ARRAY_SIZE(keyMapTable),
	};

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
	mcudata->irq = spi->irq;
	mcudata->keymcu_data = &keymcu_data;

	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err) {
		dev_dbg(&spi->dev, "spi master setup failed\n");
		kfree(mcudata);
		return err;
	}

	err = em6kstm32_spi_read_byte(spi, CMD_CHIPID);
	if(err!=CHIP_ID){
		dev_dbg(&spi->dev, "wrong stm32 spi chip id 0x%x\n", err);
		kfree(mcudata);
		return -1;
	}

	err = mcu_key_init(spi, mcudata);
	if(err<0){
		kfree(mcudata);
		return err;
	}

	sysfs_create_group(&spi->dev.kobj, &mcu_attr_group);

	spi_set_drvdata(spi, mcudata);

	return 0;
}

static int __devexit em6kstm32_spi_remove(struct spi_device *spi)
{
	struct mcu_data *mcu = spi_get_drvdata(spi);

	mcu_key_deinit(spi);
	sysfs_remove_group(&spi->dev.kobj, &mcu_attr_group);

	spi_set_drvdata(spi, NULL);
	kfree(mcu);
	return 0;
}

static struct spi_driver em6kstm32_spi_driver = {
	.driver = {
		.name	= "em6057e-stm32",
		.owner	= THIS_MODULE,
	},
	.probe		= em6kstm32_spi_probe,
	.remove		= __devexit_p(em6kstm32_spi_remove),
};

module_spi_driver(em6kstm32_spi_driver);
MODULE_AUTHOR("threewater <threewaterl@163.com>");
MODULE_LICENSE("GPL v2");
