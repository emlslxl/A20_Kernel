/*
 * RC522 keyboard drivers (SPI bus)
 *
 * Copyright (C) 2013-2015 threewater<threewaterl@163.com>
 *
 * Licensed under the GPL-2 or later.
 */
#define DEBUG
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <asm/uaccess.h>

struct rc522_data{
	struct spi_device* spi;
	struct delayed_work	work;
	u32 card_id;
	int valid;
	wait_queue_head_t wq;
	spinlock_t lock;
};

static struct rc522_data g_rcdata;

#define RC522_POLL_TIME		200	//ms

/////////////////////////////////////////////////////////////////////
//MF522 command
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE			  0x00				 //取消当前命令
#define PCD_AUTHENT 		  0x0E				 //验证密钥
#define PCD_RECEIVE 		  0x08				 //接收数据
#define PCD_TRANSMIT		  0x04				 //发送数据
#define PCD_TRANSCEIVE		  0x0C				 //发送并接收数据
#define PCD_RESETPHASE		  0x0F				 //复位
#define PCD_CALCCRC 		  0x03				 //CRC计算

/////////////////////////////////////////////////////////////////////
//Mifare_One card command
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL 		  0x26				 //寻天线区内未进入休眠状态
#define PICC_REQALL 		  0x52				 //寻天线区内全部卡
#define PICC_ANTICOLL1		  0x93				 //防冲撞
#define PICC_ANTICOLL2		  0x95				 //防冲撞
#define PICC_AUTHENT1A		  0x60				 //验证A密钥
#define PICC_AUTHENT1B		  0x61				 //验证B密钥
#define PICC_READ			  0x30				 //读块
#define PICC_WRITE			  0xA0				 //写块
#define PICC_DECREMENT		  0xC0				 //扣款
#define PICC_INCREMENT		  0xC1				 //充值
#define PICC_RESTORE		  0xC2				 //调块数据到缓冲区
#define PICC_TRANSFER		  0xB0				 //保存缓冲区中数据
#define PICC_HALT			  0x50				 //休眠

/////////////////////////////////////////////////////////////////////
//MF522 FIFO
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH 	  64				 //FIFO size=64byte
#define MAXRLEN  18

/////////////////////////////////////////////////////////////////////
//MF522 register
/////////////////////////////////////////////////////////////////////
// PAGE 0
#define 	RFU00				  0x00	  
#define 	CommandReg			  0x01	  
#define 	ComIEnReg			  0x02	  
#define 	DivlEnReg			  0x03	  
#define 	ComIrqReg			  0x04	  
#define 	DivIrqReg			  0x05
#define 	ErrorReg			  0x06	  
#define 	Status1Reg			  0x07	  
#define 	Status2Reg			  0x08	  
#define 	FIFODataReg 		  0x09
#define 	FIFOLevelReg		  0x0A
#define 	WaterLevelReg		  0x0B
#define 	ControlReg			  0x0C
#define 	BitFramingReg		  0x0D
#define 	CollReg 			  0x0E
#define 	RFU0F				  0x0F
// PAGE 1	  
#define 	RFU10				  0x10
#define 	ModeReg 			  0x11
#define 	TxModeReg			  0x12
#define 	RxModeReg			  0x13
#define 	TxControlReg		  0x14
#define 	TxAutoReg			  0x15
#define 	TxSelReg			  0x16
#define 	RxSelReg			  0x17
#define 	RxThresholdReg		  0x18
#define 	DemodReg			  0x19
#define 	RFU1A				  0x1A
#define 	RFU1B				  0x1B
#define 	MifareReg			  0x1C
#define 	RFU1D				  0x1D
#define 	RFU1E				  0x1E
#define 	SerialSpeedReg		  0x1F
// PAGE 2	 
#define 	RFU20				  0x20	
#define 	CRCResultRegM		  0x21
#define 	CRCResultRegL		  0x22
#define 	RFU23				  0x23
#define 	ModWidthReg 		  0x24
#define 	RFU25				  0x25
#define 	RFCfgReg			  0x26
#define 	GsNReg				  0x27
#define 	CWGsCfgReg			  0x28
#define 	ModGsCfgReg 		  0x29
#define 	TModeReg			  0x2A
#define 	TPrescalerReg		  0x2B
#define 	TReloadRegH 		  0x2C
#define 	TReloadRegL 		  0x2D
#define 	TCounterValueRegH	  0x2E
#define 	TCounterValueRegL	  0x2F
// PAGE 3	   
#define 	RFU30				  0x30
#define 	TestSel1Reg 		  0x31
#define 	TestSel2Reg 		  0x32
#define 	TestPinEnReg		  0x33
#define 	TestPinValueReg 	  0x34
#define 	TestBusReg			  0x35
#define 	AutoTestReg 		  0x36
#define 	VersionReg			  0x37
#define 	AnalogTestReg		  0x38
#define 	TestDAC1Reg 		  0x39	
#define 	TestDAC2Reg 		  0x3A	 
#define 	TestADCReg			  0x3B	 
#define 	RFU3C				  0x3C	 
#define 	RFU3D				  0x3D	 
#define 	RFU3E				  0x3E	 
#define 	RFU3F				  0x3F

/**********************rc522 spi access ***************************/
static int rc522_spi_xfer(struct spi_device *spi,
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

static int rc522_read(struct spi_device *spi, u8 address)
{
	u8 ret;

	return rc522_spi_xfer(spi, (address<<1)|0x80, 1, NULL, &ret) ? : ret;
}

static int rc522_write(struct spi_device *spi, u8 address, u8 val)
{
	return rc522_spi_xfer(spi, address<<1, 1, &val, NULL);
}

/////////////////////////////////////////////////////////////////////
//功	能：置RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//			mask[IN]:置位值
/////////////////////////////////////////////////////////////////////
static void SetBitMask(struct spi_device *spi, u8 reg, u8 mask)
{
	char   tmp = 0x0;
	tmp = rc522_read(spi, reg);
	rc522_write(spi, reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//功	能：清RC522寄存器位
//参数说明：reg[IN]:寄存器地址
//			mask[IN]:清位值
/////////////////////////////////////////////////////////////////////
static void ClearBitMask(struct spi_device *spi, u8 reg, u8 mask)
{
	char   tmp = 0x0;
	tmp = rc522_read(spi, reg);
	rc522_write(spi, reg, tmp & ~mask);	// clear bit mask
} 

/**********************rc522 function ***************************/
static int rc522_Reset(struct spi_device *spi)
{
	rc522_write(spi, CommandReg,PCD_RESETPHASE);
	rc522_write(spi, CommandReg,PCD_RESETPHASE);
	udelay(10);
	
	rc522_write(spi, ModeReg,0x3D);			  //和Mifare卡通讯，CRC初始值0x6363
	rc522_write(spi, TReloadRegL,30);		   
	rc522_write(spi, TReloadRegH,0);
	rc522_write(spi, TModeReg,0x8D);
	rc522_write(spi, TPrescalerReg,0x3E);
	
	rc522_write(spi, TxAutoReg,0x40);//必须要

	return 0;
}

/////////////////////////////////////////////////////////////////////
//开启天线	
//每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
static void rc522_AntennaOn(struct spi_device *spi)
{
	u8 i;
	i = rc522_read(spi, TxControlReg);
	if (!(i & 0x03))
		SetBitMask(spi, TxControlReg, 0x03);
}

/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
static void rc522_AntennaOff(struct spi_device *spi)
{
	ClearBitMask(spi, TxControlReg, 0x03);
}

static void rc522_setISO(struct spi_device *spi)
{
   ClearBitMask(spi, Status2Reg,0x08);
   rc522_write(spi, ModeReg,0x3D);//3F
   rc522_write(spi, RxSelReg,0x86);//84
   rc522_write(spi, RFCfgReg,0x7F);	 //4F
   rc522_write(spi, TReloadRegL,30);//tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
   rc522_write(spi, TReloadRegH,0);
   rc522_write(spi, TModeReg,0x8D);
   rc522_write(spi, TPrescalerReg,0x3E);
   udelay(10);
   rc522_AntennaOn(spi);
}

/////////////////////////////////////////////////////////////////////
//功	能：通过RC522和ISO14443卡通讯
//参数说明：Command[IN]:RC522命令字
//			pIn [IN]:通过RC522发送到卡片的数据
//			InLenByte[IN]:发送数据的字节长度
//			pOut [OUT]:接收到的卡片返回数据
//			*pOutLenBit[OUT]:返回数据的位长度
/////////////////////////////////////////////////////////////////////
static int PcdComMF522(struct spi_device *spi, u8 Command, u8 *pIn, 
			u8 InLenByte, u8 *pOut, u8 *pOutLenBit)
{
	int status = -1;
	u8 irqEn   = 0x00;
	u8 waitFor = 0x00;
	u8 lastBits;
	u8 n;
	u32 i;
	switch (Command)
	{
		case PCD_AUTHENT:
			irqEn	= 0x12;
			waitFor = 0x10;
			break;
		case PCD_TRANSCEIVE:
			irqEn	= 0x77;
			waitFor = 0x30;
			break;
		default:
			break;
	}
   
	rc522_write(spi, ComIEnReg,irqEn|0x80);
	ClearBitMask(spi, ComIrqReg,0x80);	//清所有中断位
	rc522_write(spi, CommandReg,PCD_IDLE);
	SetBitMask(spi, FIFOLevelReg,0x80);		//清FIFO缓存
	
	for (i=0; i<InLenByte; i++)
		rc522_write(spi, FIFODataReg, pIn [i]);

	rc522_write(spi, CommandReg, Command);
//		 n = ReadRawRC(CommandReg);
	
	if (Command == PCD_TRANSCEIVE)
		SetBitMask(spi, BitFramingReg,0x80);	//开始传送

	//i = 600;//根据时钟频率调整，操作M1卡最大等待时间25ms
	i = 100000;
	do{
		n = rc522_read(spi, ComIrqReg);
		i--;
	}
	while ((i!=0) && !(n&0x01) && !(n&waitFor));
	ClearBitMask(spi, BitFramingReg,0x80);

	if (i!=0){
		if(!(rc522_read(spi, ErrorReg)&0x1B)){
			status = 0;
			if (n & irqEn & 0x01)
			   status = 1;
			if (Command == PCD_TRANSCEIVE){
				n = rc522_read(spi, FIFOLevelReg);
				lastBits = rc522_read(spi, ControlReg) & 0x07;
				if (lastBits)
					*pOutLenBit = (n-1)*8 + lastBits;
				else
					*pOutLenBit = n*8;
				if (n == 0)
					n = 1;
				if (n > MAXRLEN)
					n = MAXRLEN;
				for (i=0; i<n; i++)
					pOut [i] = rc522_read(spi, FIFODataReg);
			}
		}
		else
			status = -1;
	}

	SetBitMask(spi, ControlReg,0x80);		   // stop timer now
	rc522_write(spi, CommandReg,PCD_IDLE); 
	return status;
}

/////////////////////////////////////////////////////////////////////
//功	能：寻卡
//参数说明: req_code[IN]:寻卡方式
//				  0x52 = 寻感应区内所有符合14443A标准的卡
//				  0x26 = 寻未进入休眠状态的卡
//			pTagType[OUT]：卡片类型代码
//				  0x4400 = Mifare_UltraLight
//				  0x0400 = Mifare_One(S50)
//				  0x0200 = Mifare_One(S70)
//				  0x0800 = Mifare_Pro(X)
//				  0x4403 = Mifare_DESFire
/////////////////////////////////////////////////////////////////////
static int rc522_Request(struct spi_device *spi, u8 req_code,u8 *pTagType)
{
	int status;  
	u8 unLen;
	u8 ucComMF522Buf[MAXRLEN]; 

	ClearBitMask(spi, Status2Reg,0x08);
	rc522_write(spi, BitFramingReg,0x07);
	SetBitMask(spi, TxControlReg,0x03);
 
	ucComMF522Buf[0] = req_code;

	status = PcdComMF522(spi, PCD_TRANSCEIVE,ucComMF522Buf,1,ucComMF522Buf,&unLen);

	if ((status == 0) && (unLen == 0x10)){	 
		*pTagType	  = ucComMF522Buf[0];
		*(pTagType+1) = ucComMF522Buf[1];
		return 0;
	}

	return -1;
}

/////////////////////////////////////////////////////////////////////
//功	能：防冲撞
//参数说明: pSnr[OUT]:卡片序列号，4字节
/////////////////////////////////////////////////////////////////////  
static int PcdAnticoll(struct spi_device *spi, u8 *pSnr)
{
	int   status;
	u8 i,snr_check=0;
	u8 unLen;
	u8 ucComMF522Buf[MAXRLEN]; 
	

	ClearBitMask(spi, Status2Reg,0x08);
	rc522_write(spi, BitFramingReg,0x00);
	ClearBitMask(spi, CollReg,0x80);
 
	ucComMF522Buf[0] = PICC_ANTICOLL1;
	ucComMF522Buf[1] = 0x20;

	status = PcdComMF522(spi, PCD_TRANSCEIVE,ucComMF522Buf,2,ucComMF522Buf,&unLen);

	if(status == 0){
		for (i=0; i<4; i++){
			*(pSnr+i)	= ucComMF522Buf[i];
			snr_check ^= ucComMF522Buf[i];
		}
		if (snr_check != ucComMF522Buf[i])
			status = -1;
	}
	
	SetBitMask(spi, CollReg,0x80);
	return status;
}

/**********************rc522 operation function ***************************/
static ssize_t rc522fd_read (struct file *file, 
		char __user *buf, size_t count, loff_t *offset)
{
	char tmp[32];

retry:
	spin_lock(&g_rcdata.lock);

	if(!g_rcdata.valid){
		spin_unlock(&g_rcdata.lock);
		interruptible_sleep_on(&g_rcdata.wq);
		if (signal_pending(current))
			return -ERESTARTSYS;
		goto retry;
	}

	count = snprintf(tmp, min(sizeof(tmp), count), "%x\n", g_rcdata.card_id);

	copy_to_user(buf, tmp, count);
	g_rcdata.valid=0;
	spin_unlock(&g_rcdata.lock);

	return count;
}

static void rc522_poll_work(struct work_struct *work)
{
	struct rc522_data *rcdata = container_of(work,
		struct rc522_data, work.work);

	struct spi_device* spi=rcdata->spi;
	int ret;
	u8 CT[2];//卡类型
	u32 cardid; //卡号

	ret = rc522_Request(spi, PICC_REQALL,CT);/*扫描卡*/
	ret = PcdAnticoll(spi, (u8*)&cardid);/*防冲撞*/

	if(ret==0){
		spin_lock(&rcdata->lock);
		rcdata->valid=1;
		rcdata->card_id=cardid;
		spin_unlock(&rcdata->lock);
		wake_up_interruptible(&rcdata->wq);
	}

	schedule_delayed_work(&rcdata->work, msecs_to_jiffies(RC522_POLL_TIME));
}

static int rc522fd_open(struct inode *inode, struct file *file)
{
	g_rcdata.valid = 0;
	schedule_delayed_work(&g_rcdata.work, 0);
//	file->private_data = &g_rcdata;
	return 0;
}

static int rc522fd_release(struct inode *inode, struct file *file)
{
	cancel_delayed_work(&g_rcdata.work);
	return 0;
}


static const struct file_operations rc522fd_fops = {
	.read		= rc522fd_read,
	.open		= rc522fd_open,
	.release	= rc522fd_release,
};

//Initialize driver.
static struct miscdevice misc_rc522 = {
	.minor = MISC_DYNAMIC_MINOR, 
	.name = "rfid", 
	.fops = &rc522fd_fops,
};

static int rc522_init(struct spi_device *spi, struct rc522_data *rcdata)
{
	int err;

	rc522_Reset(spi);
	rc522_AntennaOff(spi);
	rc522_AntennaOn(spi);
	rc522_setISO(spi);

	INIT_DELAYED_WORK(&g_rcdata.work, rc522_poll_work);
	init_waitqueue_head(&g_rcdata.wq);
	spin_lock_init(&g_rcdata.lock);

	err = misc_register(&misc_rc522);
	if(err)
		return err;

	return err;
}

#define MAX_SPI_FREQ_HZ		2000000

static int __devinit rc522_spi_probe(struct spi_device *spi)
{
	int err;

	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
		return -EINVAL;
	}

	g_rcdata.spi = spi;

	spi->bits_per_word = 8;
	err = spi_setup(spi);
	if (err) {
		dev_dbg(&spi->dev, "spi master setup failed\n");
		return err;
	}

	err=rc522_init(spi, &g_rcdata);
	if(err)
		return err;
	
	spi_set_drvdata(spi, &g_rcdata);

	return 0;
}

static int __devexit rc522_spi_remove(struct spi_device *spi)
{
//	struct rc522_data *rcdata = spi_get_drvdata(spi);

	spi_set_drvdata(spi, NULL);
	return 0;
}

static struct spi_driver rc522_spi_driver = {
	.driver = {
		.name	= "rc522",
		.owner	= THIS_MODULE,
	},
	.probe		= rc522_spi_probe,
	.remove		= __devexit_p(rc522_spi_remove),
};

module_spi_driver(rc522_spi_driver);
MODULE_AUTHOR("threewater <threewaterl@163.com>");
MODULE_LICENSE("GPL v2");
