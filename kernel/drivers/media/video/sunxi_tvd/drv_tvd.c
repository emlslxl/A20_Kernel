#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <linux/delay.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
#include <linux/freezer.h>
#endif

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-dma-contig.h>
#include <linux/moduleparam.h>

//#include <mach/gpio_v2.h>
//#include <mach/script_v2.h>
#include <plat/sys_config.h>
#include <mach/clock.h>
#include <mach/irqs.h>
#include <linux/regulator/consumer.h>

#include <linux/types.h>
#include <media/videobuf-core.h>

#include "bsp_tvd.h"

#define DBG_EN 0	
#if(DBG_EN == 1)		
	#define __dbg(x, arg...) printk("[TVD_DBG]"x, ##arg)
#else
	#define __dbg(x, arg...) 
#endif
#define __err(x, arg...) printk(KERN_INFO"[TVD_ERR]"x, ##arg)
#define __inf(x, arg...) printk(KERN_INFO"[TVD_INF]"x, ##arg)

#define TVD_REGS_BASE   0x01c08000
#define TVD_REG_SIZE    0x1000

#define TVD_MAJOR_VERSION 1
#define TVD_MINOR_VERSION 0
#define TVD_RELEASE 0
#define TVD_VERSION KERNEL_VERSION(TVD_MAJOR_VERSION, TVD_MINOR_VERSION, TVD_RELEASE)
#define TVD_MODULE_NAME "tvd"

#define NUM_INPUTS 1

#define MIN_WIDTH  (32)
#define MIN_HEIGHT (32)
#define MAX_WIDTH  (4096)
#define MAX_HEIGHT (4096)
#define MAX_BUFFER (32*1024*1024)

//#define USE_DMA_CONTIG

#define BLINK_DELAY   HZ/2

static unsigned video_nr = 1;
static unsigned first_flag = 0;
//static struct timer_list timer;

struct fmt {
	u8					name[32];
	u32   				fourcc;          /* v4l2 format id */
	tvd_fmt_t 	        output_fmt;	
	int   				depth;
	u32                 width;
	u32                 height;
};

static struct fmt tvd_fmt = {
		.name     		= "planar YUV420",
		.fourcc   		= V4L2_PIX_FMT_NV12,
		.output_fmt		= TVD_PL_YUV420,
		.depth    		= 12,
		.width          = 720,//may be change
		.height         = 480,//may be change
};

//static struct fmt formats[] = {
//	{
//		.name     		= "planar YUV420",
//		.fourcc   		= V4L2_PIX_FMT_NV12,
//		.output_fmt		= TVD_PL_YUV420,
//		.depth    		= 12,
//		.width          = 720,
//		.height         = 480,
//	},
//	{
//		.name     		= "planar YUV420",
//		.fourcc   		= V4L2_PIX_FMT_NV12,
//		.output_fmt		= TVD_PL_YUV420,
//		.depth    		= 12,
//		.width          = 720,
//		.height         = 960,
//	},
//	{
//		.name     		= "planar YUV420",
//		.fourcc   		= V4L2_PIX_FMT_NV12,
//		.output_fmt		= TVD_PL_YUV420,
//		.depth    		= 12,
//		.width          = 720,
//		.height         = 1920,
//	},
//	{
//		.name     		= "planar YUV420",
//		.fourcc   		= V4L2_PIX_FMT_NV12,
//		.output_fmt		= TVD_PL_YUV420,
//		.depth    		= 12,
//		.width          = 1440,
//		.height         = 480,
//	},
//	{
//		.name     		= "planar YUV420",
//		.fourcc   		= V4L2_PIX_FMT_NV12,
//		.output_fmt		= TVD_PL_YUV420,
//		.depth    		= 12,
//		.width          = 2880,
//		.height         = 480,
//	},
//	{
//		.name     		= "planar YUV420",
//		.fourcc   		= V4L2_PIX_FMT_NV12,
//		.output_fmt		= TVD_PL_YUV420,
//		.depth    		= 12,
//		.width          = 1440,
//		.height         = 960,
//	},
//	{
//		.name     		= "planar YUV420",
//		.fourcc   		= V4L2_PIX_FMT_NV12,
//		.output_fmt		= TVD_PL_YUV420,
//		.depth    		= 12,
//		.width          = 720,
//		.height         = 576,
//	},
//};

/* buffer for one video frame */
struct buffer {
	struct videobuf_buffer vb;
	struct fmt        *fmt;
};

struct dmaqueue {
	struct list_head active;
	
	/* Counters to control fps rate */
	int frame;
	int ini_jiffies;
};

struct buf_addr {
	dma_addr_t	y;
	dma_addr_t	c;
};

static LIST_HEAD(devlist);

struct dev {
	struct list_head       	devlist;
	struct v4l2_device 	   	v4l2_dev;
	struct v4l2_subdev		*sd;
	struct platform_device	*pdev;

	int						id;
	
	spinlock_t              slock;

	/* various device info */
	struct video_device     *vfd;

	struct dmaqueue     vidq;

	/* Several counters */
	unsigned 		   		ms;
	unsigned long           jiffies;

	/* Input Number */
	int			   			input;

	/* video capture */
	struct fmt          *fmt;
	unsigned int            width;
	unsigned int            height;
	unsigned int			frame_size;
	struct videobuf_queue   vb_vidq;

	/*  */
	unsigned int interface;
	unsigned int system;
	unsigned int format;
	unsigned int row;
	unsigned int column;
	//unsigned int channel_en[4];
	unsigned int channel_index[4];	
	unsigned int channel_offset_y[4];
	unsigned int channel_offset_c[4];
	unsigned int channel_irq;

	/*working state*/
	unsigned long 		   	generating;
	int						opened;

	//clock
	struct clk				*ahb_clk;
#ifdef CONFIG_ARCH_SUN7I
	struct clk				*module1_clk;
	struct clk				*module2_clk;
#else
	struct clk				*module_clk;
#endif
	struct clk				*dram_clk;

	int						irq;
	void __iomem			*regs;
	struct resource			*regs_res; //??
	
	struct buf_addr		buf_addr;
};


static int is_generating(struct dev *dev)
{
	return test_bit(0, &dev->generating);
}

static void start_generating(struct dev *dev)
{
	 set_bit(0, &dev->generating);
	 return;
}	

static void stop_generating(struct dev *dev)
{
	 first_flag = 0;
	 clear_bit(0, &dev->generating);
	 return;
}

/*static struct fmt * get_format(struct v4l2_format *f)
{
	struct fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat && \
			fmt->width == f->fmt.pix.width && \
			fmt->height == f->fmt.pix.height ) {
			__dbg("fmt->fourcc = %d, fmt->width=%d, fmt->height = %d\n", fmt->fourcc, fmt->width, fmt->height); 	
			break;
		}
	}

	if (k == ARRAY_SIZE(formats)) {
		return NULL;
	}	

	return &formats[k];
};*/

static inline void set_addr(struct dev *dev,struct buffer *buffer)
{	
	int i;
	struct buffer *buf = buffer;	
	dma_addr_t addr_org;
	
	__dbg("buf ptr=%p\n",buf);

	addr_org = videobuf_to_dma_contig((struct videobuf_buffer *)buf);

	dev->buf_addr.y = addr_org;
	dev->buf_addr.c = addr_org + dev->width*dev->height;
	
//	__inf("dev->buf_addr.y=0x%08x\n", dev->buf_addr.y);

	for(i=0;i<4;i++){
		if(dev->channel_index[i]){			
			TVD_set_addr_y(i,dev->buf_addr.y + dev->channel_offset_y[i]);
			TVD_set_addr_c(i,dev->buf_addr.c + dev->channel_offset_c[i]);
		}			
	}

	__dbg("buf_addr_y=%x\n",  dev->buf_addr.y);
	__dbg("buf_addr_cb=%x\n", dev->buf_addr.c);
}

//static void timer_func(unsigned long ptr)
//{
//	__dbg("timer\n");
	
//	if(!TVD_det_finish()){
//		timer.expires = jiffies + BLINK_DELAY;
//		add_timer(&timer);
//	}
//	else{
//		__dbg("det finish\n");
//		TVD_det_disable();
//		TVD_set_mode(TVD_det_mode());
//	}
//}
//////////////////////////////////////////////////////////////////////////////////////////
static irqreturn_t TVD_isr(int irq, void *priv)
{
	struct buffer *buf;	
	struct dev *dev = (struct dev *)priv;
	struct dmaqueue *dma_q = &dev->vidq;

	spin_lock(&dev->slock);

	//__dbg("irq status=%x\n", TVD_irq_status_get(dev->channel_irq, TVD_FRAME_DONE));
	//__dbg("module_clk=%x reg=%x\n", clk_get_rate(dev->module1_clk), readl(0xf1c20000 + 0x128));

//	if(TVD_irq_status_get(TVD_FRAME_DONE)){
//	}
//	else if(TVD_irq_status_get(TVD_LOCK)){
//		__dbg("lock\n");
//		TVD_irq_status_clear(TVD_LOCK);
//		TVD_irq_enable(TVD_FRAME_DONE);
//		//start capture??
//	}
//	else if(TVD_irq_status_get(TVD_UNLOCK)){
//		__dbg("unlock\n");
//		TVD_irq_status_clear(TVD_UNLOCK);
//		TVD_det_enable();
//		timer.expires = jiffies + BLINK_DELAY;
//		add_timer(&timer);
//	}
	
	if (first_flag == 0) {
		first_flag=1;
		goto set_next_addr;
	}
	
	if (list_empty(&dma_q->active)) {		
		__err("No active queue to serve\n");		
		goto unlock;	
	}
	
	buf = list_entry(dma_q->active.next,struct buffer, vb.queue);
	__dbg("buf ptr=%p\n",buf);

	/* Nobody is waiting on this buffer*/	

	if (!waitqueue_active(&buf->vb.done)) {
		__dbg(" Nobody is waiting on this buffer,buf = 0x%p\n",buf);					
	}
	
	list_del(&buf->vb.queue);

	do_gettimeofday(&buf->vb.ts);
	buf->vb.field_count++;

	dev->ms += jiffies_to_msecs(jiffies - dev->jiffies);
	dev->jiffies = jiffies;

	buf->vb.state = VIDEOBUF_DONE;
	wake_up(&buf->vb.done);
	
	//judge if the frame queue has been written to the last
	if (list_empty(&dma_q->active)) {		
		__dbg("No more free frame\n");		
		goto unlock;	
	}
	
	if ((&dma_q->active) == dma_q->active.next->next) {
		__dbg("No more free frame on next time\n");		
		goto unlock;	
	}
		
set_next_addr:	
	buf = list_entry(dma_q->active.next->next,struct buffer, vb.queue);
	set_addr(dev,buf);

unlock:
	spin_unlock(&dev->slock);
	
	TVD_irq_status_clear(dev->channel_irq, TVD_FRAME_DONE);
	//__dbg("irq status=%x\n", TVD_irq_status_get(dev->channel_irq, TVD_FRAME_DONE));
	
	return IRQ_HANDLED;
}

static int TVD_clk_init(struct dev *dev)
{
	int ret;
	struct clk				*module_clk_src;

	dev->ahb_clk=clk_get(NULL, "ahb_tvd");
	if (IS_ERR_OR_NULL(dev->ahb_clk)) {
       	__err("get ahb clk error!\n");	
		return -1;
    }

#ifdef CONFIG_ARCH_SUN7I
	dev->module1_clk=clk_get(NULL,"tvdmod1");
	if(IS_ERR_OR_NULL(dev->module1_clk)) {
       	__err("get module1 clock error!\n");	
		return -1;
    }

	dev->module2_clk=clk_get(NULL,"tvdmod2");
	if(IS_ERR_OR_NULL(dev->module2_clk)) {
       	__err("get module2 clock error!\n");	
		return -1;
    }
#else
	dev->module_clk=clk_get(NULL,"tvd");
	if(IS_ERR_OR_NULL(dev->module_clk)) {
       	__err("get module clock error!\n");	
		return -1;
    }
#endif
    
	dev->dram_clk = clk_get(NULL, "sdram_tvd");
	if (IS_ERR_OR_NULL(dev->dram_clk)) {
       	__err("get dram clock error!\n");
		return -1;
    }
    
	module_clk_src=clk_get(NULL,"video_pll0"); //can select video_pll0 or video_pll1
	if (IS_ERR_OR_NULL(module_clk_src)) {
   		__err("get clock source error!\n");	
		return -1;
    }  
	ret = clk_set_rate(module_clk_src, 297000000);
	if (ret<0) {
        __err("set parent clock error!\n");
	    return -1;
    }     
#ifdef CONFIG_ARCH_SUN7I
	ret = clk_set_parent(dev->module1_clk, module_clk_src);
	ret = clk_set_parent(dev->module2_clk, module_clk_src);
#else
	ret = clk_set_parent(dev->module_clk, module_clk_src);
#endif
	if (ret<0) {
        __err("set parent clock error!\n");
	    return -1;
    }     
	clk_put(module_clk_src); //use ok
#ifdef CONFIG_ARCH_SUN7I
	ret = clk_set_rate(dev->module1_clk, 27000000);
	if (ret<0) {
        __err("set module clock rate error!\n");
	    return -1;
    }
#endif
	return 0;
}

static int TVD_clk_exit(struct dev *dev)
{
	clk_put(dev->ahb_clk);        
	dev->ahb_clk = NULL;
#ifdef CONFIG_ARCH_SUN7I
	clk_put(dev->module1_clk);        
	dev->module1_clk = NULL;
	clk_put(dev->module2_clk);        
	dev->module2_clk = NULL;
#else
	clk_put(dev->module_clk);        
	dev->module_clk = NULL;
#endif
	clk_put(dev->dram_clk);        
	dev->dram_clk = NULL;

	return 0;
}

static int TVD_clk_open(struct dev *dev)
{
	clk_enable(dev->dram_clk);
#ifdef CONFIG_ARCH_SUN7I
	clk_enable(dev->module1_clk);
	clk_enable(dev->module2_clk);
#else
	clk_enable(dev->module_clk);
#endif
	clk_enable(dev->ahb_clk);
	
	return 0;
}

static int TVD_clk_close(struct dev *dev)
{
	clk_disable(dev->ahb_clk);
#ifdef CONFIG_ARCH_SUN7I
	clk_disable(dev->module1_clk);
	clk_disable(dev->module2_clk);
#else
	clk_disable(dev->module_clk);
#endif
	clk_disable(dev->dram_clk);

	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
static ssize_t tvd_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
	struct dev *dev = video_drvdata(file);

	__dbg("%s\n", __FUNCTION__);

	start_generating(dev);
	return videobuf_read_stream(&dev->vb_vidq, data, count, ppos, 0,
					file->f_flags & O_NONBLOCK);
}

static unsigned int tvd_poll(struct file *file, struct poll_table_struct *wait)
{
	struct dev *dev = video_drvdata(file);
	struct videobuf_queue *q = &dev->vb_vidq;

	__dbg("%s\n", __FUNCTION__);

	start_generating(dev);
	return videobuf_poll_stream(file, q, wait);
}

static int tvd_open(struct file *file)
{
	struct dev *dev = video_drvdata(file);
	int ret,vflip,hflip;
	struct v4l2_control ctrl;
	
	__inf("%s\n", __FUNCTION__);

	if (dev->opened == 1) {
		__err("device open busy\n");
		return -EBUSY;
	}
		
	if (TVD_clk_init(dev)) 
	{
		__err("clock init fail!\n");
	}
	TVD_clk_open(dev);

	TVD_init(dev->regs);

//	TVD_config(0, 0);
	
//	init_timer(&timer);
//	timer.function = timer_func;
//	timer.data = 0;
//	timer.expires = jiffies + BLINK_DELAY;
//	add_timer(&timer);

//	TVD_irq_enable(TVD_LOCK);
//	TVD_irq_enable(TVD_UNLOCK);
	
	dev->input=0;//default input
	dev->opened=1;
	
	return 0;		
}

static int tvd_close(struct file *file)
{
	struct dev *dev = video_drvdata(file);
	int ret;
	
	__inf("%s\n", __FUNCTION__); 
	
//	del_timer(&timer);
	
//	TVD_irq_disable(0,TVD_LOCK);
//	TVD_irq_disable(0,TVD_UNLOCK);

	TVD_clk_close(dev);

	videobuf_stop(&dev->vb_vidq);
	videobuf_mmap_free(&dev->vb_vidq);

	dev->opened=0;
	stop_generating(dev);	
}

static int tvd_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct dev *dev = video_drvdata(file);
	int ret;

	__dbg("%s\n", __FUNCTION__);

	__dbg("mmap called, vma=0x%08lx\n", (unsigned long)vma);

	ret = videobuf_mmap_mapper(&dev->vb_vidq, vma);

	__dbg("vma start=0x%08lx, size=%ld, ret=%d\n",
	(unsigned long)vma->vm_start,
	(unsigned long)vma->vm_end - (unsigned long)vma->vm_start,
	ret);
	
	return ret;
}

static const struct v4l2_file_operations fops = {
	.owner	  = THIS_MODULE,
	.open	  = tvd_open,
	.release  = tvd_close,
	.read     = tvd_read,
	.poll	  = tvd_poll,
	.ioctl    = video_ioctl2,
	.mmap     = tvd_mmap,
};

//////////////////////////////////////////////////////////////////////////////////////////
static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct dev *dev = video_drvdata(file);

	__dbg("%s\n", __FUNCTION__);

	strcpy(cap->driver, "tvd");
	strcpy(cap->card, "tvd");
	strlcpy(cap->bus_info, dev->v4l2_dev.name, sizeof(cap->bus_info));

	cap->version = TVD_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | \
			    V4L2_CAP_READWRITE;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	struct fmt *fmt;

	__dbg("%s\n", __FUNCTION__);

//	if (f->index > ARRAY_SIZE(formats)-1) {
//		return -EINVAL;
//	}	
//	fmt = &formats[f->index];
//
//	strlcpy(f->description, fmt->name, sizeof(f->description));
//	f->pixelformat = fmt->fourcc;
	
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct dev *dev = video_drvdata(file);

	__dbg("%s\n", __FUNCTION__);
	
	f->fmt.pix.width        = dev->width;
	f->fmt.pix.height       = dev->height;
	f->fmt.pix.field        = dev->vb_vidq.field;
	f->fmt.pix.pixelformat  = dev->fmt->fourcc;
	f->fmt.pix.bytesperline = (f->fmt.pix.width * dev->fmt->depth) >> 3;
	f->fmt.pix.sizeimage    = f->fmt.pix.height * f->fmt.pix.bytesperline;

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct dev *dev = video_drvdata(file);
	struct fmt *fmt;
	int ret = 0;
	
	__dbg("%s\n", __FUNCTION__);

//	/*judge the resolution*/
//	if(f->fmt.pix.width > MAX_WIDTH || f->fmt.pix.height > MAX_HEIGHT) {
//		__err("size is too large,automatically set to maximum!\n");
//		f->fmt.pix.width = MAX_WIDTH;
//		f->fmt.pix.height = MAX_HEIGHT;
//	}
//
//	fmt = get_format(f);
//	if (!fmt) {
//		__err("Fourcc format (0x%08x) invalid.\n",f->fmt.pix.pixelformat);
//		return -EINVAL;
//	}
//
//	//f->fmt.pix.width = 720;
//	//f->fmt.pix.height = 480;
//
//	__dbg("pix->width=%d\n",f->fmt.pix.width);
//	__dbg("pix->height=%d\n",f->fmt.pix.height);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct dev *dev = video_drvdata(file);
	struct videobuf_queue *q = &dev->vb_vidq;
	int ret,width_buf,height_buf,width_len;
	struct fmt *fmt;
	
	__inf("%s\n", __FUNCTION__);
	
//	if (is_generating(dev)) {
//		__err("%s device busy\n", __func__);
//		return -EBUSY;
//	}
//	
//	mutex_lock(&q->vb_lock);
//	
//	ret = vidioc_try_fmt_vid_cap(file, priv, f);
//	if (ret < 0) {
//		__err("try format failed!\n");
//		goto out;
//	}
//		
//	fmt = get_format(f);
//	if (!fmt) {
//		__err("Fourcc format (0x%08x) invalid.\n",f->fmt.pix.pixelformat);
//		ret	= -EINVAL;
//		goto out;
//	}
		
	//save the current format info
//	dev->fmt = fmt;
//	dev->vb_vidq.field = f->fmt.pix.field;
//	dev->width  = f->fmt.pix.width;
//	dev->height = f->fmt.pix.height;
	
	//TVD_set_width(0,f->fmt.pix.width);
	//TVD_set_width_jump(0,f->fmt.pix.width);
	//TVD_set_height(0,f->fmt.pix.height/2);//for interlace here set half of height
	//TVD_set_fmt(0, fmt->output_fmt);

	ret = 0;
//out:
//	mutex_unlock(&q->vb_lock);
	return ret;
}

static int vidioc_s_fmt_type_private(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct dev *dev = video_drvdata(file);
	int i; 

	__dbg("%s\n", __FUNCTION__);
	
	if (is_generating(dev)) {
		__err("%s device busy\n", __func__);
		return -EBUSY;
	}
	
	dev->interface = f->fmt.raw_data[0];
	dev->system = f->fmt.raw_data[1];
	dev->format = f->fmt.raw_data[2]; //for test only
	dev->row = f->fmt.raw_data[8];
	dev->column = f->fmt.raw_data[9];
	dev->channel_index[0] = f->fmt.raw_data[10];
	dev->channel_index[1] = f->fmt.raw_data[11];
	dev->channel_index[2] = f->fmt.raw_data[12];
	dev->channel_index[3] = f->fmt.raw_data[13];

	dev->vb_vidq.field = V4L2_FIELD_NONE;
	dev->width  = dev->row*(dev->format?704:720);
	dev->height = dev->column*(dev->system?576:480);
	tvd_fmt.width = dev->width;
	tvd_fmt.height = dev->height;
	dev->fmt = &tvd_fmt;
		
	__inf("interface=%d\n",dev->interface);
	__inf("system=%d\n",dev->system);
	__inf("format=%d\n",dev->format);
	__inf("row=%d\n",dev->row);
	__inf("column=%d\n",dev->column);
	__inf("channel_index[0]=%d\n",dev->channel_index[0]);
	__inf("channel_index[1]=%d\n",dev->channel_index[1]);
	__inf("channel_index[2]=%d\n",dev->channel_index[2]);
	__inf("channel_index[3]=%d\n",dev->channel_index[3]);
	__inf("width=%d\n",dev->width);
	__inf("height=%d\n",dev->height);
	
	TVD_config(dev->interface, dev->system);
	for(i=0;i<4;i++){
		if(dev->channel_index[i]){
			TVD_set_fmt(0, dev->format?TVD_MB_YUV420:TVD_PL_YUV420);
			TVD_set_width(i, (dev->format?704:720));
			TVD_set_height(i, (dev->system?576:480)/2);
			TVD_set_width_jump(i, (dev->format?704:720)*dev->row);
			dev->channel_offset_y[i] = ((dev->channel_index[i]-1)%dev->row)*(dev->format?704:720) + ((dev->channel_index[i]-1)/dev->row)*dev->row*(dev->format?704:720)*(dev->system?576:480);
			dev->channel_offset_c[i] = ((dev->channel_index[i]-1)%dev->row)*(dev->format?704:720) + ((dev->channel_index[i]-1)/dev->row)*dev->row*(dev->format?704:720)*(dev->system?576:480)/2;
			__inf("channel_offset_y[%d]=%d\n", i, dev->channel_offset_y[i]);
			__inf("channel_offset_c[%d]=%d\n", i, dev->channel_offset_c[i]);
			
		}			
	}
	
//	msleep(10);
//	//write back the status
//	for(i=0;i<4;i++){
//		f->fmt.raw_data[16 + i] = TVD_get_status(i);
//		__inf("status[%d]=%d\n", i, f->fmt.raw_data[16 + i]);
//	}
	
	return 0;
}

static int vidioc_g_fmt_type_private(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct dev *dev = video_drvdata(file);
	int i;

	__dbg("%s\n", __FUNCTION__);
	
	f->fmt.raw_data[0]  = dev->interface       ;
	f->fmt.raw_data[1]  = dev->system          ;
	f->fmt.raw_data[2]  = dev->format          ; //for test only
	f->fmt.raw_data[8]  = dev->row             ;
	f->fmt.raw_data[9]  = dev->column          ;
	f->fmt.raw_data[10] = dev->channel_index[0];
	f->fmt.raw_data[11] = dev->channel_index[1];
	f->fmt.raw_data[12] = dev->channel_index[2];
	f->fmt.raw_data[13] = dev->channel_index[3];
	
	for(i=0;i<4;i++){
		f->fmt.raw_data[16 + i] = TVD_get_status(i);
	}
	
	return 0;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	struct dev *dev = video_drvdata(file);
	
	__dbg("%s\n", __FUNCTION__);
	
	return videobuf_reqbufs(&dev->vb_vidq, p);
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);
	
	return videobuf_querybuf(&dev->vb_vidq, p);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);
	
	return videobuf_qbuf(&dev->vb_vidq, p);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);

	return videobuf_dqbuf(&dev->vb_vidq, p, file->f_flags & O_NONBLOCK);
}

#ifdef CONFIG_VIDEO_V4L1_COMPAT
static int vidiocgmbuf(struct file *file, void *priv, struct video_mbuf *mbuf)
{
	struct dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);

	return videobuf_cgmbuf(&dev->vb_vidq, mbuf, 8);
}
#endif


static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct dev *dev = video_drvdata(file);
	struct dmaqueue *dma_q = &dev->vidq;
	struct tvd_buffer *buf;
	int j;
	
	int ret;

	__dbg("%s\n", __FUNCTION__);
	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		return -EINVAL;
	}	
	
	if (is_generating(dev)) {
		__err("stream has been already on\n");
		return 0;
	}
	
	/* Resets frame counters */
	dev->ms = 0;
	dev->jiffies = jiffies;

	dma_q->frame = 0;
	dma_q->ini_jiffies = jiffies;
	
	ret = videobuf_streamon(&dev->vb_vidq);
	if (ret) {
		return ret;
	}	
	
	buf = list_entry(dma_q->active.next, struct buffer, vb.queue);
	set_addr(dev,buf);
		
	for(j=0;j<4;j++){
		if(dev->channel_index[j]){
			dev->channel_irq = j;//FIXME, what frame done irq you should use when more than one channel signal?
			break;
		}
	}
	__inf("channel_irq=%d\n", dev->channel_irq);
	TVD_irq_status_clear(dev->channel_irq,TVD_FRAME_DONE);	
	TVD_irq_enable(dev->channel_irq,TVD_FRAME_DONE);
	for(j=0;j<4;j++){
		if(dev->channel_index[j])
			TVD_capture_on(j);
	}
	
	start_generating(dev);
		
	return 0;
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct dev *dev = video_drvdata(file);
	struct dmaqueue *dma_q = &dev->vidq;
	int ret;
	int j;

	__dbg("%s\n", __FUNCTION__);
	
	if (!is_generating(dev)) {
		__err("stream has been already off\n");
		return 0;
	}
	
	stop_generating(dev);

	/* Resets frame counters */
	dev->ms = 0;
	dev->jiffies = jiffies;

	dma_q->frame = 0;
	dma_q->ini_jiffies = jiffies;
	
	//FIXME
	TVD_irq_disable(dev->channel_irq,TVD_FRAME_DONE);
	TVD_irq_status_clear(dev->channel_irq,TVD_FRAME_DONE);
	for(j=0;j<4;j++){
		if(dev->channel_index[j])
			TVD_capture_off(j);
	}
	
	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		return -EINVAL;
	}

	ret = videobuf_streamoff(&dev->vb_vidq);
	if (ret!=0) {
		__err("videobu_streamoff error!\n");
		return ret;
	}
	
	if (ret!=0) {
		__err("videobuf_mmap_free error!\n");
		return ret;
	}
	
	return 0;
}

/* only one input in this sample driver */
static int vidioc_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	__dbg("%s\n", __FUNCTION__);
	if (inp->index > NUM_INPUTS-1) {
		__err("input index invalid!\n");
		return -EINVAL;
	}

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);

	*i = dev->input; 
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct dev *dev = video_drvdata(file);
	__dbg("%s\n", __FUNCTION__);

	if (i > NUM_INPUTS-1) {
		__err("set input error!\n");
		return -EINVAL;
	}
	dev->input = i;
	
	return 0;
}

static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	struct dev *dev = video_drvdata(file);
	int ret;
	__dbg("%s\n", __FUNCTION__);
			
	return ret;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct dev *dev = video_drvdata(file);
	int ret;
	__dbg("%s\n", __FUNCTION__);
		
	return ret;
}


static int vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct dev *dev = video_drvdata(file);
	struct v4l2_queryctrl qc;
	int ret;
	__dbg("%s\n", __FUNCTION__);
	
	qc.id = ctrl->id;
	ret = vidioc_queryctrl(file, priv, &qc);
	if (ret < 0) {
		return ret;
	}

	if (ctrl->value < qc.minimum || ctrl->value > qc.maximum) {
		return -ERANGE;
	}
	
	return ret;
}

static int vidioc_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *parms) 
{
	struct dev *dev = video_drvdata(file);
	int ret;
	__dbg("%s\n", __FUNCTION__);
	
	return ret;
}

static int vidioc_s_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *parms)
{
	struct dev *dev = video_drvdata(file);
	int ret;
	__dbg("%s\n", __FUNCTION__);
	
	return ret;
}

static const struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap          = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap, //
	.vidioc_s_fmt_type_private     = vidioc_s_fmt_type_private, //
	.vidioc_g_fmt_type_private     = vidioc_g_fmt_type_private, //
	.vidioc_reqbufs           = vidioc_reqbufs,
	.vidioc_querybuf          = vidioc_querybuf,
	.vidioc_qbuf              = vidioc_qbuf,
	.vidioc_dqbuf             = vidioc_dqbuf,
	.vidioc_enum_input        = vidioc_enum_input,
	.vidioc_g_input           = vidioc_g_input,
	.vidioc_s_input           = vidioc_s_input,
	.vidioc_streamon          = vidioc_streamon,
	.vidioc_streamoff         = vidioc_streamoff,
	.vidioc_queryctrl         = vidioc_queryctrl,
	.vidioc_g_ctrl            = vidioc_g_ctrl,
	.vidioc_s_ctrl            = vidioc_s_ctrl,
	.vidioc_g_parm		 	  = vidioc_g_parm,
	.vidioc_s_parm		  	  = vidioc_s_parm,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
	.vidiocgmbuf              = vidiocgmbuf,
#endif
};

//////////////////////////////////////////////////////////////////////////////////////////
static struct video_device device = {
	.name		= "tvd",
	.fops       = &fops,
	.ioctl_ops 	= &ioctl_ops,
	.release	= video_device_release,
};

//////////////////////////////////////////////////////////////////////////////////////////
static int buffer_setup(struct videobuf_queue *vq, unsigned int *count, unsigned int *size)
{
	struct dev *dev = vq->priv_data;
	__dbg("%s\n", __FUNCTION__);
		
	switch (dev->fmt->output_fmt) {
		case TVD_MB_YUV420:
		case TVD_PL_YUV420:
			*size = dev->width * dev->height * 2;
			break;	
		case TVD_PL_YUV422:
		default:
			*size = dev->width * dev->height * 2;
			break;
	}
	
	dev->frame_size = *size;
	
	if (*count < 3) {
		*count = 3;
		__err("buffer count is invalid, set to 3\n");
	} else if(*count > 5) {	
		*count = 5;
		__err("buffer count is invalid, set to 5\n");
	}

	while (*size * *count > MAX_BUFFER) {//FIXME
		(*count)--;
	}
	
	__dbg("%s, buffer count=%d, size=%d\n", __func__,*count, *size);
	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct buffer *buf)
{
	__dbg("%s\n", __FUNCTION__);
	__dbg("%s, state: %i\n", __func__, buf->vb.state);

#ifdef USE_DMA_CONTIG	
	videobuf_dma_contig_free(vq, &buf->vb);
#endif
	
	__dbg("free_buffer: freed\n");
	
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static int buffer_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb,
						  enum v4l2_field field)
{
	struct dev *dev = vq->priv_data;
	struct buffer *buf = container_of(vb, struct buffer, vb);
	int rc;

	__dbg("%s\n", __FUNCTION__);

	BUG_ON(NULL == dev->fmt);

	if (dev->width  < MIN_WIDTH || dev->width  > MAX_WIDTH ||
	    dev->height < MIN_HEIGHT || dev->height > MAX_HEIGHT) {
		return -EINVAL;
	}
	
	buf->vb.size = dev->frame_size;			
	
	if (0 != buf->vb.baddr && buf->vb.bsize < buf->vb.size) {
		return -EINVAL;
	}

	/* These properties only change when queue is idle, see s_fmt */
	buf->fmt       = dev->fmt;
	buf->vb.width  = dev->width;
	buf->vb.height = dev->height;
	buf->vb.field  = field;

	if (VIDEOBUF_NEEDS_INIT == buf->vb.state) {
		rc = videobuf_iolock(vq, &buf->vb, NULL);
		if (rc < 0) {
			goto fail;
		}
	}

	vb->boff= videobuf_to_dma_contig(vb);
	buf->vb.state = VIDEOBUF_PREPARED;

	return 0;

fail:
	free_buffer(vq, buf);
	return rc;
}

static void buffer_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
	struct dev *dev = vq->priv_data;
	struct buffer *buf = container_of(vb, struct buffer, vb);
	struct dmaqueue *vidq = &dev->vidq;

	__dbg("%s\n", __FUNCTION__);
	buf->vb.state = VIDEOBUF_QUEUED;
	list_add_tail(&buf->vb.queue, &vidq->active);
}

static void buffer_release(struct videobuf_queue *vq,
			   struct videobuf_buffer *vb)
{
	struct buffer *buf  = container_of(vb, struct buffer, vb);

	__dbg("%s\n", __FUNCTION__);
	
	free_buffer(vq, buf);
}

static struct videobuf_queue_ops video_qops = {
	.buf_setup    = buffer_setup,
	.buf_prepare  = buffer_prepare,
	.buf_queue    = buffer_queue,
	.buf_release  = buffer_release,
};

//////////////////////////////////////////////////////////////////////////////////////////
static int tvd_probe(struct platform_device *pdev)
{
	struct dev *dev;
	struct resource *res;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	int ret = 0;
	int twi_id;
	__dbg("%s\n", __FUNCTION__);

	/*request mem for dev*/	
	dev = kzalloc(sizeof(struct dev), GFP_KERNEL);
	if (!dev) {
		__err("request dev mem failed!\n");
		return -ENOMEM;
	}
	dev->id = pdev->id;
	dev->pdev = pdev;
	
	spin_lock_init(&dev->slock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		__err("failed to find the registers\n");
		ret = -ENOENT;
		goto err_info;
	}

	dev->regs_res = request_mem_region(res->start, resource_size(res),
			dev_name(&pdev->dev));
	if (!dev->regs_res) {
		__err("failed to obtain register region\n");
		ret = -ENOENT;
		goto err_info;
	}
	
	dev->regs = ioremap(res->start, resource_size(res));
	if (!dev->regs) {
		__err("failed to map registers\n");
		ret = -ENXIO;
		goto err_req_region;
	}
 
    /*get irq resource*/	
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		__err("failed to get IRQ resource\n");
		ret = -ENXIO;
		goto err_regs_unmap;
	}
	
	dev->irq = res->start;
	
	ret = request_irq(dev->irq, TVD_isr, 0, pdev->name, dev);
	if (ret) {
		__err("failed to install irq (%d)\n", ret);
		goto err_clk;
	}
	
    /* v4l2 device register */
	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);	
	if (ret) {
		__err("Error registering v4l2 device\n");
		goto err_irq;
		
	}

	dev_set_drvdata(&(pdev)->dev, (dev));

	if (TVD_clk_init(dev)) {
		__err("clock init fail!\n");
		ret = -ENXIO;
		goto unreg_dev;
	}

	/*video device register	*/
	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd) {
		goto err_clk;
	}	

	*vfd = device;
	vfd->v4l2_dev = &dev->v4l2_dev;

	dev_set_name(&vfd->dev, "tvd");
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0) {
		goto rel_vdev;
	}	
	video_set_drvdata(vfd, dev);
	
	/*add device list*/
	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->devlist, &devlist);

	if (video_nr != -1) {
		video_nr++;//??
	}
	dev->vfd = vfd;

	__inf("V4L2 device registered as %s\n",video_device_node_name(vfd));

	/*initial video buffer queue*/
	videobuf_queue_dma_contig_init_cached(&dev->vb_vidq, &video_qops,
			NULL, &dev->slock, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			V4L2_FIELD_NONE,
			sizeof(struct buffer), dev,NULL);
	
	/* init video dma queues */
	INIT_LIST_HEAD(&dev->vidq.active);
	//init_waitqueue_head(&dev->vidq.wq);
	
	return 0;

rel_vdev:
	video_device_release(vfd);
err_clk:
	TVD_clk_exit(dev);	
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);	
err_irq:
	free_irq(dev->irq, dev);
err_regs_unmap:
	iounmap(dev->regs);
err_req_region:
	release_resource(dev->regs_res);
//	kfree(dev->regs_res);
err_info:
	kfree(dev);
	__err("failed to install\n");
	
	return ret;
}

static int tvd_release(void)
{
	struct dev *dev;
	struct list_head *list;

	__dbg("%s\n", __FUNCTION__);
	
	while (!list_empty(&devlist)) 
	{
		list = devlist.next;
		list_del(list);
		dev = list_entry(list, struct dev, devlist);

		v4l2_info(&dev->v4l2_dev, "unregistering %s\n", video_device_node_name(dev->vfd));
		video_unregister_device(dev->vfd);
		v4l2_device_unregister(&dev->v4l2_dev);
		kfree(dev);
	}

	return 0;
}

static int __devexit tvd_remove(struct platform_device *pdev)
{
	struct dev *dev=(struct dev *)dev_get_drvdata(&(pdev)->dev);
	
	__dbg("%s\n", __FUNCTION__);
	
__dbg("%d\n", __LINE__);	
	free_irq(dev->irq, dev);//
__dbg("%d\n", __LINE__);	
	TVD_clk_exit(dev);
__dbg("%d\n", __LINE__);
	iounmap(dev->regs);
__dbg("%d\n", __LINE__);
//	release_resource(dev->regs_res);
__dbg("%d\n", __LINE__);
	kfree(dev->regs_res);
__dbg("%d\n", __LINE__);
	kfree(dev);
__dbg("%d\n", __LINE__);
	return 0;
}

static int tvd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct dev *dev=(struct dev *)dev_get_drvdata(&(pdev)->dev);
	int ret;
	
	__dbg("%s\n", __FUNCTION__);
	
	if (dev->opened==1) {
		TVD_clk_close(dev);		
	}
	return 0;
}

static int tvd_resume(struct platform_device *pdev)
{
	int ret;
	struct dev *dev=(struct dev *)dev_get_drvdata(&(pdev)->dev);
	
	__dbg("%s\n", __FUNCTION__);

	if (dev->opened==1) {
		TVD_clk_open(dev);
	} 
	
	return 0;
}

static struct platform_driver tvd_driver = {
	.probe		= tvd_probe,
	.remove		= __devexit_p(tvd_remove),
	.suspend	= tvd_suspend,
	.resume		= tvd_resume,
	.driver = {
		.name	= "tvd",
		.owner	= THIS_MODULE,
	}
};

static struct resource tvd_resource[2] = {
	[0] = {
		.start	= TVD_REGS_BASE,
		.end	= (TVD_REGS_BASE + TVD_REG_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= SW_INT_IRQNO_TVD,
		.end	= SW_INT_IRQNO_TVD,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device tvd_device = {
		.name           	= "tvd",
	    .id             	= -1,//??
		.num_resources		= ARRAY_SIZE(tvd_resource),
	    .resource       	= tvd_resource,
		.dev            	= {}
};

static int __init tvd_init(void)
{
	u32 ret;
	int used;
	__inf("Welcome to TV decoder driver!\n");
	__dbg("%s\n", __FUNCTION__);
	
	//get system config from fex
	ret = script_parser_fetch("tvin_para", "tvin_used", &used , sizeof(int));
	if (ret) {
		__err("fetch [tvin_used] from fex file failed!\n");
		return -1;
	}
	
	if(!used)
	{
		__err("detect fex file tvin_used=0, TVD driver is disable!\n");
		return -1;
	}
	
	ret = platform_device_register(&tvd_device);
	if (ret) {
		__err("platform device register failed!\n");
		return -1;
	}
	
	ret = platform_driver_register(&tvd_driver);
	
	if (ret) {
		__err("platform driver register failed!\n");
		return -1;
	}
	return 0;
}

static void __exit tvd_exit(void)
{
	__dbg("%s\n", __FUNCTION__);
	tvd_release();
	platform_driver_unregister(&tvd_driver);
}

module_init(tvd_init);
module_exit(tvd_exit);

MODULE_AUTHOR("jshwang");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TV decoder driver");
