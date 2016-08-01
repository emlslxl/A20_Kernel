/*
 * sunxi_can.c - CAN network driver for allwinner SoC CAN controller
 *
 * (C) 2012 by threewater <threewaterL@163.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 *
 *
 * Your platform definition file should specify something like:
 *
 *
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/if_arp.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>

#include <linux/can/dev.h>
#include <linux/can/error.h>

#include <plat/sys_config.h>

//#define DEBUG

#ifdef DEBUG
#define DPRINTF(x...) printk("can: "x)
#else
#define DPRINTF(x...)
#endif


#define SUNXICAN_ECHO_SKB_MAX	1 /* the sunxi can has one TX buffer object */
#define SUNXI_CAN_NAPI_WEIGHT	16

/*
	sunxi can register
*/
#define CAN_MSEL				0x0000	//Can Mode Select Register
#define CAN_CMD					0x0004	//Can Command Register
#define CAN_STA 				0x0008	//Can Status Register
#define CAN_INT 				0x000c	//Can Interrupt Flag Register
#define CAN_INTEN				0x0010	//Can Interrupt Enable Register
#define CAN_BTIME				0x0014	//Can Bus Timing 0 Register
#define CAN_TEWL				0x0018	//Can Tx Error Warning Limit Register
#define CAN_ERRC				0x001c	//Can Error Counter Register
#define CAN_RMCNT				0x0020	//Can Receive Message Counter Register
#define CAN_RBUFSA				0x0024	//Can Receive Buffer Start Address Register
#define CAN_BUF0				0x0040	//Can Tx/Rx Buffer 0  Register
#define CAN_BUF1				0x0044	//Can Tx/Rx Buffer 1  Register
#define CAN_BUF2				0x0048	//Can Tx/Rx Buffer 2  Register
#define CAN_BUF3				0x004c	//Can Tx/Rx Buffer 3  Register
#define CAN_BUF4				0x0050	//Can Tx/Rx Buffer 4  Register
#define CAN_BUF5				0x0054	//Can Tx/Rx Buffer 5  Register
#define CAN_BUF6				0x0058	//Can Tx/Rx Buffer 6  Register
#define CAN_BUF7				0x005c	//Can Tx/Rx Buffer 7  Register
#define CAN_BUF8				0x0060	//Can Tx/Rx Buffer 8  Register
#define CAN_BUF9				0x0064	//Can Tx/Rx Buffer 9  Register
#define CAN_BUF10				0x0068	//Can Tx/Rx Buffer 10 Register
#define CAN_BUF11				0x006c	//Can Tx/Rx Buffer 11 Register
#define CAN_BUF12				0x0070	//Can Tx/Rx Buffer 12 Register
#define CAN_ACPC				0x0040	//Can Acceptance Code 0 Register
#define CAN_ACPM				0x0044	//Can Acceptance Mask 0 Register

/* registers bit field */
/* mode select */
#define SLEEP_MODE			(1<<4)
#define SINGLE_FILTER		(1<<3)
#define DUAL_FILTERS		(0<<3)
#define LOOPBACK_MODE		(1<<2)
#define LISTEN_ONLY_MODE	(1<<1)
#define RESET_MODE			(1<<0)
/* command */
#define SELF_RCV_REQ		(1<<4)
#define CLEAR_DOVERRUN		(1<<3)
#define RELEASE_RBUF		(1<<2)
#define ABORT_REQ			(1<<1)
#define TRANS_REQ			(1<<0)
/* status */
#define BUS_OFF 			(1<<7)
#define ERR_STA 			(1<<6)
#define TRANS_BUSY			(1<<5)
#define RCV_BUSY			(1<<4)
#define TRANS_OVER			(1<<3)
#define TBUF_RDY			(1<<2)
#define DATA_ORUN			(1<<1)
#define RBUF_RDY			(1<<0)
/* interrupt */
#define BUS_ERR 			(1<<7)
#define ARB_LOST			(1<<6)
#define ERR_PASSIVE 		(1<<5)
#define WAKEUP				(1<<4)
#define DATA_ORUNI			(1<<3)
#define ERR_WRN 			(1<<2)
#define TBUF_VLD			(1<<1)
#define RBUF_VLD			(1<<0)
#define IRQ_ALL				(0xff)
#define IRQ_ALL_ERR			(BUS_ERR|ARB_LOST|ERR_PASSIVE|DATA_ORUNI|ERR_WRN)
/* output control */
#define NOR_OMODE			(2)
#define CLK_OMODE			(3)
/* arbitration lost flag*/

/* error code */
#define BIT_ERR 			(0<<6)
#define FORM_ERR			(1<<6)
#define STUFF_ERR			(2<<6)
#define OTHER_ERR			(3<<6)
#define ERR_INRCV			(1<<5)
#define ERR_INTRANS 		(0<<5)

/* filter mode */
#define FILTER_CLOSE		0
#define SINGLE_FLTER_MODE	1
#define DUAL_FILTER_MODE	2

#define CANBUFF0_EFF_FLAG	(1<<7)
#define CANBUFF0_RTR_FLAG	(1<<6)

struct sunxi_priv {
	struct can_priv can;		/* must be the first member! */
	struct net_device *dev;
	struct napi_struct napi;

	spinlock_t mbx_lock; /* CAN register needs protection */

	void __iomem *reg_base;

	struct clk *clk;
	struct clk *pclk;
};

static struct can_bittiming_const sunxi_bittiming_const = {
	.name		= KBUILD_MODNAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static inline u32 sunxi_read(const struct sunxi_priv *priv, unsigned int reg)
{
	return __raw_readl(priv->reg_base + reg);
}

static inline void sunxi_write(const struct sunxi_priv *priv, unsigned int reg,
		u32 value)
{
	__raw_writel(value, priv->reg_base + reg);
}

static inline void can_sel_mode(const struct sunxi_priv *priv, __u32 mode)
{
	sunxi_write(priv, CAN_MSEL, sunxi_read(priv, CAN_MSEL)|mode);
}

static inline void can_release_mode(const struct sunxi_priv *priv, __u32 mode)
{
	sunxi_write(priv, CAN_MSEL, sunxi_read(priv, CAN_MSEL)&(~mode));
}

static inline void can_send_cmd(const struct sunxi_priv *priv, __u32 cmd)
{
	sunxi_write(priv, CAN_CMD, cmd);
}

static inline u32 can_enable_imask(struct sunxi_priv *priv, __u32 imask)
{
	unsigned long flags;
	u32 newmask;

	spin_lock_irqsave(&priv->mbx_lock, flags);
	newmask = sunxi_read(priv, CAN_INTEN) | imask;
	sunxi_write(priv, CAN_INTEN, newmask);
	spin_unlock_irqrestore(&priv->mbx_lock, flags);

	return newmask;
}

static inline u32 can_disable_imask(struct sunxi_priv *priv, __u32 imask)
{
	unsigned long flags;
	u32 newmask;

	spin_lock_irqsave(&priv->mbx_lock, flags);
	newmask = sunxi_read(priv, CAN_INTEN) & (~imask);
	sunxi_write(priv, CAN_INTEN, newmask);
	spin_unlock_irqrestore(&priv->mbx_lock, flags);

	return newmask;
}

static inline void can_clear_imask(const struct sunxi_priv *priv, __u32 imask)
{
	sunxi_write(priv, CAN_INTEN, 0);
}

static inline u32 can_get_status(const struct sunxi_priv *priv)
{
	return sunxi_read(priv, CAN_STA);
}

typedef struct {
	u32 code;
	char* msg;
} bus_err_code_t;

static bus_err_code_t bus_err_code[24] = {
	{0x3, "START"},
	{0x2, "ID28~21"},
	{0x6, "ID20~18"},
	{0x4, "SRTR"},
	{0x5, "IDE"},
	{0x7, "ID17~13"},
	{0xf, "ID12~5"},
	{0xe, "ID4~0"},
	{0xc, "RTR"},
	{0xd, "RB1"},
	{0x9, "RB0"},
	{0xb, "DLEN"},
	{0xA, "DATA Field"},
	{0x8, "CRC Sequence"},
	{0x18, "CRC Delimiter"},
	{0x19, "ACK"},
	{0x1B, "ACK Delimiter"},
	{0x1A, "END"},
	{0x12, "Intermission"},
	{0x11, "Active error"},
	{0x6, "Passive error"},
	{0x13, "Tolerate dominant bits"},
	{0x17, "Error delimiter"},
	{0x1c, "Overload"}
};

#ifdef DEBUG
static void can_buserr_msg(struct net_device *dev, u32 err_code)
{
	int i;
	
	for (i=0; i<ARRAY_SIZE(bus_err_code); i++)
	{
		if (err_code == bus_err_code[i].code){
			DPRINTF("can bus error, code = %x, msg = %s\n", err_code, bus_err_code[i].msg);
			break;
		}
	}
	DPRINTF("Can't find bus error code 0x%x\n", err_code);
}
#else
#define can_buserr_msg(dev,err_code)
#endif

//must in reset mode
static int sunxi_set_bittiming(struct net_device *dev)
{
	const struct sunxi_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	u32 reg_btime;

	reg_btime = ((priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES) ? 1 << 15 : 0) |
		((bt->sjw-1)<<6) | ((bt->phase_seg1 + bt->prop_seg - 1)<<8) | 
		((bt->phase_seg2 - 1)<<12);
	reg_btime = (reg_btime<<8)|(bt->brp-1);

	DPRINTF("writing sunxi_btime: 0x%08x\n", reg_btime);

	sunxi_write(priv, CAN_BTIME, reg_btime);

	return 0;
}

#define sunxi_get_err_cnt(ecr, tx, rx)	do{(tx) = (ecr) & 0xff; (rx) = (ecr)>>16;}while(0)

static int sunxi_get_berr_counter(const struct net_device *dev,
		struct can_berr_counter *bec)
{
	const struct sunxi_priv *priv = netdev_priv(dev);
	u32 reg_ecr = sunxi_read(priv, CAN_ERRC);

	DPRINTF("err counter reg: 0x%08x\n", reg_ecr);

	sunxi_get_err_cnt(reg_ecr, bec->txerr, bec->rxerr);

	return 0;
}

static void sunxi_chip_start(struct net_device *dev)
{
	struct sunxi_priv *priv = netdev_priv(dev);
	u32 reg;

	//set can controller in reset mode
	can_sel_mode(priv, RESET_MODE);
	sunxi_set_bittiming(dev);
	//filter close, mask 0xffffffff, match nothing
	sunxi_write(priv, CAN_ACPM, 0xffffffff);

	//enable interrupt
	reg = IRQ_ALL_ERR|RBUF_VLD|TBUF_VLD;
	can_enable_imask(priv, reg);
	
	//set error warning
	//keep default value - 96
	//...
	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	//return to transfer mode
	can_release_mode(priv, RESET_MODE);
}

static void sunxi_chip_stop(struct net_device *dev, enum can_state state)
{
	struct sunxi_priv *priv = netdev_priv(dev);

	/* disable interrupts */
	can_disable_imask(priv, IRQ_ALL);

	//set can controller in reset mode
	can_sel_mode(priv, RESET_MODE);

	priv->can.state = state;
}

static netdev_tx_t sunxi_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct sunxi_priv *priv = netdev_priv(dev);
	struct can_frame *cf = (struct can_frame *)skb->data;
	canid_t id;
	u32 reg;
	int i, dlc;

	DPRINTF("%s\n", __FUNCTION__);

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	id = cf->can_id;
	reg = dlc = cf->can_dlc;

	if(id & CAN_RTR_FLAG)
		reg |= CANBUFF0_RTR_FLAG;

	if(id & CAN_EFF_FLAG){//extern frame
		reg |= CANBUFF0_EFF_FLAG;
		sunxi_write(priv, CAN_BUF0, reg);
		reg = CAN_BUF5;

		sunxi_write(priv, CAN_BUF1, 0xff&(id>>21));  //id28~21
		sunxi_write(priv, CAN_BUF2, 0xff&(id>>13));  //id20~13
		sunxi_write(priv, CAN_BUF3, 0xff&(id>>5));	 //id12~5
		sunxi_write(priv, CAN_BUF4, (id&0x1f)<<3);	//id4~0
	}
	else{//standard frame
		sunxi_write(priv, CAN_BUF0, reg);
		reg = CAN_BUF3;

		sunxi_write(priv, CAN_BUF1, 0xff&(id>>3)); //id28~21
		sunxi_write(priv, CAN_BUF2, (id&0x7)<<5);
	}

	for (i=0; i<dlc; i++)
		sunxi_write(priv, reg+i*4, cf->data[i]);

	can_put_echo_skb(skb, dev, 0);

	//request transfer
	can_send_cmd(priv, TRANS_REQ);

	DPRINTF("%s xmit done\n", __FUNCTION__);

	return NETDEV_TX_OK;
}


static int sunxi_rcv_pkt(const struct sunxi_priv *priv)
{
	struct net_device *dev = priv->dev;
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32 data, reg, i;

	DPRINTF("%s\n", __FUNCTION__);

	skb = alloc_can_skb(dev, &cf);
	if (!skb) {
		if (printk_ratelimit())
			netdev_err(dev, "sunxi_can_rx_pkt: alloc_can_skb() failed\n");

		can_send_cmd(priv, RELEASE_RBUF);
		return -ENOMEM;
	}

	//get data
	data = sunxi_read(priv, CAN_BUF0);
	cf->can_dlc = data&0xf;

	if(data&CANBUFF0_EFF_FLAG){	//extern frame
		cf->can_id = (((__u32)sunxi_read(priv, CAN_BUF1))<<21) |	//id28~21
					(((__u32)sunxi_read(priv, CAN_BUF2))<<13) |    //id20~13
					(((__u32)sunxi_read(priv, CAN_BUF3))<<5) |	   //id12~5
					((sunxi_read(priv, CAN_BUF4)>>3)&0x1f); 	   //id4~0
		cf->can_id |= CAN_EFF_FLAG;
		reg = CAN_BUF5;
	}
	else{ //standard frame
		cf->can_id = (((__u32)sunxi_read(priv, CAN_BUF1))<<3) | //id28~21
					((sunxi_read(priv, CAN_BUF2)>>5)&0x7);	   //id20~18
		reg = CAN_BUF3;
	}

	for (i = 0; i<cf->can_dlc; i++){
			cf->data[i] = sunxi_read(priv, reg+i*4);
	}
	
	if (data & CANBUFF0_RTR_FLAG)
		cf->can_id |= CAN_RTR_FLAG;

	//release buffer
	can_send_cmd(priv, RELEASE_RBUF);

	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	DPRINTF("sunxi_can_rx_pkt: dlc=%d\n", cf->can_dlc);
	
	return 0;
}

/*
 * sunxi_rx_poll - sunxi receive pkts
 *
 * The receive mailboxes start from highest numbered mailbox till last xmit
 * mailbox. On CAN frame reception the hardware places the data into highest
 * numbered mailbox that matches the CAN ID filter. Since all receive mailboxes
 * have same filtering (ALL CAN frames) packets will arrive in the highest
 * available RX mailbox and we need to ensure in-order packet reception.
 *
 * To ensure the packets are received in the right order we logically divide
 * the RX mailboxes into main and buffer mailboxes. Packets are received as per
 * mailbox priotity (higher to lower) in the main bank and once it is full we
 * disable further reception into main mailboxes. While the main mailboxes are
 * processed in NAPI, further packets are received in buffer mailboxes.
 *
 * We maintain a RX next mailbox counter to process packets and once all main
 * mailboxe packets are passed to the upper stack we enable all of them but
 * continue to process packets received in buffer mailboxes. With each packet
 * received from buffer mailbox we enable it immediately so as to handle the
 * overflow from higher mailboxes.
 */
static int sunxi_rx_poll(struct napi_struct *napi, int quota)
{
	struct net_device *dev = napi->dev;
	struct sunxi_priv *priv = netdev_priv(dev);
	u32 num_pkts = 0;

	DPRINTF("%s\n", __FUNCTION__);

	if (!netif_running(dev))
		return 0;

	while (sunxi_read(priv, CAN_STA)&RBUF_RDY && num_pkts < quota) {
		if(sunxi_rcv_pkt(priv)<0)
			return num_pkts;
		++num_pkts;
	}

	/* Enable packet interrupt if all pkts are handled */
	if(!(sunxi_read(priv, CAN_STA)&RBUF_RDY)){
		napi_complete(napi);
		can_enable_imask(priv, RBUF_VLD);
	}

	return num_pkts;
}

static int sunxi_can_err(struct net_device *dev, u8 intflag)
{
	struct sunxi_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state canst = priv->can.state;
	u32 status;

	status = sunxi_read(priv, CAN_STA);
	DPRINTF("can interrupt flag error : raw_int = %08x, status=0x%x\n", intflag, status);
	can_buserr_msg(dev, (status>>16)&0x1f);

	if ((status&(BUS_OFF|ERR_STA|DATA_ORUN)) && !(status&TBUF_RDY)){
		//cancel send
		can_send_cmd(priv, ABORT_REQ);
	}

	skb = alloc_can_err_skb(dev, &cf);
	if (skb == NULL)
		return -ENOMEM;

	if (intflag&DATA_ORUNI){
		/* data overrun interrupt */
		printk(KERN_DEBUG"can data over run\n");
		can_send_cmd(priv, CLEAR_DOVERRUN);//clear overrun flag

		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;

		if(!(status&RBUF_RDY)){	//restart
			unsigned long flags;
			u32 mask;
			spin_lock_irqsave(&priv->mbx_lock, flags);
			mask = sunxi_read(priv, CAN_INTEN);
			can_sel_mode(priv, RESET_MODE);
			can_release_mode(priv, RESET_MODE);
			sunxi_write(priv, CAN_INTEN, mask);
			spin_unlock_irqrestore(&priv->mbx_lock, flags);
		}
	}

	if (intflag & ERR_WRN) {
		/* error warning interrupt */
		printk(KERN_DEBUG"error warning interrupt\n");

		if (status & BUS_OFF) {
			canst = CAN_STATE_BUS_OFF;
			cf->can_id |= CAN_ERR_BUSOFF;
			can_bus_off(dev);
		} else if (status & ERR_STA) {
			canst = CAN_STATE_ERROR_WARNING;
		} else
			canst = CAN_STATE_ERROR_ACTIVE;
	}

	if (intflag & BUS_ERR) {
		/* bus error interrupt */
		priv->can.can_stats.bus_error++;
		stats->rx_errors++;
		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
	}

	if (intflag & ERR_PASSIVE) {
		/* error passive interrupt */
		printk(KERN_DEBUG"error passive interrupt\n");
		if (status & ERR_STA)
			canst = CAN_STATE_ERROR_PASSIVE;
		else
			canst = CAN_STATE_ERROR_ACTIVE;
	}

	if (intflag & ARB_LOST) {
		/* arbitration lost interrupt */
		printk(KERN_DEBUG"arbitration lost interrupt\n");
		priv->can.can_stats.arbitration_lost++;
		stats->tx_errors++;
		cf->can_id |= CAN_ERR_LOSTARB;
		cf->data[0] = 0 & 0x1f;	//to do add alc pos
	}

	if (canst != priv->can.state && (canst == CAN_STATE_ERROR_WARNING ||
					 canst == CAN_STATE_ERROR_PASSIVE)) {
		u32 reg_ecr = sunxi_read(priv, CAN_ERRC);
		uint8_t rxerr;
		uint8_t txerr;

		sunxi_get_err_cnt(reg_ecr, txerr, rxerr);

		cf->can_id |= CAN_ERR_CRTL;
		if (canst == CAN_STATE_ERROR_WARNING) {
			priv->can.can_stats.error_warning++;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_WARNING :
				CAN_ERR_CRTL_RX_WARNING;
		} else {
			priv->can.can_stats.error_passive++;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_PASSIVE :
				CAN_ERR_CRTL_RX_PASSIVE;
		}
		cf->data[6] = txerr;
		cf->data[7] = rxerr;
	}

	priv->can.state = canst;

	netif_rx(skb);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 0;
}

/*
 * interrupt handler
 */
static irqreturn_t sunxi_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct sunxi_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	u8 flag, mskd;

	flag  = sunxi_read(priv, CAN_INT);
	mskd = flag & sunxi_read(priv, CAN_INTEN);

	DPRINTF("%s\n", __FUNCTION__);

	/* Ignore masked interrupts */
	if (!mskd)
		return IRQ_NONE;

	if (mskd & IRQ_ALL_ERR){
		sunxi_can_err(dev, flag);
	}
	
	//Receive buffer valid
	if (mskd&RBUF_VLD){
		can_disable_imask(priv, RBUF_VLD);
		napi_schedule(&priv->napi);
	}

	//tx buffer valid
	if (mskd&TBUF_VLD){
		/* transmission complete interrupt */
		stats->tx_bytes += sunxi_read(priv, CAN_BUF0) & 0xf;
		stats->tx_packets++;
		can_get_echo_skb(dev, 0);
		netif_wake_queue(dev);
	}
	
	//clear the interrupt
	sunxi_write(priv, CAN_INT, flag);

	return IRQ_HANDLED;
}

static int sunxi_open(struct net_device *dev)
{
	struct sunxi_priv *priv = netdev_priv(dev);
	int err;

	DPRINTF("%s\n", __FUNCTION__);

	clk_enable(priv->pclk);
	clk_enable(priv->clk);

	/* check or determine and set bittime */
	err = open_candev(dev);
	if (err)
		goto out;

	/* register interrupt handler */
	if (request_irq(dev->irq, sunxi_irq, IRQF_SHARED,
			dev->name, dev)) {
		err = -EAGAIN;
		goto out_close;
	}

	/* start chip and queuing */
	sunxi_chip_start(dev);
	napi_enable(&priv->napi);
	netif_start_queue(dev);

	return 0;

 out_close:
	close_candev(dev);
 out:
	clk_disable(priv->clk);
	clk_disable(priv->pclk);

	return err;
}

/*
 * stop CAN bus activity
 */
static int sunxi_close(struct net_device *dev)
{
	struct sunxi_priv *priv = netdev_priv(dev);

	DPRINTF("%s\n", __FUNCTION__);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	sunxi_chip_stop(dev, CAN_STATE_STOPPED);

	free_irq(dev->irq, dev);
	clk_disable(priv->clk);
	clk_disable(priv->pclk);

	close_candev(dev);

	return 0;
}

static int sunxi_set_mode(struct net_device *dev, enum can_mode mode)
{
	DPRINTF("%s\n", __FUNCTION__);

	switch (mode) {
	case CAN_MODE_START:
		sunxi_chip_start(dev);
		netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct net_device_ops sunxi_netdev_ops = {
	.ndo_open	= sunxi_open,
	.ndo_stop	= sunxi_close,
	.ndo_start_xmit	= sunxi_start_xmit,
};


/**********************register dump***************************/
static ssize_t register_show(struct device *dev,	struct device_attribute *attr, char *buf);

#define DEF_REG_ATTR(_name)	static DEVICE_ATTR(REG##_name, S_IWUSR|S_IRUGO, register_show, NULL);

#define DEF_SUNXI_ATTR(_name)	&dev_attr_##REG##_name.attr,

DEF_REG_ATTR(MSEL)
DEF_REG_ATTR(CMD)
DEF_REG_ATTR(STA)
DEF_REG_ATTR(INT)
DEF_REG_ATTR(INTEN)
DEF_REG_ATTR(BTIME)
DEF_REG_ATTR(TEWL)
DEF_REG_ATTR(ERRC)
DEF_REG_ATTR(RMCNT)
DEF_REG_ATTR(RBUFSA)
DEF_REG_ATTR(BUF0)
DEF_REG_ATTR(BUF1)
DEF_REG_ATTR(BUF2)
DEF_REG_ATTR(BUF3)
DEF_REG_ATTR(BUF4)
DEF_REG_ATTR(BUF5)
DEF_REG_ATTR(BUF6)
DEF_REG_ATTR(BUF7)
DEF_REG_ATTR(BUF8)
DEF_REG_ATTR(BUF9)
DEF_REG_ATTR(BUF10)
DEF_REG_ATTR(BUF11)
DEF_REG_ATTR(BUF12)
DEF_REG_ATTR(ACPC)
DEF_REG_ATTR(ACPM)

static struct attribute *sunxi_sysfs_attrs[] = {
	DEF_SUNXI_ATTR(MSEL)
	DEF_SUNXI_ATTR(CMD)
	DEF_SUNXI_ATTR(STA)
	DEF_SUNXI_ATTR(INT)
	DEF_SUNXI_ATTR(INTEN)
	DEF_SUNXI_ATTR(BTIME)
	DEF_SUNXI_ATTR(TEWL)
	DEF_SUNXI_ATTR(ERRC)
	DEF_SUNXI_ATTR(RMCNT)
	DEF_SUNXI_ATTR(RBUFSA)
	DEF_SUNXI_ATTR(BUF0)
	DEF_SUNXI_ATTR(BUF1)
	DEF_SUNXI_ATTR(BUF2)
	DEF_SUNXI_ATTR(BUF3)
	DEF_SUNXI_ATTR(BUF4)
	DEF_SUNXI_ATTR(BUF5)
	DEF_SUNXI_ATTR(BUF6)
	DEF_SUNXI_ATTR(BUF7)
	DEF_SUNXI_ATTR(BUF8)
	DEF_SUNXI_ATTR(BUF9)
	DEF_SUNXI_ATTR(BUF10)
	DEF_SUNXI_ATTR(BUF11)
	DEF_SUNXI_ATTR(BUF12)
	DEF_SUNXI_ATTR(ACPC)
	DEF_SUNXI_ATTR(ACPM)
	NULL,
};

static struct attribute_group sunxi_sysfs_attr_group = {
	.attrs = sunxi_sysfs_attrs,
};

struct register_node{
	unsigned int offset;
	const char* name;
};

#define REGISTER_NODE_DEF(_name)	{.offset=CAN_##_name, .name = "REG"#_name, }

static struct register_node can_reg_node[]={
	REGISTER_NODE_DEF(MSEL),
	REGISTER_NODE_DEF(CMD),
	REGISTER_NODE_DEF(STA),
	REGISTER_NODE_DEF(INT),
	REGISTER_NODE_DEF(INTEN),
	REGISTER_NODE_DEF(BTIME),
	REGISTER_NODE_DEF(TEWL),
	REGISTER_NODE_DEF(ERRC),
	REGISTER_NODE_DEF(RMCNT),
	REGISTER_NODE_DEF(RBUFSA),
	REGISTER_NODE_DEF(BUF0),
	REGISTER_NODE_DEF(BUF1),
	REGISTER_NODE_DEF(BUF2),
	REGISTER_NODE_DEF(BUF3),
	REGISTER_NODE_DEF(BUF4),
	REGISTER_NODE_DEF(BUF5),
	REGISTER_NODE_DEF(BUF6),
	REGISTER_NODE_DEF(BUF7),
	REGISTER_NODE_DEF(BUF8),
	REGISTER_NODE_DEF(BUF9),
	REGISTER_NODE_DEF(BUF10),
	REGISTER_NODE_DEF(BUF11),
	REGISTER_NODE_DEF(BUF12),
	REGISTER_NODE_DEF(ACPC),
	REGISTER_NODE_DEF(ACPM),
};

static ssize_t register_show(struct device *dev,	struct device_attribute *attr, char *buf)
{
	int i;
	struct register_node *pnode=can_reg_node;
	const struct sunxi_priv *priv=netdev_priv(to_net_dev(dev));
	u32 data=0;
	
	for(i=0;i<ARRAY_SIZE(can_reg_node); i++, pnode++){
		if(strcmp(pnode->name, attr->attr.name)==0){
			//find node
			data = sunxi_read(priv, pnode->offset);
			return snprintf(buf, PAGE_SIZE, "0x%08x\n", data);
		}
	}

	return 0;
}
/*
static ssize_t at91_sysfs_set_mb0_id(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct net_device *ndev = to_net_dev(dev);
	struct at91_priv *priv = netdev_priv(ndev);
	unsigned long can_id;
	ssize_t ret;
	int err;

	rtnl_lock();

	if (ndev->flags & IFF_UP) {
		ret = -EBUSY;
		goto out;
	}

	err = strict_strtoul(buf, 0, &can_id);
	if (err) {
		ret = err;
		goto out;
	}

	if (can_id & CAN_EFF_FLAG)
		can_id &= CAN_EFF_MASK | CAN_EFF_FLAG;
	else
		can_id &= CAN_SFF_MASK;

	priv->mb0_id = can_id;
	ret = count;

 out:
	rtnl_unlock();
	return ret;
}*/

/**********************register dump end***************************/

static int __devinit sunxi_can_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct sunxi_priv *priv;
	struct resource *mem;
	struct clk *clk=NULL, *pclk=NULL;
	void __iomem *addr;
	int err, irq;

	if(!gpio_request_ex("can_para", NULL))
		dev_err(&pdev->dev, "request can gpio failed\n");

	pclk = clk_get(&pdev->dev, "apb_can");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no apb clock defined\n");
		err = -ENODEV;
		goto exit;
	}

	clk = clk_get(&pdev->dev, "can");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		err = -ENODEV;
		clk_put(pclk);
		goto exit;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!mem || irq <= 0) {
		err = -ENODEV;
		goto exit_put;
	}

	if (!request_mem_region(mem->start,
				resource_size(mem),pdev->name)) {
		err = -EBUSY;
		goto exit_put;
	}

	addr = ioremap(mem->start, resource_size(mem));
	if (!addr) {
		err = -ENOMEM;
		goto exit_release;
	}

	dev = alloc_candev(sizeof(struct sunxi_priv), SUNXICAN_ECHO_SKB_MAX);
	if (!dev) {
		err = -ENOMEM;
		goto exit_iounmap;
	}

	dev->netdev_ops	= &sunxi_netdev_ops;
	dev->irq = irq;
	dev->flags |= IFF_ECHO;

	priv = netdev_priv(dev);
	spin_lock_init(&priv->mbx_lock);
	priv->can.clock.freq = clk_get_rate(clk);
	priv->can.bittiming_const = &sunxi_bittiming_const;
	priv->can.do_set_mode = sunxi_set_mode;
	priv->can.do_get_berr_counter = sunxi_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES;
	priv->dev = dev;
	priv->reg_base = addr;
	priv->clk = clk;
	priv->pclk = pclk;

	netif_napi_add(dev, &priv->napi, sunxi_rx_poll, SUNXI_CAN_NAPI_WEIGHT);
	dev->sysfs_groups[0] = &sunxi_sysfs_attr_group;

	dev_set_drvdata(&pdev->dev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	err = register_candev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering netdev failed\n");
		goto exit_free;
	}

	dev_info(&pdev->dev, "device registered (reg_base=%p, irq=%d) clk=%d\n",
		 priv->reg_base, dev->irq, priv->can.clock.freq);

	return 0;

exit_free:
	free_candev(dev);
exit_iounmap:
	iounmap(addr);
exit_release:
	release_mem_region(mem->start, resource_size(mem));
exit_put:
	clk_put(clk);
	clk_put(pclk);
exit:
	return err;
}

static int __devexit sunxi_can_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct sunxi_priv *priv = netdev_priv(dev);
	struct resource *res;

	unregister_netdev(dev);

	platform_set_drvdata(pdev, NULL);

	iounmap(priv->reg_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	clk_put(priv->clk);
	clk_put(priv->pclk);

	free_candev(dev);

	return 0;
}

static struct platform_driver sunxi_can_driver = {
	.probe = sunxi_can_probe,
	.remove = __devexit_p(sunxi_can_remove),
	.driver = {
		.name = "sunxi-can",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(sunxi_can_driver);

MODULE_AUTHOR("threewater <threewaterL@163.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(KBUILD_MODNAME " CAN netdevice driver");
