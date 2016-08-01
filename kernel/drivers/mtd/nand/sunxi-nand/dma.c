/*
 * dma.c
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

#include <linux/wait.h>
#include <linux/sched.h>
#include <asm/cacheflush.h>
#include <plat/dma_compat.h>

#include "defs.h"

#define DMA_HALF_INT_MASK       (1<<0)
#define DMA_END_INT_MASK        (1<<1)

#define NAND_DMA_TIMEOUT 20000 /*20 sec*/

static int nanddma_completed_flag = 1;
static DECLARE_WAIT_QUEUE_HEAD(DMA_wait);

static struct sunxi_dma_params nand_dma = {
	.client.name="NAND_DMA",
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I
	.channel = DMACH_DNAND,
#endif
	.dma_addr = 0x01c03030,
};

static void nanddma_buffdone(struct sunxi_dma_params *dma, void *dev_id)
{
	nanddma_completed_flag = 1;
	wake_up(&DMA_wait);
	//DBG_INFO("buffer done. nanddma_completed_flag: %d\n", nanddma_completed_flag);
}

int dma_nand_request(void)
{
	int r;

	r = sunxi_dma_request(&nand_dma, 1);
	if (r < 0)
		return r;

	r = sunxi_dma_set_callback(&nand_dma, nanddma_buffdone, NULL);
	if (r < 0)
		return r;

	return 0;
}

int dma_nand_release()
{
	sunxi_dma_release(&nand_dma);
	return 0;
}

void dma_nand_config_start(int rw, unsigned int buff_addr, size_t len)
{
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I
	struct dma_hw_conf nand_hwconf = {
		.xfer_type = DMAXFER_D_BWORD_S_BWORD,
		.hf_irq = SW_DMA_IRQ_FULL,
	};

	nand_hwconf.dir = rw + 1;

	if(rw == 0){
		nand_hwconf.from = 0x01C03030,
		nand_hwconf.address_type = DMAADDRT_D_LN_S_IO,
		nand_hwconf.drqsrc_type = DRQ_TYPE_NAND;
	} else {
		nand_hwconf.to = 0x01C03030,
		nand_hwconf.address_type = DMAADDRT_D_IO_S_LN,
		nand_hwconf.drqdst_type = DRQ_TYPE_NAND;
	}

	sw_dma_setflags(DMACH_DNAND, SW_DMAF_AUTOSTART);
#else
	static int dma_started = 0;

	dma_config_t nand_hwconf = {
		.xfer_type.src_data_width	= DATA_WIDTH_32BIT,
		.xfer_type.src_bst_len		= DATA_BRST_4,
		.xfer_type.dst_data_width	= DATA_WIDTH_32BIT,
		.xfer_type.dst_bst_len		= DATA_BRST_4,
		.bconti_mode			= false,
		.irq_spt			= CHAN_IRQ_FD,
	};

	if(rw == 0) {
		nand_hwconf.address_type.src_addr_mode	= DDMA_ADDR_IO; 
		nand_hwconf.address_type.dst_addr_mode	= DDMA_ADDR_LINEAR;
		nand_hwconf.src_drq_type		= D_DST_NAND;
		nand_hwconf.dst_drq_type		= D_DST_SDRAM;
	} else {
		nand_hwconf.address_type.src_addr_mode	= DDMA_ADDR_LINEAR;
		nand_hwconf.address_type.dst_addr_mode	= DDMA_ADDR_IO;
		nand_hwconf.src_drq_type		= D_DST_SDRAM;
		nand_hwconf.dst_drq_type		= D_DST_NAND;
	}
#endif
	sunxi_dma_config(&nand_dma, &nand_hwconf, 0x7f077f07);
	__cpuc_flush_dcache_area((void *)buff_addr, len + (1 << 5) * 2 - 2);
	nanddma_completed_flag = 0;
	sunxi_dma_enqueue(&nand_dma, buff_addr, len, rw == 0);
#if !defined CONFIG_ARCH_SUN4I && !defined CONFIG_ARCH_SUN5I
	/* No auto-start, start manually */
	if (!dma_started) {
		sunxi_dma_start(&nand_dma);
		dma_started = 1;
	}
#endif
}

int dma_nand_wait_finish(void)
{
	int ret = wait_event_timeout(DMA_wait, nanddma_completed_flag, msecs_to_jiffies(NAND_DMA_TIMEOUT));
	if (!ret) {
		ERR_INFO("Dma operation finish timeout\n");
	}
	return ret;
}

