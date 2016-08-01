/*
 * arch/arm/plat-sunxi/include/plat/uncompress.h
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Benn Huang <benn@allwinnertech.com>
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

#ifndef __SW_UNCOMPRESS_H
#define __SW_UNCOMPRESS_H

#include <linux/serial_reg.h>
#include <mach/platform.h>

#define UART_BASE       SW_PA_UART0_IO_BASE
#define UARTx_BASE(x)   (UART_BASE + (x) * 0x400)
#define UART_REGSHIFT	2

static inline void putc(int c)
{
#ifdef CONFIG_DEBUG_LL
	volatile u8 * uart_base = (volatile u8 *)(UARTx_BASE(CONFIG_SW_DEBUG_UART));

	while (!(uart_base[UART_LSR << UART_REGSHIFT] & UART_LSR_THRE))
		barrier();
	uart_base[UART_TX << UART_REGSHIFT] = c;
#endif
}

static inline void flush(void)
{
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()

#endif


