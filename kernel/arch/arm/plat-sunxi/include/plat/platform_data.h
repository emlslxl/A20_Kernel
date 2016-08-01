/*
 * arch/arm/plat-sunxi/include/plat/platform_data.h
 *
 * (C) Copyright 2007-2012
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

#ifndef __SW_PLATFORM_DATA_H
#define __SW_PLATFORM_DATA_H

struct sw_keypad_platdata {
        const struct matrix_keymap_data *keymap_data;
        unsigned int rows;
        unsigned int cols;
        bool no_autorepeat;
};

#endif

