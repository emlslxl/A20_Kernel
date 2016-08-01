/*
 * drivers/media/video/sun4i_tvd/test/app_tvd.c
 *
 * (C) Copyright 2007-2012
 * threewater
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>
#include <linux/videodev2.h>
#include <time.h>
#include <linux/fb.h>
#include "video.h"

int main(int argc, char** argv)
{
	Video_st2 vst={
		.channel = 0,
		.show_rect = {
			.x = 0, 
			.y = 0,
			.width = 300,
			.height = 200,
		},
		.crop_rect = {
			.x = 0,
			.y = 0,
			.width = 720,
			.height = 576,
		},
	};
	
	video_init(stdout);
	video_start(&vst);
	getchar();
	video_stop();
	
	return 0;
}
