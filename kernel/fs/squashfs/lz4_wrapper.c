/*
 * Squashfs - a compressed read only filesystem for Linux
 *
 * Copyright (c) 2010 LG Electronics
 * Chan Jeong <chan.jeong@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2,
 * or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * lz4_wrapper.c
 */

#include <linux/mutex.h>
#include <linux/buffer_head.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/lz4.h>

#include "squashfs_fs.h"
#include "squashfs_fs_sb.h"
#include "squashfs.h"
#include "decompressor.h"

struct squashfs_lz4 {
	char *input;
	int nbuff;
	int total_buff;
	struct buffer_head **bh;
};

static void *lz4_init(struct squashfs_sb_info *msblk, void *buff, int len)
{
	int block_size = max_t(int, msblk->block_size, SQUASHFS_METADATA_SIZE);

	struct squashfs_lz4 *stream = kzalloc(sizeof(*stream), GFP_KERNEL);

	if (stream == NULL)
		goto failed;
	stream->input = vmalloc(block_size);
	if (stream->input == NULL)
		goto failed;

	return stream;

failed:
	ERROR("Failed to allocate lzo workspace\n");
	kfree(stream);
	return ERR_PTR(-ENOMEM);
}

static void lz4_free(void *strm)
{
	struct squashfs_lz4 *stream = strm;

	if (stream) {
		vfree(stream->input);
	}
	kfree(stream);
}

#if 0
static int dump_data(const char *p, int n)
{
	int i;
	unsigned char d;

	for(i=0;i<n;i++){
		if((i&0xf)==0)
			printk("0x%04x:", i);
		d=p[i];
		printk("%02x%c", d, (i&0xf)==0xf?'\n':',');
	}
	printk("\n");
}
#endif

static int squashfs_lz4_fill(struct lz4_fill_s *fs)
{
	struct squashfs_lz4 *stream = (struct squashfs_lz4 *)fs->priv;
	struct buffer_head *bh;

	if(stream->nbuff >= stream->total_buff)
		return -1;

	bh = stream->bh[stream->nbuff];
	wait_on_buffer(bh);
	if (!buffer_uptodate(bh))
		return -1;

	memcpy(fs->in_next, bh->b_data, PAGE_CACHE_SIZE);
	fs->in_next+=PAGE_CACHE_SIZE;
	put_bh(bh);
	stream->nbuff++;

	return 0;
}

static int lz4_uncompress(struct squashfs_sb_info *msblk, void **buffer,
	struct buffer_head **bh, int b, int offset, int length, int srclength,
	int pages)
{
	int size;
	struct squashfs_lz4 *stream = msblk->stream;
	struct lz4_fill_s fs={
		.in = stream->input,
		.total_in_size = length,
		.out = buffer[0],
		.max_out_size = srclength,
		.fill = squashfs_lz4_fill,
		.priv = stream,
		};

	mutex_lock(&msblk->read_data_mutex);

	wait_on_buffer(*bh);
	if (!buffer_uptodate(*bh))
		goto failed;

	size = PAGE_CACHE_SIZE-offset;
	memcpy(fs.in, bh[0]->b_data+offset, size);
	put_bh(*bh);
	fs.in_next = fs.in+size;
	stream->nbuff = 1;
	stream->total_buff = b;
	stream->bh = bh;

	if(lz4_decompress_fill(&fs)<0)
		goto failed;

	mutex_unlock(&msblk->read_data_mutex);
	return srclength;

failed:
	mutex_unlock(&msblk->read_data_mutex);

	ERROR("lz4 decompression failed, data probably corrupt\n");
	return -EIO;
}

const struct squashfs_decompressor squashfs_lz4_comp_ops = {
	.init = lz4_init,
	.free = lz4_free,
	.decompress = lz4_uncompress,
	.id = LZ4_COMPRESSION,
	.name = "lz4",
	.supported = 1
};
