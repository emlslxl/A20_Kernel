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
 * lzo_wrapper.c
 */

#include <linux/mutex.h>
#include <linux/buffer_head.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/lzo.h>

#include "squashfs_fs.h"
#include "squashfs_fs_sb.h"
#include "squashfs.h"
#include "decompressor.h"

struct squashfs_lzo {
	struct page **input_mp;
};

static void *lzo_init(struct squashfs_sb_info *msblk, void *buff, int len)
{
	int block_size = max_t(int, msblk->block_size, SQUASHFS_METADATA_SIZE);

	struct squashfs_lzo *stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	int mpsize = (block_size>>msblk->devblksize_log2)*sizeof(struct page **);

	if (stream == NULL)
		goto failed;
	stream->input_mp = kmalloc(mpsize, GFP_KERNEL);
	if (stream->input_mp == NULL)
		goto failed;

	return stream;

failed:
	ERROR("Failed to allocate lzo workspace\n");
	kfree(stream);
	return ERR_PTR(-ENOMEM);
}


static void lzo_free(void *strm)
{
	struct squashfs_lzo *stream = strm;

	if (stream) {
		kfree(stream->input_mp);
	}
	kfree(stream);
}


static int lzo_uncompress(struct squashfs_sb_info *msblk, void **buffer,
	struct buffer_head **bh, int b, int offset, int length, int srclength,
	int pages)
{
	struct squashfs_lzo *stream = msblk->stream;
	int i, res;
	size_t out_len = srclength;
	void *in, *out;

	mutex_lock(&msblk->read_data_mutex);

	for (i = 0; i < b; i++) {
		wait_on_buffer(bh[i]);
		if (!buffer_uptodate(bh[i]))
			goto block_release;

		stream->input_mp[i]=virt_to_page(bh[i]->b_data);
	}

	//map to contigous virtrual memory
	in = vmap(stream->input_mp, b, VM_MAP, PAGE_KERNEL);
	out = buffer[0];

	res = lzo1x_decompress_safe(in+offset, (size_t)length,
					out, &out_len);
	if (res != LZO_E_OK)
		goto failed;

	for (i = 0; i < b; i++) {
		put_bh(bh[i]);
	}

	vunmap(in);

	res = (int)out_len;

	mutex_unlock(&msblk->read_data_mutex);
	return res;

block_release:
	for (; i < b; i++)
		put_bh(bh[i]);

failed:
	mutex_unlock(&msblk->read_data_mutex);

	ERROR("lzo decompression failed, data probably corrupt\n");
	return -EIO;
}

const struct squashfs_decompressor squashfs_lzo_comp_ops = {
	.init = lzo_init,
	.free = lzo_free,
	.decompress = lzo_uncompress,
	.id = LZO_COMPRESSION,
	.name = "lzo",
	.supported = 1
};
