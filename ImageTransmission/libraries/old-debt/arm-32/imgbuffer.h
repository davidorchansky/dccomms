/*
 * Copyright (c) 2014, Eduardo Moscoso Rubino
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS `AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * imgbuffer.h
 *
 *  Created on: Apr 16, 2014
 *      Author: Eduardo Moscoso Rubino
 */

#ifndef IMGBUFFER_H_
#define IMGBUFFER_H_

struct imgBuffer {
	unsigned char *		allocBuffer;
	unsigned char *		buffer;
	unsigned char *		ubuffer;
	unsigned char *		vbuffer;
	int					bufferSize; // total buffer size beginning from allocBuffer
	int					colorFormat;
	int					width;
	int					height;
	int					pitch;
	int					uvwidth;
	int					uvheight;
	int					uvpitch;
	int					bufferAlignment;
	int					lineAlignment;
};

struct imgBuffer *imgBuffer_init(struct imgBuffer *ib);
struct imgBuffer *imgBuffer_new(void);
struct imgBuffer *imgBuffer_aligned_init(struct imgBuffer *ib, int align);
struct imgBuffer *imgBuffer_aligned_new(int align);
struct imgBuffer *imgBuffer_destroy(struct imgBuffer *ib);
void imgBuffer_del(struct imgBuffer *ib);
struct imgBuffer *imgBuffer_reinit(struct imgBuffer *ib, int width, int height, int colorformat);
struct imgBuffer *imgBuffer_prepare(struct imgBuffer *ib, int width, int height, int colorformat, unsigned char *buffer, int pitch, unsigned char *ubuffer, unsigned char *vbuffer, int uvwidth, int uvheight, int uvpitch);
struct imgBuffer *imgBuffer_packed_prepare(struct imgBuffer *ib, int width, int height, int colorformat, unsigned char *buffer);
int imgBuffer_packed_size(int width, int height, int colorformat);
struct imgBuffer *imgBuffer_clear(struct imgBuffer *ib);

#endif /* IMGBUFFER_H_ */
