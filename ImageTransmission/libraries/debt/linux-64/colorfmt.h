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
 * colorfmt.h
 *
 *  Created on: Apr 16, 2014
 *      Author: Eduardo Moscoso Rubino
 */

#ifndef COLORFMT_H_
#define COLORFMT_H_

/*
 * The current header format allows for 1 or 3 planes (and alpha channel) and decimation
 * of the U and V planes along the width and height dimensions.
 *
 * We also allow for 8 or 16 bit samples (unimplemented) with or without an alpha channel (unimplemented)
 *
 * We support the following color formats: (2 bits)
 *
 * 00 - 4:4:4 (24 bits) (3 planes)
 * 01 - 4:2:2 (16 bits) (3 planes)
 * 10 - 4:2:0 (12 bits)	(3 planes)
 * 11 - 4:0:0 (8 bits - grey) (1 plane)
 *
 * We support the following color standards: (3 bits)
 *
 * BT.601 digital (limited swing Y 16-235 U,V 16-240)
 * BT.601 analog (full swing)
 * BT.709 digital (limited swing Y 16-235 U,V 16-240)
 * BT.709 analog (full swing)
 * Lossless RGB->YUV (unimplemented)
 * user-defined (unknown)
 *
 * Alpha channel (unimplemented) (1 bit)
 *
 * 16-bit samples (unimplemented) (1 bit)
 *
 * Therefore, we need 7 bits to represent the color format: but we add an initial bit of zero to allow for
 * future expansion
 *
 * bit 7 - 0
 * bit 6 - 0 = 8 bit, 1 = 16 bit
 * bit 5 - 0 = no alpha, 1 = alpha
 * bits 4,3,2 = color standard
 * 	 000 - BT.601 digital
 * 	 001 - BT.601 analog
 * 	 010 - BT.709 digital
 * 	 011 - BT.709 analog
 * 	 100 - Lossless (to be defined)
 * 	 ... - reserved
 * 	 111 - user defined
 *
 * bits 1,0 = color format
 *   00 - 4:4:4 (24 bits) (3 planes)
 *   01 - 4:2:2 (16 bits) (3 planes)
 *   10 - 4:2:0 (12 bits) (3 planes)
 *   11 - 4:0:0 (8 bits - grey) (1 plane)
 *
 * FIXME:
 * Our current implementation implements codes from 00000000b to 00001111b (601 and 709 digital and analog)
 * 00011100b to 00011111b (user defined YUV conversion). We don't support neither alpha channel nor 16 bit
 * samples for now.
 */

#define COLORFORMAT_STD_BT601_DIGITAL	0
#define COLORFORMAT_STD_BT601_ANALOG	1
#define COLORFORMAT_STD_BT709_DIGITAL	2
#define COLORFORMAT_STD_BT709_ANALOG	3
#define COLORFORMAT_STD_LOSSLESS		4	/* YCoCg-R - but the image should be input in RGB and converted (8 bit Y, 9 bit Co and Cg) */
#define COLORFORMAT_STD_UNKNOWN			7

#define COLORFORMAT_FMT_444				0
#define COLORFORMAT_FMT_422				1
#define COLORFORMAT_FMT_420				2
#define COLORFORMAT_FMT_GREY			3

static inline int
newColorFormat(int std, int fmt)
{
	return ((std & 7) << 2) | (fmt & 3);
}

static inline int
colorFormatFmt(int cf)
{
	return (cf & 3);
}

static inline int
colorFormatStd(int cf)
{
	return ((cf >> 2) & 7);
}

static inline int
colorFormatAlpha(int cf)
{
	return ((cf >> 5) & 1);
}

static inline int
colorFormatExt(int cf)
{
	return ((cf >> 6) & 1);
}

/* returns if we can compress or not the data - not necessarily that we can convert it to RGB */
static inline int
colorFormatSupport(int cf)
{
	return ((cf & 0x60) == 0);
}

/* FIXME: allow more standards */
static inline int
colorFormatSupportRGB(int cf)
{
	return (cf >= 0x00 && cf <= 0x0f);
}

static inline int
colorFormatGrey(int cf)
{
	return ((cf & 3) == 3);
}

static inline int
colorFormatColor(int cf)
{
	return ((cf & 3) != 3);
}

static inline int
colorFormatWdiv(int cf)
{
	static int wdivtab[4] = {1, 2, 2, 1};
	return wdivtab[cf & 3];
}

static inline int
colorFormatHdiv(int cf)
{
	static int hdivtab[4] = {1, 1, 2, 1};
	return hdivtab[cf & 3];
}

#endif  /* COLORFMT_H_ */
