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
 * param.h
 *
 *  Created on: Apr 16, 2014
 *      Author: Eduardo Moscoso Rubino
 */

#ifndef PARAM_H_
#define PARAM_H_

typedef float energy_t;

struct debtEncParam {
	// encode parameters
	int					transform;	// which transform to use
	int					nbands;		// number of decompositions
	int					maxk;		// depth to decompose/partition
	int					dynamic;	// do dynamic decomposition/partitioning
	energy_t			refmul;		// refmul
	energy_t			quality;	// where to truncate the qcell list for all components
	int					resolution;	// order cells by resolution
	int					lowres;		// force resolution independence in the stream
	int					maxsize;	// max amount of bytes in the output
	int					listlen;	// either quality or listlen may be specified (listlen is the only one retrieved)
	// output format parameters
	int					esc;	// ESC or RAW (file format)
	int					map;	// send band x bitplane coding [sig|ref]:1[subband]:2[var length code]
	int					pad;	// pad stream between cells (is always true for esc && map)
	int					dict;	// append dictionary at the end (cell map)
	int					y_uv;	// send while y then uv
	// image data
	int					width;
	int					height;
	int					colorfmt;
	// ROI list
	int					uroi_shift; // amount of right shift to apply to mask
	void *				uroi_list;	// opaque data structure to handle a ROI object list (user ROI)
	// after transformation and roi we get the number of bitplanes
	int					nbitplanes;
	// use vector instructions if possible
	int					vector;
	// statistics
	int					benchmark;
	int					timestats;	// print time statistics
};

struct debtEncParam *debtEncParam_init(struct debtEncParam *p);
struct debtEncParam *debtEncParam_destroy(struct debtEncParam *p);
struct debtEncParam *debtEncParam_fix(struct debtEncParam *p, int maxbands);

struct debtDecParam {
	// decode parameters (all optional)
	int							bitlen;		// the max amount of bits to decode
	int							bias;		// the bias type to use
	float						weight;		// the weight of this bias
};

struct debtDecParam *debtDecParam_init(struct debtDecParam *p);
struct debtDecParam *debtDecParam_destroy(struct debtDecParam *p);

/* user ROI */

void *uroi_list_new(void);
void uroi_list_del(void *uroi);
int uroi_point(void *uroi, int x, int y);
int uroi_rect(void *uroi, int x0, int y0, int x1, int y1);

#endif /* PARAM_H_ */
