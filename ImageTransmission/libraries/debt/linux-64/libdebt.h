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
 * libdebt.h
 *
 *  Created on: Apr 16, 2014
 *      Author: Eduardo Moscoso Rubino
 */

#ifndef LIBDEBT_H_
#define LIBDEBT_H_

#include "colorfmt.h"
#include "imgbuffer.h"
#include "param.h"

/*
 * This is the only include file to be used by clients
 */

/*
 * Opaque data structures so advanced clients can use extra functionality with the correct headers
 */

struct encBT;
struct decBT;

struct imgBuffer;
struct outBitBuffer;
struct inBitBuffer;

enum transform_t {
	TRANSFORM_B_CDF9x7,		/* biorthogonal real valued CDF 9/7 (1,2) */
	TRANSFORM_I_B_5x3,		/* Biorthogonal integer valued 5/3 (1,2) */
	TRANSFORM_I_B_9x7M,		/* Biorthogonal integer valued 9/7-M (1,2) */
	TRANSFORM_I_B_13x7T,	/* Biorthogonal integer valued 13/7-T (1,2) */
	TRANSFORM_I_O_2x2,		/* Orthogonal integer valued Haar (1,2) */
	NTRANSFORMS
};

int transform_code(char *name);
char *transform_name(int code);

enum bias_t {
	BIAS_DEF,
	BIAS_ZERO,
	BIAS_MID,
	BIAS_EXP,
	NBIAS
};

int bias_code(char *name);
char *bias_name(int code);

#define MAX_BANDS		15	// max number of bands (0..14)
#define MAX_BITPLANES	15	// max number of bitplanes (0..14)
#define MAX_DEGREES		15	// max number of depths (0..14)
#define NCC				4	// max number of color components (0..3)

/* encode */

struct encBT *encBT_destroy(struct encBT *ebt);
void encBT_del(struct encBT *ebt);
struct encBT *encBT_init(struct encBT *ebt);
struct encBT *encBT_new(void);

int debt_encode(struct encBT *ebt, struct outBitBuffer *obb, struct imgBuffer *img, struct encParam *p);
int debt_encode_imgbuffer(struct imgBuffer *img, unsigned char **outbufptr, int *outbufsizeptr, int fixed, struct encParam *p);
int debt_encode_fixed_imgbuffer(struct imgBuffer *img, unsigned char *outbufptr, int outbufsizeptr, struct encParam *p);
int debt_encode_state_imgbuffer(struct encBT *ebt, struct imgBuffer *img, unsigned char **outbufptr, int *outbufsizeptr, int fixed, struct encParam *p);
int debt_encode_state_fixed_imgbuffer(struct encBT *ebt, struct imgBuffer *img, unsigned char *outbufptr, int outbufsizeptr, struct encParam *p);

/* decode */

struct decBT *decBT_destroy(struct decBT *dbt);
void decBT_del(struct decBT *dbt);
struct decBT *decBT_init(struct decBT *dbt);
struct decBT *decBT_new(void);

int debt_decode(struct decBT *dbt, struct inBitBuffer *ibb, struct imgBuffer *dst, struct decParam *dp, struct encParam *ep, struct imgBuffer *src);
int debt_decode_header(unsigned char *debtimg, int debtimgsize, struct encParam *ep);
struct imgBuffer *debt_decode_imgbuffer(struct imgBuffer *dst, unsigned char *debtimg, int debtimgsize, struct decParam *dp, struct encParam *ep, struct imgBuffer *src);
struct imgBuffer *debt_decode_state_imgbuffer(struct decBT *dbt, struct imgBuffer *dst, unsigned char *debtimg, int debtimgsize, struct decParam *dp, struct encParam *ep, struct imgBuffer *src);

#endif /* LIBDEBT_H_ */
