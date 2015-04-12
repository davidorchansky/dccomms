/*
 * libezbt.h
 *
 *  Created on: Mar 14, 2012
 *      Author: moscoso
 */

/*
 * This is the simple external API where the caller only knows the ezbt parameters
 */

/*
 * This file should never be included by any internal source and must be in sync with them
 */

#ifndef LIBEZBT_H_
#define LIBEZBT_H_

#define EZBT_MAGIC		0x657a6274	/* ezbt */

/* we should also include colorformat.h */

/* Transform enum */

enum ezbtTransforms {
	EZBT_TRANSFORM_IW53,
	EZBT_TRANSFORM_IW97,
	EZBT_TRANSFORM_IW22,
	EZBT_TRANSFORM_IW42,
	EZBT_TRANSFORM_IW44,
	EZBT_TRANSFORM_RW97,
	EZBT_NTRANSFORMS
};

char *ezbtTransformName(int transform);
int ezbtTransform(char *name);

/* Params struct and API */

/*
 * For compression, the user MUST fill the following fields:
 *   cscan, iscan, sscan, vscan, rscan, xscan, transform, nBands, maxK, flat, blocktree, statK, minK, deep, and insn.
 * All others are filled from other parts of the API.
 */
struct ezbtParams {
	int							width;				// 16 : width
	int							height;				// 16 : height
	int							colorFormat;		//  6 : color format (YUV12, BGR24, RGB24, etc.)

	int							lscan;				//  1 : LL resolution scan and AC
	int							iscan;				//  1 : 0 (LIPS/LSP for each band) or 1 (LIPS/LSP by band)
	int							dscan;				//  1 : diagonal scan

	int							pad;				//  1 : pad each section to a byte boundary to ease random access

	int							lastBand;			//  4 : rectangle number of bands
	int							lastBitplane;		//  4 : rectangle number of bitplanes

	int							zscan;				//  1 : scan the top-left, top-right and bottom-left rectangles. The order of the last 2 rects depends on vscan
	int							vscan;				//  1 : scan the top/bottom (0) or left/right (1) rectangles
	int							rscan;				//  1 : scan the rectangles by resolution (bitplane)
	int							xscan;				//  1 : only scan the top-left rectangle

	int							transform;			//  4 : EZBT_TRANSFORM_XXX
	int							nBands;				//  4 : # of decompositions
	int							maxK;				//  4 : highest level k-zerotree used
	int							flat;				//  4 : partition the top layer of a significant tree in depth "flat" (coeffs if flat >= degree)
	int							blocktree;			//  1 : 0: send the top block significance first - 1: send the subtree significance first
	int							statK;				//  1 : 0: do dynamic repartition - 1: static repartition (never send a repartition bit)
	int							minK;				//  1 : 0: repartition until degree-0 trees - 1: repartition until degree-1 trees
	int							deep;				//  1 : go after a significant set across all bands

	int							insn;				// see mmx.h for the MM_* macros

	int							logmin;				// cilog2(min(w,h))
	int							logmax;				// cilog2(max(w,h))
	int							logq;				// ilog2(max transform coefficient = firstbitplane)
};

/*
 * External API
 */
struct ezbtParams *ezbtInitParams(struct ezbtParams *ezbtp);
struct ezbtParams *ezbtDestroyParams(struct ezbtParams *ezbtp);
struct ezbtParams *ezbtNewParams();
void ezbtDelParams(struct ezbtParams *ezbtp);

/* ImgBuffer struct and API */

struct ezbtImgBuffer {
	unsigned char *		allocBuffer;
	unsigned char *		buffer;
	unsigned char *		ubuffer;
	unsigned char *		vbuffer;
	int					bufferSize; // total buffer size beginning from allocBuffer
	int					colorFormat;
	int					width;
	int					height;
	int					pitch;
	int					uvpitch;
	int					bufferAlignment;
	int					lineAlignment;
};

struct ezbtImgBuffer *ezbtInitImgBuffer(struct ezbtImgBuffer *ib);
struct ezbtImgBuffer *ezbtNewImgBuffer();
struct ezbtImgBuffer *ezbtInitAlignedImgBuffer(struct ezbtImgBuffer *ib, int align);
struct ezbtImgBuffer *ezbtNewAlignedImgBuffer(int align);
struct ezbtImgBuffer *ezbtDestroyImgBuffer(struct ezbtImgBuffer *ib);
void ezbtDelImgBuffer(struct ezbtImgBuffer *ib);
struct ezbtImgBuffer *ezbtCheckImgBuffer(struct ezbtImgBuffer *ib, int width, int height, int colorformat);
struct ezbtImgBuffer *ezbtPrepareImgBuffer(struct ezbtImgBuffer *ib, int width, int height, int colorformat, unsigned char *buffer, int pitch, unsigned char *ubuffer, unsigned char *vbuffer, int uvpitch);
struct ezbtImgBuffer *ezbtPreparePackedImgBuffer(struct ezbtImgBuffer *ib, int width, int height, int colorformat, unsigned char *buffer);
int ezbtPackedImgBufferSize(int width, int height, int colorformat);

/* Encode API */

int ezbtEncodeImg(unsigned char *imagepack, int width, int height, int colorformat, unsigned char **outbufptr, int *outbufsizeptr, int outoffset, struct ezbtParams *ezbtp, int transform, int nbands, int maxsize, int lastband, int lastbitplane);
int ezbtEncodeFixedImg(unsigned char *imagepack, int width, int height, int colorformat, unsigned char *outbuf, int outbufsize, int outoffset, struct ezbtParams *ezbtp, int transform, int nbands, int maxsize, int lastband, int lastbitplane);
int ezbtEncodeImgBuffer(struct ezbtImgBuffer *ib, unsigned char **outbufptr, int *outbufsizeptr, int outoffset, struct ezbtParams *ezbtp, int transform, int nbands, int maxsize, int lastband, int lastbitplane);
int ezbtEncodeFixedImgBuffer(struct ezbtImgBuffer *ib, unsigned char *outbuf, int outbufsize, int outoffset, struct ezbtParams *ezbtp, int transform, int nbands, int maxsize, int lastband, int lastbitplane);
void *ezbtEncodeNewState();
void ezbtEncodeDelState(void *cezbtstate);
int ezbtEncodeStateImgBuffer(void *cezbtstate, struct ezbtImgBuffer *ib, unsigned char **outbufptr, int *outbufsizeptr, int outoffset, struct ezbtParams *ezbtp, int transform, int nbands, int maxsize, int lastband, int lastbitplane);
int ezbtEncodeStateFixedImgBuffer(void *cezbtstate, struct ezbtImgBuffer *ib, unsigned char *outbuf, int outbufsize, int outoffset, struct ezbtParams *ezbtp, int transform, int nbands, int maxsize, int lastband, int lastbitplane);

/* Bias enum */

enum ezbtBias {
	EZBT_BIAS_DEFAULT,
	EZBT_BIAS_ZERO,
	EZBT_BIAS_EXP,
	EZBT_BIAS_MID
};

/* Decode API */

struct ezbtParams *ezbtDecodeParams(unsigned char *cimg, int cimgsize, struct ezbtParams *ezbtp, int *headersize);
unsigned char *ezbtDecodeImg(unsigned char *cimg, int cimgsize, unsigned char *imgpack, int imgpacksize, int nbias);
struct ezbtImgBuffer *ezbtDecodeImgBuffer(unsigned char *cimg, int cimgsize, struct ezbtImgBuffer *ib, int  nbias);

void *ezbtDecodeNewState();
void ezbtDecodeDelState(void *dezbtstate);
int ezbtDecodeSetBias(void *dezbtstate, int bias);
struct ezbtImgBuffer *ezbtDecodeStateImgBuffer(void *dezbtstate, unsigned char *cimg, int cimgsize, struct ezbtImgBuffer *ib);
struct ezbtParams *ezbtDecodeStateParams(void *dezbtstate, struct ezbtParams *dezbtp);

/* Re encode API */

int ezbtRencode(unsigned char **dst, int *dstlen, unsigned char *src, int srclen, struct ezbtParams *ezbtp);
int ezbtRencodeFixed(unsigned char *dst, int dstlen, unsigned char *src, int srclen, struct ezbtParams *ezbtp);

#endif /* LIBEZBT_H_ */
