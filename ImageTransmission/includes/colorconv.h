/*
 * colorconv.h
 *
 *  Created on: May 16, 2012
 *      Author: moscoso
 */

#ifndef COLORCONV_H_
#define COLORCONV_H_

#define CC_HAAR		0	/* haar wavelet (average) */
#define CC_FILT		1	/* [1,2,1] vertical filter */
#define CC_WAVE		2	/* CDF 5/3 vertical low pass */

extern char *cc_interpolation[];

void yuyv2yuv(int cc, unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h, int insn);
void yuv_analog2digital(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, int w, int h, int insn);
void yuv_digital2analog(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, int w, int h, int insn);

#endif /* COLORCONV_H_ */
