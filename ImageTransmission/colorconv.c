/*
 * colorconv.c
 *
 *  Created on: May 16, 2012
 *      Author: moscoso
 */

/*
 * We do packed bigendian yuyv to planar yuv12 color conversion
 */

#include <colorconv.h>
//#include "../../../../lib/simd.h"

#include <string.h>

char *cc_interpolation[] = {
	"haar",
	"filter",
	"wavelet",
	NULL
};

inline
static int
saturate(int x)
{
	return (x < 0) ? 0 : (x > 255) ? 255 : x;
}

/* copy only the y values */
inline
static void
yuyv2y_line_c(unsigned char *y, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		yuyv += 4;
	}
}

/* copy the y, u, and v values */
inline
static void
yuyv2yuv_copy_line_c(unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = yuyv[1];
		*v++ = yuyv[3];
		yuyv += 4;
	}
}

/* copy only the y components from the 2 lines and the u and v components from the low pass of the haar transform of the 2 lines */
inline
static void
yuyv2yuv_haar_lines_c(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = (pyuyv[1] + yuyv[1]) >> 1;
		*v++ = (pyuyv[3] + yuyv[3]) >> 1;
		pyuyv += 4;
		yuyv += 4;
	}
}

/* haar */
inline
static void
yuyv2yuv_haar_c(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	/* convert from YUYV to YUV12 */
	unsigned char *l0 = yuyv;
	while (h > 1) {
		unsigned char *l1 = l0 + yuyvpitch;
		yuyv2yuv_haar_lines_c(y, y + ypitch, u, v, l0, l1, w);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l1 + yuyvpitch;
		h -= 2;
	}
	if (h > 0) {
		yuyv2yuv_copy_line_c(y, u, v, l0, w);
	}
}

/* [1,2,1] vertical filter */
/* pyuyv is the previous line and nyuyv is the next line */
inline
static void
yuyv2yuv_filter_lines_c(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = (pyuyv[1] + (yuyv[1] << 1) + nyuyv[1] + 2) >> 2;
		*v++ = (pyuyv[3] + (yuyv[3] << 1) + nyuyv[3] + 2) >> 2;
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

/* [1,2,1] vertical filter */
inline
static void
yuyv2yuv_filter_c(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	unsigned char *l0 = yuyv;
	while (h > 2) {
		unsigned char *l1 = l0 + yuyvpitch;
		unsigned char *l2 = l1 + yuyvpitch;
		yuyv2yuv_filter_lines_c(y, y + ypitch, u, v, l0, l1, l2, w);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l2;
		h -= 2;
	}
	if (h > 1) {
		yuyv2yuv_filter_lines_c(y, y + ypitch, u, v, l0, l0 + yuyvpitch, l0, w);
	} else if (h > 0) {
		yuyv2yuv_copy_line_c(y, u, v, l0, w);
	}
}

inline
static void
yuyv2yuv_cdf53_first_lines_c(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		int iu = yuyv[1] - ((pyuyv[1] + nyuyv[1]) >> 1);
		int iv = yuyv[3] - ((pyuyv[3] + nyuyv[3]) >> 1);
		*u++ = saturate(pyuyv[1] + (((*uv++ = iu) + 1) >> 1));
		*v++ = saturate(pyuyv[3] + (((*uv++ = iv) + 1) >> 1));
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

inline
static void
yuyv2yuv_cdf53_last_odd_line_c(unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *yuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = saturate(yuyv[1] + ((*uv++ + 1) >> 1));
		*v++ = saturate(yuyv[3] + ((*uv++ + 1) >> 1));
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_cdf53_lines_c(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		int iu = yuyv[1] - ((pyuyv[1] + nyuyv[1]) >> 1);
		int iv = yuyv[3] - ((pyuyv[3] + nyuyv[3]) >> 1);
		*u++ = saturate(pyuyv[1] + ((*uv + iu + 2) >> 2));
		*uv++ = iu;
		*v++ = saturate(pyuyv[3] + ((*uv + iv + 2) >> 2));
		*uv++ = iv;
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

/* we are using a [-1/8 1/4 3/4 1/4 -1/8] vertical filter : Le Gall 5/3 wavelet low pass (lifting) */
inline
static void
yuyv2yuv_cdf53_c(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	/* convert from YUYV to YUV12 */
	if (h > 1) {
		short uv[w];
		unsigned char *l0 = yuyv;
		unsigned char *l1 = l0 + yuyvpitch;
		unsigned char *l2 = (h > 2) ? l1 + yuyvpitch : l0;
		yuyv2yuv_cdf53_first_lines_c(y, y + ypitch, u, v, l0, l1, l2, w, uv);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l2;
		while ((h -= 2) > 2) {
			l2 = (l1 = l0 + yuyvpitch) + yuyvpitch;
			yuyv2yuv_cdf53_lines_c(y, y + ypitch, u, v, l0, l1, l2, w, uv);
			y += ypitch << 1;
			u += uvpitch;
			v += uvpitch;
			l0 = l2;
		}
		if (h > 1) {
			l1 = l0 + yuyvpitch;
			yuyv2yuv_cdf53_lines_c(y, y + ypitch, u, v, l0, l1, l0, w, uv);
		} else if (h > 0) {
			yuyv2yuv_cdf53_last_odd_line_c(y, u, v, l0, w, uv);
		}
	} else if (h > 0) {
		yuyv2yuv_copy_line_c(y, u, v, yuyv, w);
	}
}

#if defined(ARCH_x86_32) || defined(ARCH_x86_64)

static const sse_t vc_wxff		__attribute__((aligned (16))) =	{.uq[0] = 0x00ff00ff00ff00ffULL, .uq[1] = 0x00ff00ff00ff00ffULL};
static const sse_t vc_w1		__attribute__((aligned (16))) =	{.uq[0] = 0x0001000100010001ULL, .uq[1] = 0x0001000100010001ULL};
static const sse_t vc_w2		__attribute__((aligned (16))) =	{.uq[0] = 0x0002000200020002ULL, .uq[1] = 0x0002000200020002ULL};

#define DD(p)	(*((uint32_t *)(p)))
#define MM(p)   (*((mmx_t *)(p)))

#define yuyv2y_mmx16(src,y,a,b)     \
	movq_m2r(MM(src), a);           \
	movq_m2r(MM(src+8), b);         \
	pand_m2r(vc_wxff.uq[0], a);     \
	pand_m2r(vc_wxff.uq[0], b);     \
	packuswb_r2r(b, a);             \
	movq_r2m(a, MM(y))

#define yuyv2y_mmx32(src,y,a,b,c,d) \
	movq_m2r(MM(src), a);           \
	movq_m2r(MM(src+8), b);         \
	movq_m2r(MM(src+16), c);        \
	movq_m2r(MM(src+24), d);        \
	pand_m2r(vc_wxff.uq[0], a);     \
	pand_m2r(vc_wxff.uq[0], b);     \
	pand_m2r(vc_wxff.uq[0], c);     \
	pand_m2r(vc_wxff.uq[0], d);     \
	packuswb_r2r(b, a);             \
	packuswb_r2r(d, c);             \
	movq_r2m(a, MM(y));             \
	movq_r2m(c, MM(y+8))

#define yuyv2yuvs_mmx16(src,y,u,v,a,b,c,d)	\
	movq_m2r(MM(src), a);					\
	movq_m2r(MM(src+8), b);					\
	movq_r2r(a, c);							\
	movq_r2r(b, d);							\
	pand_m2r(vc_wxff.uq[0], a);				\
	pand_m2r(vc_wxff.uq[0], b);				\
	psrlw_i2r(8, c);						\
	psrlw_i2r(8, d);						\
	packuswb_r2r(b, a);						\
	packuswb_r2r(d, c);						\
	movq_r2m(a, MM(y));						\
	movq_r2r(c, d);							\
	pand_m2r(vc_wxff.uq[0], c);				\
	psrlw_i2r(8, d);						\
	packuswb_r2r(c, c);						\
	packuswb_r2r(d, d);						\
	movd_r2m(c, DD(u));						\
	movd_r2m(d, DD(v))

#define yuyv2yuvs_mmx32(src,y,u,v,a,b,c,d,e,f,g,h)	\
	movq_m2r(MM(src), a);							\
	movq_m2r(MM(src+8), b);							\
	movq_m2r(MM(src+16), c);						\
	movq_m2r(MM(src+24), d);						\
	movq_r2r(a, e);									\
	movq_r2r(b, f);									\
	movq_r2r(c, g);									\
	movq_r2r(d, h);									\
	pand_m2r(vc_wxff.uq[0], a);						\
	pand_m2r(vc_wxff.uq[0], b);						\
	pand_m2r(vc_wxff.uq[0], c);						\
	pand_m2r(vc_wxff.uq[0], d);						\
	psrlw_i2r(8, e);								\
	psrlw_i2r(8, f);								\
	psrlw_i2r(8, g);								\
	psrlw_i2r(8, h);								\
	packuswb_r2r(b, a);								\
	packuswb_r2r(d, c);								\
	packuswb_r2r(f, e);								\
	packuswb_r2r(h, g);								\
	movq_r2m(a, MM(y));								\
	movq_r2m(c, MM(y+8));							\
	movq_r2r(e, f);									\
	movq_r2r(g, h);									\
	pand_m2r(vc_wxff.uq[0], e);						\
	psrlw_i2r(8, f);								\
	pand_m2r(vc_wxff.uq[0], g);						\
	psrlw_i2r(8, h);								\
	packuswb_r2r(g, e);								\
	packuswb_r2r(h, f);								\
	movq_r2m(e, MM(u));								\
	movq_r2m(f, MM(v))

#define yuyv2yuv_haar_mmx16(psrc,src,py,y,u,v,a,b,c,d,e,f,g,h)		\
	movq_m2r(MM(psrc), a);											\
	movq_m2r(MM(psrc+8), b);										\
	movq_m2r(MM(src), c);											\
	movq_m2r(MM(src+8), d);											\
	movq_r2r(a, e);													\
	movq_r2r(b, f);													\
	movq_r2r(c, g);													\
	movq_r2r(d, h);													\
	pand_m2r(vc_wxff.uq[0], a);										\
	pand_m2r(vc_wxff.uq[0], b);										\
	pand_m2r(vc_wxff.uq[0], c);										\
	pand_m2r(vc_wxff.uq[0], d);										\
	psrlw_i2r(8, e);												\
	psrlw_i2r(8, f);												\
	psrlw_i2r(8, g);												\
	psrlw_i2r(8, h);												\
	packuswb_r2r(b, a);												\
	packuswb_r2r(d, c);												\
	movq_r2m(a, MM(py));											\
	movq_r2m(c, MM(y));												\
	paddw_r2r(g, e);												\
	paddw_r2r(h, f);												\
	psrlw_i2r(1, e);												\
	psrlw_i2r(1, f);												\
	packuswb_r2r(f, e);			/* a = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movq_r2r(e, f);				/* b = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff.uq[0], e);	/* a =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, f);			/* b =    v3    v2    v1    v0 */	\
	packuswb_r2r(e, e);			/* a = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(f, f);			/* b = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movd_r2m(e, DD(u));												\
	movd_r2m(f, DD(v))

#define yuyv2yuv_filt_mmx16(psrc,src,nsrc,py,y,u,v,a,b,c,d,e,f,g,h)	\
	movq_m2r(MM(psrc), a);											\
	movq_m2r(MM(psrc+8), b);										\
	movq_m2r(MM(src), c);											\
	movq_m2r(MM(src+8), d);											\
	movq_r2r(a, e);													\
	movq_r2r(b, f);													\
	movq_r2r(c, g);													\
	movq_r2r(d, h);													\
	pand_m2r(vc_wxff.uq[0], e);										\
	pand_m2r(vc_wxff.uq[0], f);										\
	pand_m2r(vc_wxff.uq[0], g);										\
	pand_m2r(vc_wxff.uq[0], h);										\
	packuswb_r2r(f, e);												\
	packuswb_r2r(h, g);												\
	movq_r2m(e, MM(py));											\
	movq_r2m(g, MM(y));												\
	movq_m2r(MM(nsrc), e);											\
	movq_m2r(MM(nsrc+8), f);										\
	psrlw_i2r(8, a);												\
	psrlw_i2r(8, b);												\
	psrlw_i2r(8, c);												\
	psrlw_i2r(8, d);												\
	psrlw_i2r(8, e);												\
	psrlw_i2r(8, f);												\
	psllw_i2r(1, c);												\
	psllw_i2r(1, d);												\
	paddw_r2r(e, a);												\
	paddw_r2r(f, b);												\
	paddw_r2r(c, a);												\
	paddw_r2r(d, b);												\
	paddw_m2r(vc_w2.uq[0], a);										\
	paddw_m2r(vc_w2.uq[0], b);										\
	psrlw_i2r(2, a);												\
	psrlw_i2r(2, b);												\
	packuswb_r2r(b, a);			/* a = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movq_r2r(a, b);				/* b = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff.uq[0], a);	/* a =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, b);			/* b =    v3    v2    v1    v0 */	\
	packuswb_r2r(a, a);			/* a = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(b, b);			/* b = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movd_r2m(a, DD(u));												\
	movd_r2m(b, DD(v))

#define yuyv2yuv_cdf53_first_mmx16(psrc,src,nsrc,py,y,u,v,uv,a,b,c,d,e,f,g,h)	\
	movq_m2r(MM(psrc), a);											\
	movq_m2r(MM(psrc+8), b);										\
	movq_m2r(MM(src), c);											\
	movq_m2r(MM(src+8), d);											\
	movq_r2r(a, e);													\
	movq_r2r(b, f);													\
	movq_r2r(c, g);													\
	movq_r2r(d, h);													\
	pand_m2r(vc_wxff.uq[0], e);										\
	pand_m2r(vc_wxff.uq[0], f);										\
	pand_m2r(vc_wxff.uq[0], g);										\
	pand_m2r(vc_wxff.uq[0], h);										\
	packuswb_r2r(f, e);												\
	packuswb_r2r(h, g);												\
	movq_r2m(e, MM(py));											\
	movq_r2m(g, MM(y));												\
	movq_m2r(MM(nsrc), e);											\
	movq_m2r(MM(nsrc+8), f);										\
	psrlw_i2r(8, a);												\
	psrlw_i2r(8, b);												\
	psrlw_i2r(8, c);												\
	psrlw_i2r(8, d);												\
	psrlw_i2r(8, e);												\
	psrlw_i2r(8, f);												\
	paddw_r2r(a, e);												\
	paddw_r2r(b, f);												\
	psrlw_i2r(1, e);												\
	psrlw_i2r(1, f);												\
	psubw_r2r(e, c);												\
	psubw_r2r(f, d);												\
	movq_r2m(c, MM(uv));											\
	movq_r2m(d, MM(uv+8));											\
	paddw_m2r(vc_w1.uq[0], c);										\
	paddw_m2r(vc_w1.uq[0], d);										\
	psrlw_i2r(1, c);												\
	psrlw_i2r(1, d);												\
	paddw_r2r(c, a);												\
	paddw_r2r(d, b);												\
	packuswb_r2r(b, a);			/* a = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movq_r2r(a, b);				/* b = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff.uq[0], a);	/* a =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, b);			/* b =    v3    v2    v1    v0 */	\
	packuswb_r2r(a, a);			/* a = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(b, b);			/* b = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movd_r2m(a, DD(u));												\
	movd_r2m(b, DD(v))

#define yuyv2yuv_cdf53_last_mmx16(src,y,u,v,uv,a,b,c,d)				\
	movq_m2r(MM(src), a);											\
	movq_m2r(MM(src+8), b);											\
	movq_r2r(a, c);													\
	movq_r2r(b, d);													\
	pand_m2r(vc_wxff.uq[0], c);										\
	pand_m2r(vc_wxff.uq[0], d);										\
	packuswb_r2r(d, c);												\
	movq_r2m(c, MM(y));												\
	psrlw_i2r(8, a);												\
	psrlw_i2r(8, b);												\
	movq_m2r(MM(uv), c);											\
	movq_m2r(MM(uv+8), d);											\
	paddw_m2r(vc_w1.uq[0], c);										\
	paddw_m2r(vc_w1.uq[0], d);										\
	psrlw_i2r(1, c);												\
	psrlw_i2r(1, d);												\
	paddw_r2r(c, a);												\
	paddw_r2r(d, b);												\
	packuswb_r2r(b, a);			/* a = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movq_r2r(a, b);				/* b = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff.uq[0], a);	/* a =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, b);			/* b =    v3    v2    v1    v0 */	\
	packuswb_r2r(a, a);			/* a = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(b, b);			/* b = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movd_r2m(a, DD(u));												\
	movd_r2m(b, DD(v))

#define yuyv2yuv_cdf53_mmx16(psrc,src,nsrc,py,y,u,v,uv,a,b,c,d,e,f,g,h)	\
	movq_m2r(MM(psrc), a);											\
	movq_m2r(MM(psrc+8), b);										\
	movq_m2r(MM(src), c);											\
	movq_m2r(MM(src+8), d);											\
	movq_r2r(a, e);													\
	movq_r2r(b, f);													\
	movq_r2r(c, g);													\
	movq_r2r(d, h);													\
	pand_m2r(vc_wxff.uq[0], e);										\
	pand_m2r(vc_wxff.uq[0], f);										\
	pand_m2r(vc_wxff.uq[0], g);										\
	pand_m2r(vc_wxff.uq[0], h);										\
	packuswb_r2r(f, e);												\
	packuswb_r2r(h, g);												\
	movq_r2m(e, MM(py));											\
	movq_r2m(g, MM(y));												\
	movq_m2r(MM(nsrc), e);											\
	movq_m2r(MM(nsrc+8), f);										\
	psrlw_i2r(8, a);												\
	psrlw_i2r(8, b);												\
	psrlw_i2r(8, c);												\
	psrlw_i2r(8, d);												\
	psrlw_i2r(8, e);												\
	psrlw_i2r(8, f);												\
	paddw_r2r(a, e);												\
	paddw_r2r(b, f);												\
	psrlw_i2r(1, e);												\
	psrlw_i2r(1, f);												\
	psubw_r2r(e, c);												\
	psubw_r2r(f, d);												\
	movq_m2r(MM(uv), e);											\
	movq_m2r(MM(uv+8), f);											\
	movq_r2m(c, MM(uv));											\
	movq_r2m(d, MM(uv+8));											\
	paddw_r2r(c, e);												\
	paddw_r2r(d, f);												\
	paddw_m2r(vc_w2.uq[0], e);										\
	paddw_m2r(vc_w2.uq[0], f);										\
	psrlw_i2r(2, e);												\
	psrlw_i2r(2, f);												\
	paddw_r2r(e, a);												\
	paddw_r2r(f, b);												\
	packuswb_r2r(b, a);			/* a = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movq_r2r(a, b);				/* b = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff.uq[0], a);	/* a =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, b);			/* b =    v3    v2    v1    v0 */	\
	packuswb_r2r(a, a);			/* a = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(b, b);			/* b = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movd_r2m(a, DD(u));												\
	movd_r2m(b, DD(v))

inline
static void
yuyv2y_line_mmx(unsigned char *y, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w - 16; y <= yend; y += 16) {
		yuyv2y_mmx32(yuyv, y, mm0, mm1, mm2, mm3);
		yuyv += 32;
	}
	for (yend += 8; y <= yend; y += 8) {
		yuyv2y_mmx16(yuyv, y, mm0, mm1);
		yuyv += 16;
	}
	for (yend += 8; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_copy_line_mmx(unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w - 16; y <= yend; y += 16) {
		yuyv2yuvs_mmx32(yuyv, y, u, v, mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7);
		u += 8;
		v += 8;
		yuyv += 32;
	}
	for (yend += 8; y <= yend; y += 8) {
		yuyv2yuvs_mmx16(yuyv, y, u, v, mm0, mm1, mm2, mm3);
		u += 4;
		v += 4;
		yuyv += 16;
	}
	for (yend += 8; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = yuyv[1];
		*v++ = yuyv[3];
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_haar_lines_mmx(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w - 8; y <= yend; py += 8, y += 8) {
		yuyv2yuv_haar_mmx16(pyuyv, yuyv, py, y, u, v, mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7);
		u += 4;
		v += 4;
		pyuyv += 16;
		yuyv += 16;
	}
	for (yend += 8; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = (pyuyv[1] + yuyv[1]) >> 1;
		*v++ = (pyuyv[3] + yuyv[3]) >> 1;
		pyuyv += 4;
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_haar_mmx(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	/* convert from YUYV to YUV12 */
	unsigned char *l0 = yuyv;
	while (h > 1) {
		unsigned char *l1 = l0 + yuyvpitch;
		yuyv2yuv_haar_lines_mmx(y, y + ypitch, u, v, l0, l1, w);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l1 + yuyvpitch;
		h -= 2;
	}
	if (h > 0) {
		yuyv2yuv_copy_line_mmx(y, u, v, l0, w);
	}
}

inline
static void
yuyv2yuv_filter_lines_mmx(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w - 8; y <= yend; py += 8, y += 8) {
		yuyv2yuv_filt_mmx16(pyuyv, yuyv, nyuyv, py, y, u, v, mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7);
		u += 4;
		v += 4;
		pyuyv += 16;
		yuyv += 16;
		nyuyv += 16;
	}
	for (yend += 8; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = (pyuyv[1] + (yuyv[1] << 1) + nyuyv[1] + 2) >> 2;
		*v++ = (pyuyv[3] + (yuyv[3] << 1) + nyuyv[3] + 2) >> 2;
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

/* [1,2,1] vertical filter */
inline
static void
yuyv2yuv_filter_mmx(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	unsigned char *l0 = yuyv;
	while (h > 2) {
		unsigned char *l1 = l0 + yuyvpitch;
		unsigned char *l2 = l1 + yuyvpitch;
		yuyv2yuv_filter_lines_mmx(y, y + ypitch, u, v, l0, l1, l2, w);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l2;
		h -= 2;
	}
	if (h > 1) {
		yuyv2yuv_filter_lines_mmx(y, y + ypitch, u, v, l0, l0 + yuyvpitch, l0, w);
	} else if (h > 0) {
		yuyv2yuv_copy_line_mmx(y, u, v, l0, w);
	}
}

inline
static void
yuyv2yuv_cdf53_first_lines_mmx(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w - 8; y <= yend; py += 8, y += 8) {
		yuyv2yuv_cdf53_first_mmx16(pyuyv, yuyv, nyuyv, py, y, u, v, uv, mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7);
		u += 4;
		v += 4;
		uv += 8;
		pyuyv += 16;
		yuyv += 16;
		nyuyv += 16;
	}
	for (yend += 8; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		int iu = yuyv[1] - ((pyuyv[1] + nyuyv[1]) >> 1);
		int iv = yuyv[3] - ((pyuyv[3] + nyuyv[3]) >> 1);
		*u++ = saturate(pyuyv[1] + (((*uv++ = iu) + 1) >> 1));
		*v++ = saturate(pyuyv[3] + (((*uv++ = iv) + 1) >> 1));
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

inline
static void
yuyv2yuv_cdf53_last_odd_line_mmx(unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *yuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w - 8; y <= yend; y += 8) {
		yuyv2yuv_cdf53_last_mmx16(yuyv, y, u, v, uv, mm0, mm1, mm2, mm3);
		u += 4;
		v += 4;
		uv += 8;
		yuyv += 16;
	}
	for (yend += 8; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = saturate(yuyv[1] + ((*uv++ + 1) >> 1));
		*v++ = saturate(yuyv[3] + ((*uv++ + 1) >> 1));
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_cdf53_lines_mmx(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w - 8; y <= yend; py += 8, y += 8) {
		yuyv2yuv_cdf53_mmx16(pyuyv, yuyv, nyuyv, py, y, u, v, uv, mm0, mm1, mm2, mm3, mm4, mm5, mm6, mm7);
		u += 4;
		v += 4;
		uv += 8;
		pyuyv += 16;
		yuyv += 16;
		nyuyv += 16;
	}
	for (yend += 8; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		int iu = yuyv[1] - ((pyuyv[1] + nyuyv[1]) >> 1);
		int iv = yuyv[3] - ((pyuyv[3] + nyuyv[3]) >> 1);
		*u++ = saturate(pyuyv[1] + ((*uv + iu + 2) >> 2));
		*uv++ = iu;
		*v++ = saturate(pyuyv[3] + ((*uv + iv + 2) >> 2));
		*uv++ = iv;
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

/* we are using a [-1/8 1/4 3/4 1/4 -1/8] vertical filter : Le Gall 5/3 wavelet low pass (lifting) */
inline
static void
yuyv2yuv_cdf53_mmx(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	/* convert from YUYV to YUV12 */
	if (h > 1) {
		short uv[w];
		unsigned char *l0 = yuyv;
		unsigned char *l1 = l0 + yuyvpitch;
		unsigned char *l2 = (h > 2) ? l1 + yuyvpitch : l0;
		yuyv2yuv_cdf53_first_lines_mmx(y, y + ypitch, u, v, l0, l1, l2, w, uv);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l2;
		while ((h -= 2) > 2) {
			l2 = (l1 = l0 + yuyvpitch) + yuyvpitch;
			yuyv2yuv_cdf53_lines_mmx(y, y + ypitch, u, v, l0, l1, l2, w, uv);
			y += ypitch << 1;
			u += uvpitch;
			v += uvpitch;
			l0 = l2;
		}
		if (h > 1) {
			l1 = l0 + yuyvpitch;
			yuyv2yuv_cdf53_lines_mmx(y, y + ypitch, u, v, l0, l1, l0, w, uv);
		} else if (h > 0) {
			yuyv2yuv_cdf53_last_odd_line_mmx(y, u, v, l0, w, uv);
		}
	} else if (h > 0) {
		yuyv2yuv_copy_line_mmx(y, u, v, yuyv, w);
	}
}

#define SS(p)   (*((sse_t *)(p)))

#define yuyv2y_sse32(src,y,a,b)     \
	movdqu_m2r(SS(src), a);         \
	movdqu_m2r(SS(src+16), b);      \
	pand_m2r(vc_wxff, a);           \
	pand_m2r(vc_wxff, b);           \
	packuswb_r2r(b, a);             \
	movdqu_r2m(a, SS(y))

#define yuyv2y_sse64(src,y,a,b,c,d) \
	movdqu_m2r(SS(src), a);         \
	movdqu_m2r(SS(src+16), b);      \
	movdqu_m2r(SS(src+32), c);      \
	movdqu_m2r(SS(src+48), d);      \
	pand_m2r(vc_wxff, a);           \
	pand_m2r(vc_wxff, b);           \
	pand_m2r(vc_wxff, c);           \
	pand_m2r(vc_wxff, d);           \
	packuswb_r2r(b, a);             \
	packuswb_r2r(d, c);             \
	movdqu_r2m(a, SS(y));           \
	movdqu_r2m(c, SS(y+16))

#define yuyv2yuvs_sse32(src,y,u,v,a,b,c,d)	\
	movdqu_m2r(SS(src), a);					\
	movdqu_m2r(SS(src+16), b);				\
	movdqa_r2r(a, c);						\
	movdqa_r2r(b, d);						\
	pand_m2r(vc_wxff, a);					\
	pand_m2r(vc_wxff, b);					\
	psrlw_i2r(8, c);						\
	psrlw_i2r(8, d);						\
	packuswb_r2r(b, a);						\
	packuswb_r2r(d, c);						\
	movdqu_r2m(a, SS(y));					\
	movdqa_r2r(c, d);						\
	pand_m2r(vc_wxff, c);					\
	psrlw_i2r(8, d);						\
	packuswb_r2r(c, c);						\
	packuswb_r2r(d, d);						\
	movq_r2m(c, MM(u));						\
	movq_r2m(d, MM(v))

#define yuyv2yuvs_sse64(src,y,u,v,a,b,c,d,e,f,g,h)	\
	movdqu_m2r(SS(src), a);							\
	movdqu_m2r(SS(src+16), b);						\
	movdqu_m2r(SS(src+32), c);						\
	movdqu_m2r(SS(src+48), d);						\
	movdqa_r2r(a, e);								\
	movdqa_r2r(b, f);								\
	movdqa_r2r(c, g);								\
	movdqa_r2r(d, h);								\
	pand_m2r(vc_wxff, a);							\
	pand_m2r(vc_wxff, b);							\
	pand_m2r(vc_wxff, c);							\
	pand_m2r(vc_wxff, d);							\
	psrlw_i2r(8, e);								\
	psrlw_i2r(8, f);								\
	psrlw_i2r(8, g);								\
	psrlw_i2r(8, h);								\
	packuswb_r2r(b, a);								\
	packuswb_r2r(d, c);								\
	packuswb_r2r(f, e);								\
	packuswb_r2r(h, g);								\
	movdqu_r2m(a, SS(y));							\
	movdqu_r2m(c, SS(y+16));						\
	movdqa_r2r(e, f);								\
	movdqa_r2r(g, h);								\
	pand_m2r(vc_wxff, e);							\
	psrlw_i2r(8, f);								\
	pand_m2r(vc_wxff, g);							\
	psrlw_i2r(8, h);								\
	packuswb_r2r(g, e);								\
	packuswb_r2r(h, f);								\
	movdqu_r2m(e, MM(u));							\
	movdqu_r2m(f, MM(v))

#define yuyv2yuv_haar_sse32(psrc,src,py,y,u,v,a,b,c,d,e,f,g,h)		\
	movdqu_m2r(SS(psrc), a);										\
	movdqu_m2r(SS(psrc+16), b);										\
	movdqu_m2r(SS(src), c);											\
	movdqu_m2r(SS(src+16), d);										\
	movdqa_r2r(a, e);												\
	movdqa_r2r(b, f);												\
	movdqa_r2r(c, g);												\
	movdqa_r2r(d, h);												\
	pand_m2r(vc_wxff, a);											\
	pand_m2r(vc_wxff, b);											\
	pand_m2r(vc_wxff, c);											\
	pand_m2r(vc_wxff, d);											\
	psrlw_i2r(8, e);												\
	psrlw_i2r(8, f);												\
	psrlw_i2r(8, g);												\
	psrlw_i2r(8, h);												\
	packuswb_r2r(b, a);												\
	packuswb_r2r(d, c);												\
	movdqu_r2m(a, SS(py));											\
	movdqu_r2m(c, SS(y));											\
	paddw_r2r(g, e);												\
	paddw_r2r(h, f);												\
	psrlw_i2r(1, e);												\
	psrlw_i2r(1, f);												\
	packuswb_r2r(f, e);			/* e = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movdqa_r2r(e, f);			/* f = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff, e);		/* e =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, f);			/* f =    v3    v2    v1    v0 */	\
	packuswb_r2r(e, e);			/* e = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(f, f);			/* f = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movq_r2m(e, MM(u));												\
	movq_r2m(f, MM(v))

#define yuyv2yuv_filt_sse32(psrc,src,nsrc,py,y,u,v,a,b,c,d,e,f,g,h)	\
	movdqu_m2r(SS(psrc), a);										\
	movdqu_m2r(SS(psrc+16), b);										\
	movdqu_m2r(SS(src), c);											\
	movdqu_m2r(SS(src+16), d);										\
	movdqa_r2r(a, e);												\
	movdqa_r2r(b, f);												\
	movdqa_r2r(c, g);												\
	movdqa_r2r(d, h);												\
	pand_m2r(vc_wxff, e);											\
	pand_m2r(vc_wxff, f);											\
	pand_m2r(vc_wxff, g);											\
	pand_m2r(vc_wxff, h);											\
	packuswb_r2r(f, e);												\
	packuswb_r2r(h, g);												\
	movdqu_r2m(e, SS(py));											\
	movdqu_r2m(g, SS(y));											\
	movdqu_m2r(SS(nsrc), e);										\
	movdqu_m2r(SS(nsrc+16), f);										\
	psrlw_i2r(8, a);												\
	psrlw_i2r(8, b);												\
	psrlw_i2r(8, c);												\
	psrlw_i2r(8, d);												\
	psrlw_i2r(8, e);												\
	psrlw_i2r(8, f);												\
	psllw_i2r(1, c);												\
	psllw_i2r(1, d);												\
	paddw_r2r(e, a);												\
	paddw_r2r(f, b);												\
	paddw_r2r(c, a);												\
	paddw_r2r(d, b);												\
	paddw_m2r(vc_w2, a);											\
	paddw_m2r(vc_w2, b);											\
	psrlw_i2r(2, a);												\
	psrlw_i2r(2, b);												\
	packuswb_r2r(b, a);		/* a = v3 u3 v2 u2 v1 u1 v0 u0 */		\
	movdqa_r2r(a, b);		/* b = v3 u3 v2 u2 v1 u1 v0 u0 */		\
	pand_m2r(vc_wxff, a);	/* a =    u3    u2    u1    u0 */		\
	psrlw_i2r(8, b);		/* b =    v3    v2    v1    v0 */		\
	packuswb_r2r(a, a);		/* a = u3 u2 u1 u0 u3 u2 u1 u0 */		\
	packuswb_r2r(b, b);		/* b = v3 v2 v1 v0 v3 v2 v1 v0 */		\
	movq_r2m(a, MM(u));												\
	movq_r2m(b, MM(v))

#define yuyv2yuv_cdf53_first_sse32(psrc,src,nsrc,py,y,u,v,uv,a,b,c,d,e,f,g,h)	\
	movdqu_m2r(SS(psrc), a);										\
	movdqu_m2r(SS(psrc+16), b);										\
	movdqu_m2r(SS(src), c);											\
	movdqu_m2r(SS(src+16), d);										\
	movdqa_r2r(a, e);												\
	movdqa_r2r(b, f);												\
	movdqa_r2r(c, g);												\
	movdqa_r2r(d, h);												\
	pand_m2r(vc_wxff, e);											\
	pand_m2r(vc_wxff, f);											\
	pand_m2r(vc_wxff, g);											\
	pand_m2r(vc_wxff, h);											\
	packuswb_r2r(f, e);												\
	packuswb_r2r(h, g);												\
	movdqu_r2m(e, SS(py));											\
	movdqu_r2m(g, SS(y));											\
	movdqu_m2r(SS(nsrc), e);										\
	movdqu_m2r(SS(nsrc+16), f);										\
	psrlw_i2r(8, a);												\
	psrlw_i2r(8, b);												\
	psrlw_i2r(8, c);												\
	psrlw_i2r(8, d);												\
	psrlw_i2r(8, e);												\
	psrlw_i2r(8, f);												\
	paddw_r2r(a, e);												\
	paddw_r2r(b, f);												\
	psrlw_i2r(1, e);												\
	psrlw_i2r(1, f);												\
	psubw_r2r(e, c);												\
	psubw_r2r(f, d);												\
	movdqu_r2m(c, SS(uv));											\
	movdqu_r2m(d, SS(uv+16));										\
	paddw_m2r(vc_w1, c);											\
	paddw_m2r(vc_w1, d);											\
	psrlw_i2r(1, c);												\
	psrlw_i2r(1, d);												\
	paddw_r2r(c, a);												\
	paddw_r2r(d, b);												\
	packuswb_r2r(b, a);			/* a = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movdqa_r2r(a, b);			/* b = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff.uq[0], a);	/* a =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, b);			/* b =    v3    v2    v1    v0 */	\
	packuswb_r2r(a, a);			/* a = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(b, b);			/* b = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movq_r2m(a, MM(u));												\
	movq_r2m(b, MM(v))

#define yuyv2yuv_cdf53_last_sse32(src,y,u,v,uv,a,b,c,d)				\
	movdqu_m2r(SS(src), a);											\
	movdqu_m2r(SS(src+16), b);										\
	movdqa_r2r(a, c);												\
	movdqa_r2r(b, d);												\
	pand_m2r(vc_wxff, c);											\
	pand_m2r(vc_wxff, d);											\
	packuswb_r2r(d, c);												\
	movdqu_r2m(c, SS(y));											\
	psrlw_i2r(8, a);												\
	psrlw_i2r(8, b);												\
	movdqu_m2r(SS(uv), c);											\
	movdqu_m2r(SS(uv+16), d);										\
	paddw_m2r(vc_w1, c);											\
	paddw_m2r(vc_w1, d);											\
	psrlw_i2r(1, c);												\
	psrlw_i2r(1, d);												\
	paddw_r2r(c, a);												\
	paddw_r2r(d, b);												\
	packuswb_r2r(b, a);			/* a = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movdqa_r2r(a, b);			/* b = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff, a);		/* a =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, b);			/* b =    v3    v2    v1    v0 */	\
	packuswb_r2r(a, a);			/* a = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(b, b);			/* b = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movq_r2m(a, MM(u));												\
	movq_r2m(b, MM(v))

#define yuyv2yuv_cdf53_sse32(psrc,src,nsrc,py,y,u,v,uv,a,b,c,d,e,f,g,h)	\
	movdqu_m2r(SS(psrc), a);										\
	movdqu_m2r(SS(psrc+16), b);										\
	movdqu_m2r(SS(src), c);											\
	movdqu_m2r(SS(src+16), d);										\
	movdqa_r2r(a, e);												\
	movdqa_r2r(b, f);												\
	movdqa_r2r(c, g);												\
	movdqa_r2r(d, h);												\
	pand_m2r(vc_wxff, e);											\
	pand_m2r(vc_wxff, f);											\
	pand_m2r(vc_wxff, g);											\
	pand_m2r(vc_wxff, h);											\
	packuswb_r2r(f, e);												\
	packuswb_r2r(h, g);												\
	movdqu_r2m(e, SS(py));											\
	movdqu_r2m(g, SS(y));											\
	movdqu_m2r(SS(nsrc), e);										\
	movdqu_m2r(SS(nsrc+16), f);										\
	psrlw_i2r(8, a);												\
	psrlw_i2r(8, b);												\
	psrlw_i2r(8, c);												\
	psrlw_i2r(8, d);												\
	psrlw_i2r(8, e);												\
	psrlw_i2r(8, f);												\
	paddw_r2r(a, e);												\
	paddw_r2r(b, f);												\
	psrlw_i2r(1, e);												\
	psrlw_i2r(1, f);												\
	psubw_r2r(e, c);												\
	psubw_r2r(f, d);												\
	movdqu_m2r(SS(uv), e);											\
	movdqu_m2r(SS(uv+16), f);										\
	movdqu_r2m(c, SS(uv));											\
	movdqu_r2m(d, SS(uv+16));										\
	paddw_r2r(c, e);												\
	paddw_r2r(d, f);												\
	paddw_m2r(vc_w2.uq[0], e);										\
	paddw_m2r(vc_w2.uq[0], f);										\
	psrlw_i2r(2, e);												\
	psrlw_i2r(2, f);												\
	paddw_r2r(e, a);												\
	paddw_r2r(f, b);												\
	packuswb_r2r(b, a);			/* a = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	movdqa_r2r(a, b);			/* b = v3 u3 v2 u2 v1 u1 v0 u0 */	\
	pand_m2r(vc_wxff.uq[0], a);	/* a =    u3    u2    u1    u0 */	\
	psrlw_i2r(8, b);			/* b =    v3    v2    v1    v0 */	\
	packuswb_r2r(a, a);			/* a = u3 u2 u1 u0 u3 u2 u1 u0 */	\
	packuswb_r2r(b, b);			/* b = v3 v2 v1 v0 v3 v2 v1 v0 */	\
	movq_r2m(a, MM(u));												\
	movq_r2m(b, MM(v))

inline
static void
yuyv2y_line_sse(unsigned char *y, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w - 32; y <= yend; y += 32) {
		yuyv2y_sse64(yuyv, y, xmm0, xmm1, xmm2, xmm3);
		yuyv += 64;
	}
	for (yend += 16; y <= yend; y += 16) {
		yuyv2y_sse32(yuyv, y, xmm0, xmm1);
		yuyv += 32;
	}
	for (yend += 16; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_copy_line_sse(unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w - 32; y <= yend; y += 32) {
		yuyv2yuvs_sse64(yuyv, y, u, v, xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7);
		yuyv += 64;
		v += 16;
		u += 16;
	}
	for (yend += 16; y <= yend; y += 16) {
		yuyv2yuvs_sse32(yuyv, y, u, v, xmm0, xmm1, xmm2, xmm3);
		yuyv += 32;
		v += 8;
		u += 8;
	}
	for (yend += 16; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = yuyv[1];
		*v++ = yuyv[3];
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_haar_lines_sse(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w - 16; y <= yend; py += 16, y += 16) {
		yuyv2yuv_haar_sse32(pyuyv, yuyv, py, y, u, v, xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7);
		u += 8;
		v += 8;
		pyuyv += 32;
		yuyv += 32;
	}
	for (yend += 16; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = (pyuyv[1] + yuyv[1]) >> 1;
		*v++ = (pyuyv[3] + yuyv[3]) >> 1;
		pyuyv += 4;
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_haar_sse(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	/* convert from YUYV to YUV12 */
	unsigned char *l0 = yuyv;
	while (h > 1) {
		unsigned char *l1 = l0 + yuyvpitch;
		yuyv2yuv_haar_lines_sse(y, y + ypitch, u, v, l0, l1, w);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l1 + yuyvpitch;
		h -= 2;
	}
	if (h > 0) {
		yuyv2yuv_copy_line_sse(y, u, v, l0, w);
	}
}

inline
static void
yuyv2yuv_filter_lines_sse(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w)
{
	unsigned char *yend;
	for (yend = y + w - 16; y <= yend; py += 16, y += 16) {
		yuyv2yuv_filt_sse32(pyuyv, yuyv, nyuyv, py, y, u, v, xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7);
		u += 8;
		v += 8;
		pyuyv += 32;
		yuyv += 32;
		nyuyv += 32;
	}
	for (yend += 16; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = (pyuyv[1] + (yuyv[1] << 1) + nyuyv[1] + 2) >> 2;
		*v++ = (pyuyv[3] + (yuyv[3] << 1) + nyuyv[3] + 2) >> 2;
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

/* [1,2,1] vertical filter */
inline
static void
yuyv2yuv_filter_sse(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	unsigned char *l0 = yuyv;
	while (h > 2) {
		unsigned char *l1 = l0 + yuyvpitch;
		unsigned char *l2 = l1 + yuyvpitch;
		yuyv2yuv_filter_lines_sse(y, y + ypitch, u, v, l0, l1, l2, w);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l2;
		h -= 2;
	}
	if (h > 1) {
		yuyv2yuv_filter_lines_sse(y, y + ypitch, u, v, l0, l0 + yuyvpitch, l0, w);
	} else if (h > 0) {
		yuyv2yuv_copy_line_sse(y, u, v, l0, w);
	}
}

inline
static void
yuyv2yuv_cdf53_first_lines_sse(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w - 16; y <= yend; py += 16, y += 16) {
		yuyv2yuv_cdf53_first_sse32(pyuyv, yuyv, nyuyv, py, y, u, v, uv, xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7);
		u += 8;
		v += 8;
		uv += 16;
		pyuyv += 32;
		yuyv += 32;
		nyuyv += 32;
	}
	for (yend += 16; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		int iu = yuyv[1] - ((pyuyv[1] + nyuyv[1]) >> 1);
		int iv = yuyv[3] - ((pyuyv[3] + nyuyv[3]) >> 1);
		*u++ = saturate(pyuyv[1] + (((*uv++ = iu) + 1) >> 1));
		*v++ = saturate(pyuyv[3] + (((*uv++ = iv) + 1) >> 1));
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

inline
static void
yuyv2yuv_cdf53_last_odd_line_sse(unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *yuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w - 16; y <= yend; y += 16) {
		yuyv2yuv_cdf53_last_sse32(yuyv, y, u, v, uv, xmm0, xmm1, xmm2, xmm3);
		u += 8;
		v += 8;
		uv += 16;
		yuyv += 32;
	}
	for (yend += 16; y < yend; y += 2) {
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		*u++ = saturate(yuyv[1] + ((*uv++ + 1) >> 1));
		*v++ = saturate(yuyv[3] + ((*uv++ + 1) >> 1));
		yuyv += 4;
	}
}

inline
static void
yuyv2yuv_cdf53_lines_sse(unsigned char *py, unsigned char *y, unsigned char *u, unsigned char *v, unsigned char *pyuyv, unsigned char *yuyv, unsigned char *nyuyv, int w, short *uv)
{
	unsigned char *yend;
	for (yend = y + w - 16; y <= yend; py += 16, y += 16) {
		yuyv2yuv_cdf53_sse32(pyuyv, yuyv, nyuyv, py, y, u, v, uv, xmm0, xmm1, xmm2, xmm3, xmm4, xmm5, xmm6, xmm7);
		u += 8;
		v += 8;
		uv += 16;
		pyuyv += 32;
		yuyv += 32;
		nyuyv += 32;
	}
	for (yend += 16; y < yend; py += 2, y += 2) {
		py[0] = pyuyv[0];
		py[1] = pyuyv[2];
		y[0] = yuyv[0];
		y[1] = yuyv[2];
		int iu = yuyv[1] - ((pyuyv[1] + nyuyv[1]) >> 1);
		int iv = yuyv[3] - ((pyuyv[3] + nyuyv[3]) >> 1);
		*u++ = saturate(pyuyv[1] + ((*uv + iu + 2) >> 2));
		*uv++ = iu;
		*v++ = saturate(pyuyv[3] + ((*uv + iv + 2) >> 2));
		*uv++ = iv;
		pyuyv += 4;
		yuyv += 4;
		nyuyv += 4;
	}
}

/* we are using a [-1/8 1/4 3/4 1/4 -1/8] vertical filter : Le Gall 5/3 wavelet low pass (lifting) */
inline
static void
yuyv2yuv_cdf53_sse(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h)
{
	/* convert from YUYV to YUV12 */
	if (h > 1) {
		short uv[w];
		unsigned char *l0 = yuyv;
		unsigned char *l1 = l0 + yuyvpitch;
		unsigned char *l2 = (h > 2) ? l1 + yuyvpitch : l0;
		yuyv2yuv_cdf53_first_lines_sse(y, y + ypitch, u, v, l0, l1, l2, w, uv);
		y += ypitch << 1;
		u += uvpitch;
		v += uvpitch;
		l0 = l2;
		while ((h -= 2) > 2) {
			l2 = (l1 = l0 + yuyvpitch) + yuyvpitch;
			yuyv2yuv_cdf53_lines_sse(y, y + ypitch, u, v, l0, l1, l2, w, uv);
			y += ypitch << 1;
			u += uvpitch;
			v += uvpitch;
			l0 = l2;
		}
		if (h > 1) {
			l1 = l0 + yuyvpitch;
			yuyv2yuv_cdf53_lines_sse(y, y + ypitch, u, v, l0, l1, l0, w, uv);
		} else if (h > 0) {
			yuyv2yuv_cdf53_last_odd_line_sse(y, u, v, l0, w, uv);
		}
	} else if (h > 0) {
		yuyv2yuv_copy_line_sse(y, u, v, yuyv, w);
	}
}

#endif

void
yuyv2yuv(int cc, unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, unsigned char *yuyv, int yuyvpitch, int w, int h, int insn)
{
#if defined(ARCH_x86_32) || defined(ARCH_x86_64)
	if (insn == -1)
		insn = mmsupport();
	if (insn & MM_SSE2) {
		switch (cc) {
		case CC_HAAR:
		default:
			yuyv2yuv_haar_sse(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
		case CC_FILT:
			yuyv2yuv_filter_sse(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
		case CC_WAVE:
			yuyv2yuv_cdf53_sse(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
			break;
		}
	} else if (insn & MM_MMX) {
		switch (cc) {
		case CC_HAAR:
		default:
			yuyv2yuv_haar_mmx(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
		case CC_FILT:
			yuyv2yuv_filter_mmx(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
		case CC_WAVE:
			yuyv2yuv_cdf53_mmx(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
			break;
		}
		emms();
	} else
#endif
	{
		switch (cc) {
		case CC_HAAR:
		default:
			yuyv2yuv_haar_c(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
		case CC_FILT:
			yuyv2yuv_filter_c(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
		case CC_WAVE:
			yuyv2yuv_cdf53_c(y, ypitch, u, v, uvpitch, yuyv, yuyvpitch, w, h); break;
		}
	}
}

/*
  Rescales the Y values from the range 0..255 to the range 16..235
  Rescales the UV values from the range 0..255 to the range 16..240
  @param yp: buffer for Y plane of decoded JPEG
  @param up: buffer for U plane of decoded JPEG
  @param vp: buffer for V plane of decoded JPEG

  We transform the discrete values into continuous values by adding
  0.5 to the discrete value and then transform the rescaled continuous
  value to discrete values by truncating (integer part or floor) the
  continuous value.
*/
inline
static void
jpgy2y_line_c(unsigned char *y, int w)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; ++y) {
		*y = (*y * 220 + 4206) >> 8;
	}
}

inline
static void
jpguv2uv_line_c(unsigned char *u, unsigned char *v, int w)
{
	unsigned char *vend;
	for (vend = v + w; v < vend; ++u, ++v) {
		*u = (*u * 225 + 4208) >> 8; // has the same value as *u = (*u * 450 + 8417) >> 9; for 0..255
		*v = (*v * 225 + 4208) >> 8; // has the same value as *v = (*v * 450 + 8417) >> 9; for 0..255
	}
}

inline
static void
jpgyuv2yuv_c(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, int w, int h)
{
	int hh;
	for (hh = h; hh; --hh) {
		jpgy2y_line_c(y, w);
		y += ypitch;
	}
	int ww = w >> 1;
	for (hh = h >> 1; hh; --hh) {
		jpguv2uv_line_c(u, v, ww);
		u += uvpitch;
		v += uvpitch;
	}
}

#if defined(ARCH_x86_32) || defined(ARCH_x86_64)

static const sse_t vc_220  __attribute__((aligned (16))) = {.uq[0] = 0x00dc00dc00dc00dcULL, .uq[1] = 0x00dc00dc00dc00dcULL};
static const sse_t vc_4206 __attribute__((aligned (16))) = {.uq[0] = 0x106e106e106e106eULL, .uq[1] = 0x106e106e106e106eULL};

static const sse_t vc_225  __attribute__((aligned (16))) = {.uq[0] = 0x00e100e100e100e1ULL, .uq[1] = 0x00e100e100e100e1ULL};
static const sse_t vc_4208 __attribute__((aligned (16))) = {.uq[0] = 0x1070107010701070ULL, .uq[1] = 0x1070107010701070ULL};

inline
static void
jpgy2y_line_mmx(unsigned char *y, int w)
{
	unsigned char *yend;
	pxor_r2r(mm4, mm4);
	for (yend = y + w - 16; y <= yend; y += 16) {
		movq_m2r(MM(y), mm0);
		movq_m2r(MM(y+8), mm2);
		movq_r2r(mm0, mm1);
		movq_r2r(mm2, mm3);
		punpcklbw_r2r(mm4, mm0);
		punpckhbw_r2r(mm4, mm1);
		punpcklbw_r2r(mm4, mm2);
		punpckhbw_r2r(mm4, mm3);
		pmullw_m2r(vc_220.m[0], mm0);
		pmullw_m2r(vc_220.m[0], mm1);
		pmullw_m2r(vc_220.m[0], mm2);
		pmullw_m2r(vc_220.m[0], mm3);
		paddw_m2r(vc_4206.m[0], mm0);
		paddw_m2r(vc_4206.m[0], mm1);
		paddw_m2r(vc_4206.m[0], mm2);
		paddw_m2r(vc_4206.m[0], mm3);
		psrlw_i2r(8, mm0);
		psrlw_i2r(8, mm1);
		psrlw_i2r(8, mm2);
		psrlw_i2r(8, mm3);
		packuswb_r2r(mm1, mm0);
		packuswb_r2r(mm3, mm2);
		movq_r2m(mm0, MM(y));
		movq_r2m(mm2, MM(y+8));
	}
	for (yend += 8; y <= yend; y += 8) {
		movq_m2r(MM(y), mm0);
		movq_r2r(mm0, mm1);
		punpcklbw_r2r(mm4, mm0);
		punpckhbw_r2r(mm4, mm1);
		pmullw_m2r(vc_220.m[0], mm0);
		pmullw_m2r(vc_220.m[0], mm1);
		paddw_m2r(vc_4206.m[0], mm0);
		paddw_m2r(vc_4206.m[0], mm1);
		psrlw_i2r(8, mm0);
		psrlw_i2r(8, mm1);
		packuswb_r2r(mm1, mm0);
		movq_r2m(mm0, MM(y));
	}
	for (yend += 8; y < yend; ++y) {
		*y = (*y * 220 + 4206) >> 8;
	}
}

inline
static void
jpguv2uv_line_mmx(unsigned char *u, unsigned char *v, int w)
{
	unsigned char *vend;
	pxor_r2r(mm4, mm4);
	for (vend = v + w - 8; v <= vend; u += 8, v += 8) {
		movq_m2r(MM(u), mm0);
		movq_m2r(MM(v), mm2);
		movq_r2r(mm0, mm1);
		movq_r2r(mm2, mm3);
		punpcklbw_r2r(mm4, mm0);
		punpckhbw_r2r(mm4, mm1);
		punpcklbw_r2r(mm4, mm2);
		punpckhbw_r2r(mm4, mm3);
		pmullw_m2r(vc_225.m[0], mm0);
		pmullw_m2r(vc_225.m[0], mm1);
		pmullw_m2r(vc_225.m[0], mm2);
		pmullw_m2r(vc_225.m[0], mm3);
		paddw_m2r(vc_4208.m[0], mm0);
		paddw_m2r(vc_4208.m[0], mm1);
		paddw_m2r(vc_4208.m[0], mm2);
		paddw_m2r(vc_4208.m[0], mm3);
		psrlw_i2r(8, mm0);
		psrlw_i2r(8, mm1);
		psrlw_i2r(8, mm2);
		psrlw_i2r(8, mm3);
		packuswb_r2r(mm1, mm0);
		packuswb_r2r(mm3, mm2);
		movq_r2m(mm0, MM(u));
		movq_r2m(mm2, MM(v));
	}
	for (vend += 8; v < vend; ++u, ++v) {
		*u = (*u * 225 + 4208) >> 8; // has the same value as *u = (*u * 450 + 8417) >> 9; for 0..255
		*v = (*v * 225 + 4208) >> 8; // has the same value as *v = (*v * 450 + 8417) >> 9; for 0..255
	}
}

inline
static void
jpgyuv2yuv_mmx(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, int w, int h)
{
	int hh;
	for (hh = h; hh; --hh) {
		jpgy2y_line_mmx(y, w);
		y += ypitch;
	}
	int ww = w >> 1;
	for (hh = h >> 1; hh; --hh) {
		jpguv2uv_line_mmx(u, v, ww);
		u += uvpitch;
		v += uvpitch;
	}
}

inline
static void
jpgy2y_line_sse(unsigned char *y, int w)
{
	unsigned char *yend;
	pxor_r2r(xmm4, xmm4);
	for (yend = y + w - 32; y <= yend; y += 32) {
		movdqu_m2r(SS(y), xmm0);
		movdqu_m2r(SS(y+16), xmm2);
		movdqa_r2r(xmm0, xmm1);
		movdqa_r2r(xmm2, xmm3);
		punpcklbw_r2r(xmm4, xmm0);
		punpckhbw_r2r(xmm4, xmm1);
		punpcklbw_r2r(xmm4, xmm2);
		punpckhbw_r2r(xmm4, xmm3);
		pmullw_m2r(vc_220, xmm0);
		pmullw_m2r(vc_220, xmm1);
		pmullw_m2r(vc_220, xmm2);
		pmullw_m2r(vc_220, xmm3);
		paddw_m2r(vc_4206, xmm0);
		paddw_m2r(vc_4206, xmm1);
		paddw_m2r(vc_4206, xmm2);
		paddw_m2r(vc_4206, xmm3);
		psrlw_i2r(8, xmm0);
		psrlw_i2r(8, xmm1);
		psrlw_i2r(8, xmm2);
		psrlw_i2r(8, xmm3);
		packuswb_r2r(xmm1, xmm0);
		packuswb_r2r(xmm3, xmm2);
		movdqu_r2m(xmm0, SS(y));
		movdqu_r2m(xmm2, SS(y+16));
	}
	for (yend += 16; y <= yend; y += 16) {
		movdqu_m2r(SS(y), xmm0);
		movdqa_r2r(xmm0, xmm1);
		punpcklbw_r2r(xmm4, xmm0);
		punpckhbw_r2r(xmm4, xmm1);
		pmullw_m2r(vc_220, xmm0);
		pmullw_m2r(vc_220, xmm1);
		paddw_m2r(vc_4206, xmm0);
		paddw_m2r(vc_4206, xmm1);
		psrlw_i2r(8, xmm0);
		psrlw_i2r(8, xmm1);
		packuswb_r2r(xmm1, xmm0);
		movdqu_r2m(xmm0, SS(y));
	}
	for (yend += 16; y < yend; ++y) {
		*y = (*y * 220 + 4206) >> 8;
	}
}

inline
static void
jpguv2uv_line_sse(unsigned char *u, unsigned char *v, int w)
{
	unsigned char *vend;
	pxor_r2r(xmm4, xmm4);
	for (vend = v + w - 16; v <= vend; u += 16, v += 16) {
		movdqu_m2r(SS(u), xmm0);
		movdqu_m2r(SS(v), xmm2);
		movdqa_r2r(xmm0, xmm1);
		movdqa_r2r(xmm2, xmm3);
		punpcklbw_r2r(xmm4, xmm0);
		punpckhbw_r2r(xmm4, xmm1);
		punpcklbw_r2r(xmm4, xmm2);
		punpckhbw_r2r(xmm4, xmm3);
		pmullw_m2r(vc_225, xmm0);
		pmullw_m2r(vc_225, xmm1);
		pmullw_m2r(vc_225, xmm2);
		pmullw_m2r(vc_225, xmm3);
		paddw_m2r(vc_4208, xmm0);
		paddw_m2r(vc_4208, xmm1);
		paddw_m2r(vc_4208, xmm2);
		paddw_m2r(vc_4208, xmm3);
		psrlw_i2r(8, xmm0);
		psrlw_i2r(8, xmm1);
		psrlw_i2r(8, xmm2);
		psrlw_i2r(8, xmm3);
		packuswb_r2r(xmm1, xmm0);
		packuswb_r2r(xmm3, xmm2);
		movdqu_r2m(xmm0, SS(u));
		movdqu_r2m(xmm2, SS(v));
	}
	for (vend += 16; v < vend; ++u, ++v) {
		*u = (*u * 225 + 4208) >> 8; // has the same value as *u = (*u * 450 + 8417) >> 9; for 0..255
		*v = (*v * 225 + 4208) >> 8; // has the same value as *v = (*v * 450 + 8417) >> 9; for 0..255
	}
}

inline
static void
jpgyuv2yuv_sse(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, int w, int h)
{
	int hh;
	for (hh = h; hh; --hh) {
		jpgy2y_line_sse(y, w);
		y += ypitch;
	}
	int ww = w >> 1;
	for (hh = h >> 1; hh; --hh) {
		jpguv2uv_line_sse(u, v, ww);
		u += uvpitch;
		v += uvpitch;
	}
}

#endif

void
yuv_analog2digital(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, int w, int h, int insn)
{
#if defined(ARCH_x86_32) || defined(ARCH_x86_64)
	if (insn == -1)
		insn = mmsupport();
	if (insn & MM_SSE2) {
		jpgyuv2yuv_sse(y, ypitch, u, v, uvpitch, w, h);
	} else if (insn & MM_MMX) {
		jpgyuv2yuv_mmx(y, ypitch, u, v, uvpitch, w, h);
		emms();
	} else
#endif
	{
		jpgyuv2yuv_c(y, ypitch, u, v, uvpitch, w, h);
	}
}

/*
  Rescales the Y values from the range 16..235 to the range 0..255
  Rescales the UV values from the range 16..240 to the range 0..255
  @param yp: buffer for Y plane of decoded JPEG
  @param up: buffer for U plane of decoded JPEG
  @param vp: buffer for V plane of decoded JPEG

  We transform the discrete values into continuous values by adding
  0.5 to the discrete value and then transform the rescaled continuous
  value to discrete values by truncating (integer part or floor) the
  continuous value.
*/
inline
static void
y2jpgy_line_c(unsigned char *y, int w)
{
	unsigned char *yend;
	for (yend = y + w; y < yend; ++y) {
		*y = saturate(((*y << 8) - 3968) / 220);
	}
}

inline
static void
uv2jpguv_line_c(unsigned char *u, unsigned char *v, int w)
{
	unsigned char *vend;
	for (vend = v + w; v < vend; ++u, ++v) {
		*u = saturate(((*u << 8) - 3968) / 225);
		*v = saturate(((*v << 8) - 3968) / 225);
	}
}

inline
static void
yuv2jpgyuv_c(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, int w, int h)
{
	int hh;
	for (hh = h; hh; --hh) {
		y2jpgy_line_c(y, w);
		y += ypitch;
	}
	int ww = w >> 1;
	for (hh = h >> 1; hh; --hh) {
		uv2jpguv_line_c(u, v, ww);
		u += uvpitch;
		v += uvpitch;
	}
}

void
yuv_digital2analog(unsigned char *y, int ypitch, unsigned char *u, unsigned char *v, int uvpitch, int w, int h, int insn)
{
	yuv2jpgyuv_c(y, ypitch, u, v, uvpitch, w, h);
}
