/*
 * pruebas.c
 *
 *  Created on: 10/12/2014
 *      Author: diego
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <stdint.h>


void clear()
{
        char c;
        while(fread(&c,1,1,stdin)>0){}
}

int readNb(void * b, int size)
{
		uint8_t* bu = (uint8_t*)b;
        int r =0;
        while(r < size)
        {
                r += fread(bu+r,1,size-r,stdin);
        }
        return r;
}

int fd_set_blocking(int fd, int blocking) {
    /* Save the current flags */
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
        return 0;

    if (blocking)
        flags &= ~O_NONBLOCK;
    else
        flags |= O_NONBLOCK;
    return fcntl(fd, F_SETFL, flags) != -1;
}
static void
defaultParams(struct encParam *e, struct decParam *d)
{
	encParam_init(e);
	decParam_init(d);
}

#define MAXSIZE 	(64 * 1024 * 1024) // 64 megabytes is a HUGE BUFFER for a compressed image! Signal anything above it

static unsigned char *
read_img(int *size, FILE *f)
{
	int rsize = *size;
	/* silently limit the compressed image size to 64M */
	if (rsize <= 0 || rsize > MAXSIZE) {
		rsize = MAXSIZE;
	}
	/* allocate memory for it */
	unsigned char *cimg = malloc(rsize);
	if (cimg) {
		rsize = fread(cimg, 1, rsize, f);
		if (rsize > 0) {
			*size = rsize;
			return cimg;
		}
		fprintf(stderr, "Short read (%d)\n", rsize);
		free(cimg);
	} else {
		fprintf(stderr, "Failed to alloc memory (%d)\n", rsize);
	}
	return NULL;
}



/*
 * returns the number of bytes written or -1 in case of error or short write
 */
static int
write_img(struct imgBuffer *img, FILE *f)
{
	int color = colorFormatFmt(img->colorFormat);
	if (color != COLORFORMAT_FMT_GREY && color != COLORFORMAT_FMT_420) {
		fprintf(stderr, "Unsupported color format (%d)\n", color);
		return -1;
	}
	int ysize = img->width * img->height; // FIXME: assumes img->pitch == img->width
	int uvsize = img->uvwidth * img->uvheight; // FIXME: assumes img->uvpitch == img->uvwidth
	int yuvsize = ysize + (uvsize << 1);
	/* PGM header */
	fprintf(stdout, "P%d\n%d %d\n255\n", color == COLORFORMAT_FMT_GREY ? 5 : 7, img->width, img->height);
	int n = fwrite(img->buffer, 1, yuvsize, f);
	if (n != yuvsize) {
		fprintf(stderr, "Short write (%d out of %d)\n", n, yuvsize);
		n = -1;
	}
	return n;
}

static int
decode(struct decParam *d)
{
	int retval = -1;
	/* read the image up to size */
	int debtimgsize = ((d->bitlen + 7) >> 3);
	unsigned char *debtimg = read_img(&debtimgsize, stdin);
	if (debtimg) {
		/* prepare an imgBuffer for decoding */
		struct imgBuffer *dst = imgBuffer_new();
		if (dst) {
			if (debt_decode_imgbuffer(dst, debtimg, debtimgsize, d, NULL, NULL)) {
				retval = write_img(dst, stdout);
			}
			imgBuffer_del(dst);
		}
		free(debtimg);
	}
	return retval;
}


int main(int argc, char ** argv) {
	fprintf(stderr, "Hello World\n");


	fd_set_blocking(0,0);

	struct encParam e;
	struct decParam d;

	defaultParams(&e, &d);


	while(1)
	{
		clear();
		decode(&d);
	}

	decParam_destroy(&d);
	encParam_destroy(&e);


	return 0;
}
