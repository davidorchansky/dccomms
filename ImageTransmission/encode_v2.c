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
#include <ctype.h>
#include <string.h>
#include <sys/time.h>
#include <libdebt.h>
#include <getopt.h>


int
readint(int *v)
{
	int c;
	while ((c = getchar()) != EOF && isdigit(c)) {
		*v = *v * 10 + (c - '0');
	}
	return c;
}

int
readp9(int *w, int *h)
{
	if (getchar() != 'P' || getchar() != '9' || getchar() != '\n')
		return -1;
	*w = 0;
	if (readint(w) != ' ')
		return -1;
	*h = 0;
	if (readint(h) != '\n')
		return -1;
	int v = 0;
	if (readint(&v) != '\n' || v != 255)
		return -1;

	return COLORFORMAT_FMT_420;
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


void clearInputBuffer()
{
		fd_set_blocking(0,0);
        char c;
        while(fread(&c,1,1,stdin)>0){}
        fd_set_blocking(0,1);
}


static struct imgBuffer *
read_src_img(int w, int h, int color)
{
	struct imgBuffer *img = NULL;
	if (color != -1) {
		/* prepare an image buffer for the input image */
		if ((img = imgBuffer_aligned_new(1))) {
			if (imgBuffer_reinit(img, w, h, color)) {
				int yuvsize = (img->width * img->height) + ((img->uvwidth * img->uvheight) << 1);
			
				int n = fread(img->buffer, 1, yuvsize, stdin);
				if (n == yuvsize) {
					return img;
				}
				fprintf(stderr, "Short read (%d from %d)\n", n, yuvsize);
			}
			imgBuffer_del(img);
			img = NULL;
		}
	}
	return img;
}

int main(int argc, char ** argv) {
	fprintf(stderr, "Encoder UP\n");

	int headerSize;
	char * header;
	struct timeval start;

	if(argc==6)
	{
		header = argv[5];
		headerSize = strlen(argv[5]);
	}
	else if(argc==5)
	{
		header = NULL;
		headerSize = 0;
	}
	else
	{
		perror("Incorrect number of parameters (lscan,dscan,rscan,zscan,Header(optional))");
		exit(1);
	}


	int width, height;

	struct encParam *e = (struct encParam*) malloc(sizeof(struct encParam*));
	struct decParam *d = (struct decParam*) malloc(sizeof(struct decParam*)); //not used

	defaultParams(e, d);

	int imgsize, n;
	struct imgBuffer * src;
	
	while(1)
	{

		if(readp9(&width, &height)==0)
		{
			src = read_src_img(width, height, COLORFORMAT_FMT_420);
			
			fflush(stdin);


			if (src) {
				/* setup the output buffer */
				unsigned char *buffer = NULL;
				int buflen = 0;
				int fixed = 0;
				if (e->maxsize) {
					buffer = malloc(buflen = e->maxsize);
					fixed = 1;
				}
				/* encode it! */
				int blen = debt_encode_imgbuffer(src, &buffer, &buflen, fixed, e);
				if (blen) {
					/* negative result means valid but truncated output (the user has probably specified a max size or no more memory */
					if (blen < 0) {
						blen = -blen;
					}
					n = (blen + 7) >> 3;

					fprintf(stderr, "ENCODER: Imagen comprimida a %d bytes\n", n);


                			gettimeofday(&start, NULL);

					write(1, header, headerSize);
					write(1, &start, sizeof(struct timeval));
					imgsize = write(1, buffer, n);

					if (n != imgsize) {
						fprintf(stderr, "Short write (%d out of %d)\n", imgsize, n);
					}
				}
				free(buffer);
				buflen = 0;
				imgBuffer_del(src);
			}

			clearInputBuffer();

		}

	}

	return 0;
}
