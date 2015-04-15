/*
 * pruebas.c
 *
 *  Created on: 10/12/2014
 *      Author: diego
 */

#include <stdio.h>
#include <libezbt.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <colorconv.h>
#include <colorfmt.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <sys/time.h>

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
	return 0;
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
	struct ezbtParams ezbtp;


	ezbtInitParams(&ezbtp);

	ezbtp.lscan = !!atoi(argv[1]);//1;
	ezbtp.dscan = !!atoi(argv[2]);//1;
	ezbtp.rscan = !!atoi(argv[3]);//1;
	ezbtp.zscan = !!atoi(argv[4]);//0;

	fprintf(stderr, "lscan: %d, dscan: %d, rscan: %d, zscan: %d\n",
			ezbtp.lscan,
			ezbtp.dscan,
			ezbtp.rscan,
			ezbtp.zscan);

	ezbtp.pad = 0;

	int width, height;


	while(1)
	{

		if(readp9(&width, &height)==0)
		{
			//fprintf(stderr, "Width: %d ; Height: %d\n", width, height);

			int ysize = width * height;
			int uvsize = ysize / 4;

			//fprintf(stderr, "ysize: %d ; uvsize: %d\n", ysize, uvsize);

			unsigned char *y = malloc(ysize+uvsize*2);
			unsigned char *u = y+ysize;
			unsigned char *v = u+uvsize;

			fread(y, ysize+uvsize*2, 1, stdin);
			fflush(stdin);




			int colorformat = newColorFormat(COLORFORMAT_STD_BT601_ANALOG,
					COLORFORMAT_FMT_420);

			unsigned char *cimg = NULL;
			int cimgsize = 0;
			int size = ezbtEncodeImg(y, width, height, colorformat, &cimg, &cimgsize, 0, &ezbtp,
					EZBT_TRANSFORM_RW97, 6, -1, 0, 0);

			fprintf(stderr, "ENCODER: Imagen comprimida a %d bytes\n", size);

                	gettimeofday(&start, NULL);

			write(1, header, headerSize);
			write(1, &start, sizeof(struct timeval));
			write(1, cimg, size);

			free(y);

			clearInputBuffer();


		}

	}

	return 0;
}
