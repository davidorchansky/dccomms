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

int main(int argc, char ** argv) {
	fprintf(stderr, "Hello World\n");

	if(argc != 2)
	{
		perror("Incorrect number of parameters\n");
		exit(1);
	}

	int size = atoi(argv[1]) + 24;
	unsigned char buffer[size];

	int rsize = 0;

	fd_set_blocking(0,0);

	while(1)
	{
		clear();

		rsize = readNb(buffer, size);



		struct ezbtParams ezbtp;
		int headersize;
		ezbtDecodeParams(buffer, rsize, &ezbtp, &headersize);

		if(headersize > 0)
		{

			fprintf(stderr, "DECODER: Se decomprimirán %d bytes de imagen\n", rsize);
			int width = ezbtp.width;
			int height = ezbtp.height;
			int ysize = width * height;
			int uvsize = ysize / 4;

			int yuvsize =ysize+uvsize*2;
			unsigned char y[yuvsize];


			//fprintf(stderr, "Width= %d, Height= %d, Header size: %d\n", width, height, headersize/8);
			ezbtDecodeImg(buffer, rsize, y, yuvsize, EZBT_BIAS_EXP);

			//writep9header(width, height);

			//fwrite(y,1, yuvsize, stdout );
			write(1, y, yuvsize);



		}
	}
	return 0;
}
