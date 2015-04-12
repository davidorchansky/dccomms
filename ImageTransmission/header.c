/*
 ============================================================================
 Name        : header.c
 Author      : Diego
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>

int
writep9header(int w, int h)
{/*
	fputs("P9\n", stdout);
	fprintf(stdout, "%d %d\n", w ,h);
	fputs("255\n",stdout);
	return 0;
	*/
	char header[100];
	int n=sprintf(header, "P9\n%d %d\n255\n", w, h);
	write(1, header, n);

	return 0;
}


/**
 * Set a file descriptor to blocking or non-blocking mode.
 *
 * @param fd The file descriptor
 * @param blocking 0:non-blocking mode, 1:blocking mode
 *
 * @return 1:success, 0:failure.
 **/
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

int main(int argc, char** argv) {
	//puts("This program puts the header 'P9\\nWIDTH HEIGHT\\n255\\n'\nfor each WIDTH*HEIGHT+WIDTH*HEIGHT/2 bytes from stdin (yuyv420 size)\nand sends the result through stdout"); /* prints This program puts the header "P9\nWIDTH HEIGHT\n255\n" for each WIDTH*HEIGHT+ */

	if(argc != 3)
	{
		perror("Incorrect number of parameters.\nUsage: program <WIDTH> <HEIGHT>");
		exit(1);
	}
	int width, height;

	width = atoi(argv[1]);
	height = atoi(argv[2]);

	uint32_t ysize = width * height;
	uint32_t uvsize = ysize / 4;


	uint32_t size = ysize+uvsize*2;

	fd_set_blocking(0,1);

	unsigned char *y = malloc(size);
	long r;
	while(1)
	{
		writep9header(width, height);
		r = 0;
		while(r < size)
		{
			r += read(0, y+r, size-r);
		}

		write(1, y, size);
		fprintf(stderr, "\nHEADER: Añadido header a %d bytes de imagen yuyv420\n", r);



	}

	return EXIT_SUCCESS;
}
