#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h> 
#include <fcntl.h>

static int
readint(int *x, int *next, FILE *f)
{
	int error = -1;
	int v = 0;
	int c;
	if (isdigit(c = getc(f))) {
		v = c - '0';
		while (isdigit(c = getc(f))) {
			v = v * 10 + (c - '0');
		}
		*x = v;
		error = 0;
	}
	*next = c;
	return error;
}

static int
readwh(int *w, int *h, FILE *f)
{
	int next;
	if (readint(w, &next, f) || next != ' ')
		return -1;
	if (readint(h, &next, f) || next != '\n')
		return -1;
	return 0;
}

static int
readmax(int *max, FILE *f)
{
	int next;
	return (readint(max, &next, f) || next != '\n') ? -1 : 0;
}

static int
read_header(int *w, int *h)
{
	if (getc(stdin) != 'P')
		return -1;
	int c = getc(stdin);
	if (c != '6')
		return -1;
	if (getc(stdin) != '\n')
		return -1;
	if (readwh(w, h, stdin))
		return -1;
	int max;
	if (readmax(&max, stdin) || max != 255)
		return -1;
	return 0;
}


int rgb2gs(unsigned char * rgb, unsigned char * gs, unsigned int width, unsigned int height)
{
	//Y <- 0.299R + 0.587G + 0.114B

	unsigned int pixelLength = width * height;	 

	unsigned char * maxGs = gs + pixelLength;


	while(gs < maxGs)
	{
		*gs++ = 0.299**rgb++ +0.587**rgb++ +0.114**rgb++;
	}
	return 1;
}

#define FSIZE 5

uint8_t filtro[FSIZE] = {0.05449, 0.24420, 0.40262, 0.24420, 0.05449};


void filtrar(uint8_t * src, uint8_t * dst, unsigned int height, unsigned int width)
{
	unsigned int size = height * width;

	int foffset = FSIZE >> 1;
	int ioffset = foffset << 1;
	
	int filas    = height - ioffset;
	int columnas = width - ioffset;

	//Pasada por filas
	for(fila = 0; fila < filas; fila++)
	{
		uint8_t *sptr, *maxptr;
		sptr = src + fila * width + foffset;


	}
}

int main(int argc, char ** argv)
{

	int width, height;
	unsigned int rgbLength, pixelLength;
	uint8_t *ppm, *rgb, *pgm, *gs;


	if(read_header(&width, &height) == 0)
	{
		fprintf(stderr, "Recibido ppm header\nEsperando RGB...\n");

		pixelLength = width * height;
		rgbLength = pixelLength  * 3;	

		ppm = (uint8_t*) malloc(rgbLength+50);

		int ppmhl = sprintf((char*)ppm, "P6\n%d %d\n255\n", width, height);

		rgb = ppm +  ppmhl;
		//			unsigned int n = read(0, rgb, rgbLength);

		unsigned int n = fread(rgb, rgbLength, 1, stdin);

		fprintf(stderr, "Leido RGB: %d (deberia: %d) bytes\nConvirtiendo a GrayScale...\n", n, rgbLength);

		pgm = (uint8_t*) malloc(pixelLength+50);
		int pgmhl = sprintf((char*)pgm, "P5\n%d %d\n255\n", width, height);

		unsigned int ppmLength = ppmhl + rgbLength;
		unsigned int pgmLength = pgmhl + pixelLength;
		gs = pgm + pgmhl;
		rgb2gs(rgb, gs, width, height);

		int writen = write(1, pgm, pgmLength);

		int fppm = open("salida.ppm", O_CREAT | O_WRONLY | O_TRUNC);
		int fpgm = open("salida.pgm", O_CREAT | O_WRONLY | O_TRUNC);

		write(fppm, ppm, ppmLength);
		write(fpgm, pgm, pgmLength);

		close(fppm);
		close(fpgm);


		fprintf(stderr, "Escritos: %d\n", writen);



	}
	return 0;
}



/*
// TEST	
	unsigned char * a,*b;
	a = (unsigned char *) malloc(4);
	a[0] = 9;
	a[1] = 1;
	a[2] = 2;
	a[3] = 3;
	b = a;
	printf("%d\n", 2**++b);
	printf("%d\n", 2**++b);
	printf("%d\n", 2**++b);
	return 0;
*/

