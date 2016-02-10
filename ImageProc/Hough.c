#include <stdio.h>
#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h> 
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <getopt.h>
#include <errno.h>
#include <limits.h>


/***** NOTA **********

Algunas funciones y constantes estan implementadas tambien en Canny.c. Proximamente se hara un refactor creando un fichero
de cabecera comun a Canny.c y Hough.c

*********************/


#define RAD_2_DEG 57.29577951308232

static char *
pgmName(char *path)
{
	char *name = strrchr(path, '/');
	return name ? name + 1 : path;
}


static void
usage(char *pgmname)
{
	fprintf(stderr, "%s [opciones] < bordes.pgm\n"
			"Descripcion:\n"
			"Recibe por la entrada estandar una imagen en escala de grises en formato pgm y calcula su transformada de Hough.\n"
			"Opciones:\n"
			"\t-a               - Numero de angulos.\n"
			"\t-d               - Numero de distancias.\n", pgmname);
	exit(-1);
}

static int
optGetIntError(char *optarg, int *val, int min)
{
	int x = atoi(optarg);
	if (x <= min)
		return 1;
	*val = x;
	return 0;
}

static float
optGetFloatError(char *optarg, float *val, float min)
{
	float x = atof(optarg);
	if (x <= min)
		return 1;
	*val = x;
	return 0;
}


static int
getOptions(int argc, char *argv[], int * nangulos, int * ndistancias)
{
	int opt;
	int error = 0;
	*nangulos = -1;
	*ndistancias = -1;
	while ((opt = getopt(argc, argv, "a:d:")) != -1) {
		switch (opt) {
		case 'a':
			error += optGetIntError(optarg, nangulos, 0);
			break;
		case 'd':
			error += optGetIntError(optarg, ndistancias, 0);
			break;
		default: /* '?' */
			usage(argv[0]);
		}
	}
	if (error) {
		usage(pgmName(argv[0]));
	}
	if (optind < argc) {
		fprintf(stderr, "Unexpected argument(s) after options\n");
		usage(pgmName(argv[0]));
	}
	if(*nangulos < 0) 
	{
		fprintf(stderr, "Necesario especificar el numero de angulos\n");
		usage(pgmName(argv[0]));
	}
	if(*ndistancias < 0)
	{
		fprintf(stderr, "Necesario especificar el numero de distancias\n");
		usage(pgmName(argv[0]));

	}

}

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
read_pgmHeader(int *w, int *h)
{
	if (getc(stdin) != 'P')
		return -1;
	int c = getc(stdin);
	if (c != '5')
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

int main(int argc, char ** argv)
{

	int nangulos, ndistancias, width, height;

	getOptions(argc, argv, &nangulos, &ndistancias);

	fprintf(stderr, "Angulos: %d, Distancias: %d\n", nangulos, ndistancias);

	if(read_pgmHeader(&width, &height) == 0)
	{

	}
	return 0;
}
