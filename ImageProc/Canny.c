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

static char *
pgmName(char *path)
{
	char *name = strrchr(path, '/');
	return name ? name + 1 : path;
}


static void
usage(char *pgmname)
{
	fprintf(stderr, "%s [options] < infile > outfile\n"
			"Options:\n"
			"\t-l               - Tamano del filtro gausiano (>=3 e impar).\n"
			"\t-s               - Sigma del filtro gausiano.\n", pgmname);
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
getOptions(int argc, char *argv[], int * filterSize, float *sigma)
{
	int opt;
	int error = 0;
	*filterSize = -1; *sigma = -1;
	while ((opt = getopt(argc, argv, "l:s:")) != -1) {
		switch (opt) {
		case 'l':
			error += optGetIntError(optarg, filterSize, 0);
			break;
		case 's':
			error += optGetFloatError(optarg, sigma, 0);
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
	if(*filterSize == -1 || *sigma == -1)
	{
		fprintf(stderr, "Missing required options\n");
		usage(pgmName(argv[0]));
	}
	if(*filterSize % 2 == 0 || *filterSize < 3)
	{
		fprintf(stderr, "El filtro no es correcto\n");
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

void showFilter(FILE * s, float * filter, int size)
{
	fprintf(s, "[ ");
	int i;
	for(i=0; i<size; i++)
	{
		fprintf(s, " %.4f, ", filter[i]);
	}
	fprintf(s, " ]\n");

}
float gaussianZeroMean(float x, float sigma)
{
	float res = pow(M_E, -1./2*pow(x/sigma,2));

	return res;
}

void getGaussianFilter(float * filtro, unsigned int tam, float sigma)
{
	int offset = tam >> 1;
	int idx;
	float * ptr;
	float sum = 0;
	for(idx = -offset, ptr = filtro; idx <= offset; idx++, ptr++)
	{
		*ptr = gaussianZeroMean(idx, sigma);
		sum += *ptr;
	}
	for(idx = 0; idx < tam; idx++)
	{
		filtro[idx] /= sum;
	}

}

void filtrarRuido(uint8_t * src, uint8_t * dst, unsigned int width, unsigned int height, float * filtro, unsigned int tamFiltro)
{
	unsigned int size = height * width;
	
	//Copiamos todo en dst
	memcpy(dst, src, size);

	int foffset = tamFiltro >> 1;
	int ioffset = foffset << 1;

	float * centroFiltro = filtro + foffset;

	//Numero de filas y columnas de la zona central (descartamos bordes de la imagen del grosor del filtro)
	int filas    = height - ioffset;
	int columnas = width - ioffset;

	
	//Pasada por filas (aplicación del filtro horizontal)
	unsigned int offsetInicial = foffset * width + foffset;

	uint8_t * dptr;

	dptr = dst + offsetInicial;

	uint8_t * maxf = dptr + width * filas;
	
	fprintf(stderr, "Aplicando filtro horizontal...\n");

//	fprintf(stderr, "Aplicando filtro horizontal...%d -- %d -- %d -- %d\n", foffset, ioffset, filas, columnas);
	while(dptr < maxf)
	{
		uint8_t * maxc = dptr + columnas;
		while(dptr < maxc)
		{
			//Aplicamos el filtro horizontalmente
			uint8_t valorFinal = 0;
			int idx;
			for(idx = -foffset; idx <= foffset; idx++)
			{
//			fprintf(stderr, "Algo passe %ld -- %d -- %d\n", (unsigned long )dptr,idx, foffset);
				valorFinal += *(dptr+idx) * *(centroFiltro+idx);
			}
			*dptr = valorFinal;
			dptr += 1;
		}
		dptr += ioffset;
	}

	dptr = dst + offsetInicial;
	uint8_t * maxc = dptr + columnas;

	fprintf(stderr, "Aplicando filtro vertical...\n");
	while(dptr < maxc)
	{
		uint8_t * dptrc = dptr;
		maxf = dptr + filas * width;
		while(dptr < maxf)
		{
			//Aplicamos el filtro verticalmente
			uint8_t valorFinal = 0;
			int idx;
			for(idx = -foffset; idx <= foffset; idx++)
			{
				valorFinal += *(dptr+idx*(int)width) * *(centroFiltro+idx);

			}
			*dptr = valorFinal;
			dptr += width;
		}
		dptr = dptrc + 1;
	}
}

void showError()
{
	fprintf(stderr, "Ha ocurrido algun error: %s\n", strerror(errno));
}

int main(int argc, char ** argv)
{

	int width, height;
	unsigned int rgbLength, pixelLength;
	uint8_t *ppm, *rgb, *pgm, *gs;
	
	unsigned int tamFiltro;
	float * filtro;
	float sigma;
	
	getOptions(argc, argv, &tamFiltro, &sigma);

	filtro = (float*) malloc(tamFiltro * sizeof(float));

	getGaussianFilter(filtro, tamFiltro, sigma);

	fprintf(stderr, "Sigma: %f , Size: %d\n", sigma, tamFiltro);
	showFilter(stderr, filtro, tamFiltro);
	
	if(read_header(&width, &height) == 0)
	{
		fprintf(stderr, "Recibido ppm header\nEsperando RGB...\n");

		pixelLength = width * height;
		rgbLength = pixelLength  * 3;	

		fprintf(stderr, "Width: %d , Height: %d\n", width, height);
		ppm = (uint8_t*) malloc(rgbLength+50);
	
		int ppmhl = sprintf((char*)ppm, "P6\n%d %d\n255\n", width, height);

		rgb = ppm +  ppmhl;

		unsigned int n = fread(rgb, rgbLength, 1, stdin);

		fprintf(stderr, "Leido RGB: %d (%d bytes)\nConvirtiendo a GrayScale...\n", n, rgbLength*n);

		pgm = (uint8_t*) malloc(pixelLength+50);
		int pgmhl = sprintf((char*)pgm, "P5\n%d %d\n255\n", width, height);

		unsigned int ppmLength = ppmhl + rgbLength;
		unsigned int pgmLength = pgmhl + pixelLength;
		gs = pgm + pgmhl;
		rgb2gs(rgb, gs, width, height);


		int fppm = open("salida.ppm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fppm < 0) showError();
 		int fpgm = open("salida.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fpgm < 0) showError();

		int res = write(fppm, ppm, ppmLength);
		if (res != ppmLength) showError(); 
		res = write(fpgm, pgm, pgmLength);
		if (res != pgmLength) showError(); 

		close(fppm);
		close(fpgm);

		unsigned char * gsFiltrado = (uint8_t*) malloc(pixelLength);

		filtrarRuido(gs, gsFiltrado, width, height, filtro, tamFiltro);

		int writen = write(1, pgm, pgmhl);
		writen += write(1, gsFiltrado, pixelLength);
		fprintf(stderr, "Escritos: %d\n", writen);



	}
	return 0;
}



