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
			"\t-e               - Escribir por la salida estandar los acumuladores en pgm.\n", pgmname);
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
getOptions(int argc, char *argv[], int * nangulos, int * e)//, int * ndistancias)
{
	int opt;
	int error = 0;
	*nangulos = -1;
	*e = 0;
//	*ndistancias = -1;
	while ((opt = getopt(argc, argv, "a:d:e")) != -1) {
		switch (opt) {
		case 'a':
			error += optGetIntError(optarg, nangulos, 0);
			break;
		case 'e':
			*e = 1;
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
	/*if(*ndistancias < 0)
	{
		fprintf(stderr, "Necesario especificar el numero de distancias\n");
		usage(pgmName(argv[0]));

	}
	*/

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

void houghAcc(int x, int y, uint8_t ** houghSp, unsigned int ** acc, float * sinTable, float * cosTable, unsigned int width, unsigned int height, unsigned int nangulos, unsigned int rhoOffset, int vtrue)
{
	float rho;
	int thetaIndex;

	int x0=x,y0=y;
	x -= width >> 1;
	y -= height >> 1;

#ifdef DEBUG
	float maxrho0 = -99999, minrho0 =99999;
#endif
	for(thetaIndex = 0; thetaIndex < nangulos; thetaIndex++)
	{
		float rho0 = x * cosTable[thetaIndex] + y * sinTable[thetaIndex];
		int rho = ceil(rho0)+rhoOffset;
		houghSp[rho][thetaIndex] = vtrue;

		acc[rho][thetaIndex] += 1;
#ifdef DEBUG
		if(rho0 < minrho0) minrho0 = rho0;
		if(rho0 > maxrho0) maxrho0 = rho0;
#endif

	}
#ifdef DEBUG
		
	//	fprintf(stderr, "x0: %d ; y0: %d ; x: %d ; y: %d ; maxrho0: %f ; minrho0: %f ; rhoMax: %d ; rhoMax/2: %d ; ndist: %d\n", x0,y0,x, y, maxrho0,minrho0, rhoMax, rhoMaxHs, ndistancias);
#endif


	
}

static void saveImage(int fd, uint8_t * header, unsigned int hlength, uint8_t * content, unsigned int clength)
{
	int writen = write(fd, header, hlength);
	writen += write(fd, content, clength);
	fprintf(stderr, "Escritos: %d\n", writen);
}

static void showError()
{
	fprintf(stderr, "Ha ocurrido algun error: %s\n", strerror(errno));
}

static void getM_B(double smax, double smin, float * m, float *b)
{
	*m = 255. / (smax - smin);
	*b = - (smin * *m);
}

static void escalar_Int_Uint8(int * src, uint8_t * dst, unsigned int length)
{
	int * sptr,
	* maxsptr = src + length;

	uint8_t * dptr,
	* maxdptr = dst + length;

	int vmax = -9999, vmin = 9999;

	for(sptr = src; sptr < maxsptr; sptr++)
	{
		if(*sptr <= vmin) vmin = *sptr;
		if(*sptr >= vmax) vmax = *sptr;
	}

	float M, B;

	getM_B(vmax, vmin, &M, &B);
	fprintf(stderr, "Maximo: %d , Minimo: %d\n", vmax, vmin);

	for(sptr = src, dptr = dst; sptr < maxsptr; sptr++, dptr++)
	{
		*dptr = (uint8_t)round(*sptr * M + B);	
	}

}

static void invertirValores(uint8_t * src, unsigned int length)
{
	uint8_t * ptr, *mptr = src+length;
	for(ptr = src; ptr < mptr; ptr++)
	{
		*ptr = 255 - *ptr;
	}
}

int main(int argc, char ** argv)
{

	int nangulos, width, height,re;

	getOptions(argc, argv, &nangulos, &re);


	float * cosTable, * sinTable;

	cosTable = (float * ) malloc(sizeof(float) * nangulos);
	sinTable = (float * ) malloc(sizeof(float) * nangulos);

	float theta;
	int thetaIndex;
	float thetaInc = M_PI / nangulos;

	int vtrue = 0;
	int vfalse = 255;

	for(theta = 0, thetaIndex = 0; thetaIndex < nangulos; theta += thetaInc, thetaIndex++)
	{
		cosTable[thetaIndex] = cos(theta);
		sinTable[thetaIndex] = sin(theta);

#ifdef DEBUG
//		fprintf(stderr, "theta: %f ; cos: %f ; sin: %f\n", theta, cosTable[thetaIndex], sinTable[thetaIndex]);
#endif
	}

	if(read_pgmHeader(&width, &height) == 0)
	{

		unsigned int pixelLength = width*height;
		uint8_t * bordes = (uint8_t*) malloc(pixelLength);

		unsigned int n = fread(bordes, pixelLength, 1, stdin);

		fprintf(stderr, "Leido mapa de bordes: (%d bytes, width: %d, height: %d)\n", n*pixelLength,width, height);




		unsigned int rhoMax = sqrt(width*width + height*height);

		unsigned int ndistancias = rhoMax;

		fprintf(stderr, "Angulos: %d, Distancias: %d\n", nangulos, ndistancias);

		unsigned int houghSpLength = ndistancias*nangulos;

		//Matriz de acumuladores
		unsigned int ** accM = (unsigned int**) malloc (sizeof(unsigned int*) * ndistancias);
		unsigned int * acc = (unsigned int*) malloc(sizeof(unsigned int) * nangulos * ndistancias);

		int fila;
		for(fila = 0; fila < ndistancias ; fila++)
		{
			accM[fila] = acc + fila * nangulos;
			unsigned int * ptr, *mptr = accM[fila] + nangulos;
			for(ptr = accM[fila]; ptr < mptr; ptr++)
			{
				*ptr = 0;
			}

		}

		//Matriz del espacio de hough	
		uint8_t ** houghSpM = (uint8_t**) malloc (sizeof(uint8_t*) * ndistancias);
		uint8_t * houghSp = (uint8_t*) malloc(sizeof(uint8_t) * nangulos * ndistancias);

		for(fila = 0; fila < ndistancias ; fila++)
		{
			houghSpM[fila] = houghSp + fila * nangulos;
			uint8_t * ptr, *mptr = houghSpM[fila] + nangulos;
			for(ptr = houghSpM[fila]; ptr < mptr; ptr++)
			{
				*ptr = vfalse;
			}

		}

		//Matriz de bordes
		uint8_t ** bordesM = (uint8_t**) malloc(sizeof(uint8_t*)*pixelLength);
		unsigned int rhoOffset = ndistancias >> 1;
		for(fila = 0; fila < height; fila++)
		{
			bordesM[fila] = bordes + fila * width;
			int col = 0;
			for(col = 0; col < width; col++)
			{
				if(bordesM[fila][col] == 255) //255: pertenece a borde, 0: no pertenece a borde
				{
					houghAcc(col, fila, houghSpM, accM, sinTable, cosTable, width, height, nangulos, rhoOffset, vtrue);
				}
			}
		}

		uint8_t * pgmh = (uint8_t*) malloc(50);
		int pgmhl = sprintf((char*)pgmh, "P5\n%d %d\n255\n", nangulos, ndistancias);

		int fhoughSp = open("09-houghSp.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fhoughSp < 0) showError();

		
		saveImage(fhoughSp, pgmh, pgmhl, houghSp, houghSpLength);
		
		uint8_t * houghSpAccEscalado = (uint8_t *) malloc(houghSpLength);
		
		escalar_Int_Uint8(acc, houghSpAccEscalado, houghSpLength);

		int fhoughSpAcc = open("10-houghSpAcc.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fhoughSpAcc < 0) showError();

		invertirValores(houghSpAccEscalado, houghSpLength);
		saveImage(fhoughSpAcc, pgmh, pgmhl, houghSpAccEscalado, houghSpLength);

		if(re)
			saveImage(1, pgmh, pgmhl, houghSpAccEscalado, houghSpLength);

		close(fhoughSp);
		close(fhoughSpAcc);

		free(accM);
		free(acc);
		free(bordes);
		free(bordesM);
		free(pgmh);
		free(houghSpAccEscalado);



	}

	free(cosTable);
	free(sinTable);
	return 0;
}
