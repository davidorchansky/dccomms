
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
#include <sys/types.h>

#ifdef TIMMING
#include <sys/time.h>
#endif


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
	fprintf(stderr, "%s [opciones] < img.ppm\n"
			"Descripcion:\n"
			"Recibe por la entrada estandar una imagen RGB en formato ppm y escribe en ficheros pgm (XX-etapa.pgm) el resultado del algoritmo Canny primero, y luego Hough, en cada etapa. Tambien genera la version original de 'img.ppm' en ppm (rgb) y ppm (escala de grises).\n"
			"Opciones:\n"
			"\t-l               - Tamano del filtro gausiano (>=3 e impar).\n"
			"\t-s               - Sigma del filtro gausiano.\n"
			"\t-U               - Umbral superior para deteccion de borde (0 < x < 255).\n"
			"\t-L               - Umbral inferior para deteccion de borde (0 < x < 255).\n"
			"\t-a               - Numbero de angulos de la transformada de hough.\n"
			"\t-e               - Enviar por la salida estandar la imagen pgm de acumuladores del espacio de Hough.\n"
			"\t-d               - Directorio de salida.\n", pgmname);
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
getOptions(int argc, char *argv[], int * filterSize, float *sigma, unsigned int *uth, unsigned int * lth, int *re,
int *nangulos, char ** outdir)
{
	int opt;
	int error = 0;
	*re = 0;
	*filterSize = -1; *sigma = -1;
	*nangulos = -1;
	*re = 0;
	*outdir = NULL;
	while ((opt = getopt(argc, argv, "l:s:U:L:a:ed:")) != -1) {
		switch (opt) {
		case 'l':
			error += optGetIntError(optarg, filterSize, 0);
			break;
		case 's':
			error += optGetFloatError(optarg, sigma, 0);
			break;
		case 'U':
			error += optGetIntError(optarg, uth, -1);
			break;
		case 'L':
			error += optGetIntError(optarg, lth, -1);
			break;
		case 'a':
			error += optGetIntError(optarg, nangulos, 0);
			break;
		case 'e':
			*re = 1;
			break;
		case 'd':
			*outdir = optarg;
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
	if(*lth > 255 || *uth > 255 || *lth < 0 || *uth < 0)
	{
		fprintf(stderr, "El umbral no es correcto\n");
		usage(pgmName(argv[0]));
	}
	if(*nangulos < 0) 
	{
		fprintf(stderr, "Necesario especificar el numero de angulos\n");
		usage(pgmName(argv[0]));
	}
	if(*outdir == NULL)
	{
		*outdir = malloc(strlen(".")+1);
		strcpy(*outdir, ".");
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
read_ppmHeader(int *w, int *h)
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

void aplicarFiltro_Uint8_ConPrecision(uint8_t * src, uint8_t * dst, unsigned int width, unsigned int height, float * hfiltro, float * vfiltro, unsigned int tamFiltro)
{
	unsigned int size = height * width;
	
	double * bufferDeTrabajo = (double *) malloc(size * sizeof(double));
	//Copiamos todo en bufferDeTrabajo para trabajar con mayor precision

	uint8_t * sptr, *maxsptr = src + size;
	double * wptr;
	
	for(wptr = bufferDeTrabajo, sptr = src; sptr < maxsptr; sptr++, wptr++)
	{
		*wptr = (double) *sptr;
	}

	int foffset = tamFiltro >> 1;
	int ioffset = foffset << 1;


	//Numero de filas y columnas de la zona central (descartamos bordes de la imagen del grosor del filtro)
	int filas    = height - ioffset;
	int columnas = width - ioffset;

	
	//Pasada por filas (aplicación del filtro horizontal)
	unsigned int offsetInicial = foffset * width + foffset;

	double * posicionInicial = bufferDeTrabajo + offsetInicial;
	double * dptr;
	double * maxf = posicionInicial + width * filas;
	float * centroFiltro = hfiltro + foffset;

#ifdef DEBUG
	fprintf(stderr, "Aplicando filtro horizontal...\n");
	double valorMaximo = -9999, valorMinimo = 9999;
#endif

	for(dptr = posicionInicial; dptr < maxf; dptr+=ioffset)
	{
		double * maxc;
		for(maxc = dptr + columnas; dptr < maxc; dptr +=1)
		{
			//Aplicamos el filtro horizontalmente
			double valorFinal = 0;
			int idx;
			for(idx = -foffset; idx <= foffset; idx++)
			{
				valorFinal += *(dptr+idx) * *(centroFiltro+idx);
			}
			*dptr = valorFinal;
#ifdef DEBUG
			if(valorFinal >= valorMaximo) valorMaximo = valorFinal;
			if(valorFinal <= valorMinimo) valorMinimo = valorFinal;
#endif
		}
	}


#ifdef TEST
	fprintf(stderr, "Valor maximo: %f, Valor minimo: %f, Aplicando filtro vertical...\n", valorMaximo, valorMinimo);
#endif

	centroFiltro = vfiltro + foffset;

	double * maxc = posicionInicial + columnas;
	double * dptrc;
	for(dptr = posicionInicial, dptrc = posicionInicial; dptr < maxc; dptr = dptrc+1, dptrc = dptr)
	{
		for(maxf = dptr + filas * width; dptr < maxf; dptr += width)
		{
			//Aplicamos el filtro verticalmente
			double valorFinal = 0;
			int idx;
			for(idx = -foffset; idx <= foffset; idx++)
			{
				valorFinal += *(dptr+idx*(int)width) * *(centroFiltro+idx);

			}
			*dptr = valorFinal;
#ifdef DEBUG
			if(valorFinal >= valorMaximo) valorMaximo = valorFinal;
			if(valorFinal <= valorMinimo) valorMinimo = valorFinal;
#endif
		}
	}

#ifdef DEBUG
	fprintf(stderr, "Valor maximo: %f, Valor minimo: %f\n", valorMaximo, valorMinimo);
#endif
	
	uint8_t * maxdptr = dst + size;
	uint8_t * res;
	for(res = dst, wptr = bufferDeTrabajo; res < maxdptr; res++, wptr++)
	{
		*res = (uint8_t) *wptr;
	}
	free(bufferDeTrabajo);

}

static void aplicarFiltroNoSeparable(uint8_t * src, double * dst, unsigned int width, unsigned int height, double ** filtro, unsigned int tfiltro)
{

	unsigned int size = width * height;
	uint8_t ** sM = (uint8_t**) malloc(height * sizeof(uint8_t*));
	double ** dM = (double**) malloc(height * sizeof(double*));

	uint8_t * sptr;
	double * dptr;
	int pos;

	for(pos = 0, sptr = src, dptr = dst; pos < height; pos += 1, sptr += width, dptr += width)
	{
		sM[pos] = sptr;
		dM[pos] = dptr;
	}

	unsigned int foffset = tfiltro >> 1;

	int f, c;
	for(f=foffset; f < height-foffset; f++)
	{
		for(c=foffset; c < width-foffset; c++)
		{
			int f2,c2, ff,fc, f2o = f-foffset, c2o = c-foffset;
			double v = 0;
			for(f2 = f2o, ff = 0; ff < tfiltro; f2++, ff++)
			{
				for(c2 = c2o, fc = 0; fc < tfiltro; c2++, fc++)
				{
					v += sM[f2][c2] * filtro[ff][fc];
				}
			}
			dM[f][c] = v;
		}
	}

	free(sM);
	free(dM);

}
static void obtenerModuloGradiente(double * xg, double * yg, double * mg, unsigned int width, unsigned int height)
{
	unsigned int length = width * height;
	double * xptr, *yptr, *mptr, *maxxptr;

	for(xptr = xg, yptr = yg, mptr = mg, maxxptr = xptr + length; xptr < maxxptr; mptr++, xptr++, yptr++)
	{
		*mptr = sqrt(*xptr**xptr +  *yptr**yptr);
	}
}

static void obtenerDireccionGradiente(double * xg, double * yg, double * mg, unsigned int width, unsigned int height)
{
	unsigned int length = width * height;
	double * xptr, *yptr, *mptr, *maxxptr;

	for(xptr = xg, yptr = yg, mptr = mg, maxxptr = xptr + length; xptr < maxxptr; mptr++, xptr++, yptr++)
	{
		*mptr = atan(*yptr / *xptr);
	}
}

static void showError()
{
	fprintf(stderr, "Ha ocurrido algun error: %s\n", strerror(errno));
}

static uint8_t getDireccion(double rad)
{

	double deg = rad * RAD_2_DEG;

	if(deg < 0)
		deg = 180 + deg;

	double dif180 = abs(deg - 180);
	double dif135 = abs(deg - 135);
	double dif90 = abs(deg - 90);
	double dif45 = abs(deg - 45);
	double dif0 = deg;

	uint8_t res = 0;
	double diff = dif180;

	if(dif135 < diff){ res = 135; diff = dif135;}
	if(dif90 < diff){ res = 90; diff = dif90;}
	if(dif45 < diff){ res = 45; diff = dif45;}
	if(dif0 < diff){ res =  0; diff = dif0;}

        return res;
}

#ifdef DEBUG
static unsigned int escalarDireccionDiscreta(uint8_t dir)
{
	switch(dir)
	{
		case 0:
			return 0;
		case 45:
			return 85;
		case 90:
			return 170;
		case 135:
			return 255;
		default:
			return 255;
	}

}
#endif
static void discretizarDireccionGradiente(double * dgradiente, uint8_t * dgdiscretizada, unsigned int width, unsigned int height)
{
	unsigned int size = width * height;
	double * sptr, *maxdg = dgradiente + size;
	uint8_t * dptr;

#ifdef DEBUG
	double vmaxr = -999, vminr = 999; 
#endif
	for(sptr = dgradiente, dptr = dgdiscretizada; sptr < maxdg; sptr++, dptr++)
	{
#ifdef DEBUG
		if(*sptr > vmaxr) vmaxr = *sptr;
		if(*sptr < vminr) vminr = *sptr;
#endif
		*dptr = getDireccion(*sptr);
	}

#ifdef DEBUG
	uint8_t vmaxd, vmind;
	vmaxd = getDireccion(vmaxr);
	vmind = getDireccion(vminr);
	fprintf(stderr, "vmaxr: %f : %d , vminr: %f : %d\n", vmaxr ,vmaxd, vminr, vmind);

	double aux = 0;
	for(aux = -M_PI/2; aux <= M_PI/2 ; aux += M_PI/30)
	{
		double deg = RAD_2_DEG * aux;
		if(deg > 0)
			fprintf(stderr, "rad: %f : %f : %d : %d\n", aux, deg, getDireccion(aux), escalarDireccionDiscreta(getDireccion(aux)));
		else
			fprintf(stderr, "rad: %f : %f : %d : %d\n", aux, 180+deg, getDireccion(aux), escalarDireccionDiscreta(getDireccion(aux)));
	}
#endif
}

static void nonMaximum(double * mg, uint8_t * dg, double * mgthin, unsigned int width, unsigned int height)
{
	unsigned int size = width * height;
	double ** mgM = (double**) malloc(height * sizeof(double*));
	uint8_t ** dgM = (uint8_t**) malloc(height * sizeof(uint8_t*));

	double * mptr;
	uint8_t * dptr;

	int pos;

	memcpy(mgthin, mg, size*sizeof(double));

	for(pos = 0, mptr = mg, dptr = dg; pos < height; pos += 1, mptr += width, dptr += width)
	{
		mgM[pos] = mptr;
		dgM[pos] = dptr;
	}

	unsigned int foffset = 1;

	int f, c;
	for(f=foffset; f < height-foffset; f++)
	{
		for(c=foffset; c < width-foffset; c++)
		{
			int mf1,mf2,mc1,mc2;
			//Miramos la direccion del gradiente

			int caso = -1;
			switch(dgM[f][c])
			{
				case 0:
					caso = 0;
					mf1 = f;
					mc1 = c - 1;
					mf2 = f;
					mc2 = c + 1;
					break;
				case 45:	
					caso = 45;
					mf1 = f - 1;
					mc1 = c + 1;
					mf2 = f + 1;
					mc2 = c - 1;
					break;
				case 90:
					caso = 90;
					mf1 = f - 1;
					mc1 = c;
					mf2 = f + 1;
					mc2 = c;
					break;
				case 135:
					caso = 135;
					mf1 = f - 1;
					mc1 = c - 1;
					mf2 = f + 1;
					mc2 = c + 1;
					break;
				default:
					fprintf(stderr, "ERROR: angulo incorrecto en nonMaximum\n");
					exit(1);

			}

			//fprintf(stderr, "caso %d\n", caso);
			double v = mgM[f][c];

			if(v < mgM[mf1][mc1] || v < mgM[mf2][mc2])
			{
				//fprintf(stderr, "%d-%d , %d-%d, %d-%d\n", mf1,mc1,f,c,mf2,mc2);
				double * dst = mgthin + f * width + c;
				*dst = 0;
			}
		}
	}

	free(mgM);
	free(dgM);
}

static void hysteresis(uint8_t * src, uint8_t * dst, uint8_t alto, uint8_t bajo, unsigned int width, unsigned int height)
{

	unsigned int size = width * height;

	uint8_t ** srcM = (uint8_t**) malloc(height * sizeof(uint8_t*));
	uint8_t ** dstM = (uint8_t**) malloc(height * sizeof(uint8_t*));

	uint8_t * sptr;
	uint8_t * dptr;

	int pos;

	uint8_t vmax = 255, vmin = 0;
	for(pos = 0, sptr = src, dptr = dst; pos < height; pos += 1, sptr += width, dptr += width)
	{
		srcM[pos] = sptr;
		dstM[pos] = dptr;
	}

	int f, c;
	for(f = 0; f < height; f++)
	{
		for(c = 0; c < width; c++)
		{
			uint8_t v = srcM[f][c];
			if(v > alto)
				dstM[f][c] = vmax;
			else if (v > bajo)
			{
				int fb = f-1,fa = f+1, cb = c-1, ca =c+1;
				//miramos los vecinos
				if(srcM[fb][cb] > alto 
				|| srcM[fb][c] > alto
				|| srcM[fb][ca] > alto
				|| srcM[f][cb] > alto
				|| srcM[f][ca] > alto
				|| srcM[fa][cb] > alto
				|| srcM[fa][c] > alto
				|| srcM[fa][ca] > alto)
					dstM[f][c] = vmax;
				else
					dstM[f][c] = vmin;
			}
			else
				dstM[f][c] = vmin;
		}
	}
	
	free(srcM); free(dstM);
}

static void getM_B(double smax, double smin, double * m, double *b)
{
	*m = 255. / (smax - smin);
	*b = - (smin * *m);
}
		
static void escalar_Double_Uint8(double * ygradiente, uint8_t * ygescalado, unsigned int width, unsigned int height)
{
	unsigned int size = width * height;
	double * sptr,
	* maxsptr = ygradiente + size;

	uint8_t * dptr,
	* maxdptr = ygescalado + size;

	double vmax = -9999, vmin = 9999;

	for(sptr = ygradiente; sptr < maxsptr; sptr++)
	{
		if(*sptr <= vmin) vmin = *sptr;
		if(*sptr >= vmax) vmax = *sptr;
	}

	double M, B;

	getM_B(vmax, vmin, &M, &B);
#ifdef DEBUG
	fprintf(stderr, "Maximo: %f , Minimo: %f\n", vmax, vmin);
#endif

	for(sptr = ygradiente, dptr = ygescalado; sptr < maxsptr; sptr++, dptr++)
	{
		*dptr = *sptr * M + B;	
	}

}
static void escalar_Uint8_Uint8(uint8_t * ygradiente, uint8_t * ygescalado, unsigned int width, unsigned int height)
{
	unsigned int size = width * height;
	uint8_t * sptr,
	* maxsptr = ygradiente + size;

	uint8_t * dptr,
	* maxdptr = ygescalado + size;

	double vmax = -9999, vmin = 9999;

	for(sptr = ygradiente; sptr < maxsptr; sptr++)
	{
		if(*sptr <= vmin) vmin = *sptr;
		if(*sptr >= vmax) vmax = *sptr;
	}

	double M, B;

	getM_B(vmax, vmin, &M, &B);
#ifdef DEBUG
	fprintf(stderr, "Maximo: %f , Minimo: %f\n", vmax, vmin);
#endif

	for(sptr = ygradiente, dptr = ygescalado; sptr < maxsptr; sptr++, dptr++)
	{
		*dptr = (uint8_t)round(*sptr * M + B);	
	}

}


static void saveImage(int fd, uint8_t * header, unsigned int hlength, uint8_t * content, unsigned int clength)
{
	int writen = write(fd, header, hlength);
	writen += write(fd, content, clength);
#ifdef DEBUG
	fprintf(stderr, "Escritos: %d\n", writen);
#endif
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

	double M, B;

	getM_B(vmax, vmin, &M, &B);

#ifdef DEBUG
	fprintf(stderr, "Maximo: %d , Minimo: %d\n", vmax, vmin);
#endif

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


#ifdef TIMMING
static void mostrarTiempo(const char * info, struct timeval *t0, struct timeval *t1, long long *acc)
{
/*
	unsigned long long ust0 = t0->tv_sec * 1000000LL + t0->tv_usec;
	unsigned long long ust1 = t1->tv_sec * 1000000LL + t1->tv_usec;

	long long tdiff = ust1 - ust0;
	fprintf(stderr, "%s:\t%lld\n",info, tdiff);
*/
	long long elapsed = (t1->tv_sec-t0->tv_sec)*1000000LL + t1->tv_usec-t0->tv_usec;

	fprintf(stderr, "%s:\t%lld us\n",info, elapsed);
	if(acc != NULL)
		*acc += elapsed;
	
}
#endif
int main(int argc, char ** argv)
{
	int width, height, nangulos;
	int re; //indica si escribir por la salida estandar el pgm con los bordes
	unsigned int rgbLength, pixelLength;
	uint8_t *ppm, *rgb, *pgm, *gs;
	
	unsigned int tamFiltro;
	float * filtro;
	float sigma;
	char * outdir;
	struct stat dirInfo;

	unsigned int lth, uth;
	getOptions(argc, argv, &tamFiltro, &sigma, &uth, &lth, &re, &nangulos, &outdir);

	if(stat(outdir, &dirInfo) == -1)
	{
		fprintf(stderr, "Error al obtener los metadatos del fichero \"%s\"\n", outdir);
		exit(1);
	}

	if(!S_ISDIR(dirInfo.st_mode))
	{
		fprintf(stderr, "%s no es un directorio\n", outdir);
		exit(2);
	}

	char filePath[256];
	strcpy(filePath,outdir);
	strcat(filePath,"/");

	int pathLength = strlen(filePath);
	
	char * outputFileName = filePath + pathLength;
	//Canny set up
	filtro = (float*) malloc(tamFiltro * sizeof(float));

	getGaussianFilter(filtro, tamFiltro, sigma);

#ifdef DEBUG
	fprintf(stderr, "Path: %s , pathLength: %d\n", filePath, pathLength);
	fprintf(stderr, "Sigma: %f , Size: %d\n", sigma, tamFiltro);
	showFilter(stderr, filtro, tamFiltro);
#endif
	int gsize = 3;

	double ** Gx = (double **) malloc(gsize * sizeof(double*));
	double * Gxc = (double *) malloc(gsize * gsize *  sizeof(double));
	int f;
	double * gptr;
	for(f = 0, gptr = Gxc; f < gsize; f++, gptr += gsize)
	{
		Gx[f] = gptr;
	}

	Gx[0][0] = -1; Gx[0][1] = 0; Gx[0][2] = 1;
	Gx[1][0] = -2; Gx[1][1] = 0; Gx[1][2] = 2;
	Gx[2][0] = -1; Gx[2][1] = 0; Gx[2][2] = 1;
	
	double ** Gy = (double **) malloc(gsize * sizeof(double*));
	double * Gyc = (double *) malloc(gsize * gsize *  sizeof(double));
	for(f = 0, gptr = Gyc; f < gsize; f++, gptr += gsize)
	{
		Gy[f] = gptr;
	}

	Gy[0][0] = -1; Gy[0][1] = -2; Gy[0][2] = -1;
	Gy[1][0] =  0; Gy[1][1] =  0; Gy[1][2] =  0;
	Gy[2][0] =  1; Gy[2][1] =  2; Gy[2][2] =  1;


	//Hough set up
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
	}

#ifdef TIMMING
	struct timeval t0,t1;
	long long tacc = 0;
#endif

	if(read_ppmHeader(&width, &height) == 0)
	{
#ifdef DEBUG
		fprintf(stderr, "Recibido ppm header\nEsperando RGB...\n");
#endif
		pixelLength = width * height;
		rgbLength = pixelLength  * 3;	

#ifdef DEBUG
		fprintf(stderr, "Width: %d , Height: %d\n", width, height);
#endif
		ppm = (uint8_t*) malloc(rgbLength+50);
	
		int ppmhl = sprintf((char*)ppm, "P6\n%d %d\n255\n", width, height);

		rgb = ppm +  ppmhl;

		unsigned int n = fread(rgb, rgbLength, 1, stdin);

#ifdef DEBUG
		fprintf(stderr, "Leido RGB: %d (%d bytes)\nConvirtiendo a GrayScale...\n", n, rgbLength*n);
#endif

		pgm = (uint8_t*) malloc(pixelLength+50);
		int pgmhl = sprintf((char*)pgm, "P5\n%d %d\n255\n", width, height);

		unsigned int ppmLength = ppmhl + rgbLength;
		unsigned int pgmLength = pgmhl + pixelLength;
		gs = pgm + pgmhl;
		rgb2gs(rgb, gs, width, height);


		strcpy(outputFileName, "original-color.ppm");
		int fppm = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fppm < 0) showError();

		strcpy(outputFileName, "original-gs.pgm");
 		int fpgm = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fpgm < 0) showError();

		int res = write(fppm, ppm, ppmLength);
		if (res != ppmLength) showError(); 
		res = write(fpgm, pgm, pgmLength);
		if (res != pgmLength) showError(); 

		close(fppm);
		close(fpgm);

		unsigned char * gsFiltrado = (uint8_t*) malloc(pixelLength);
		double * xgradiente = (double*) malloc(pixelLength * sizeof(double));
		double * ygradiente = (double*) malloc(pixelLength * sizeof(double));
		double * mgradiente = (double*) malloc(pixelLength * sizeof(double));
		double * dgradiente = (double*) malloc(pixelLength * sizeof(double));
		uint8_t * dgdiscreta = (uint8_t*) malloc(pixelLength);
		double * mgthin = (double*) malloc(pixelLength * sizeof(double));
		uint8_t * bordes = (uint8_t *) malloc(pixelLength);
#ifdef TIMMING
		gettimeofday(&t0, NULL);
#endif 
		//Filtramos el ruido con la gausiana especificada por el usuario
		aplicarFiltro_Uint8_ConPrecision(gs, gsFiltrado, width, height, filtro, filtro, tamFiltro);
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("01-Filtrada", &t0,&t1,&tacc);
#endif
	
#ifdef TIMMING
		gettimeofday(&t0, NULL);
#endif
		//Obtenemos el cambio de intensidad del gradiente en X
		aplicarFiltroNoSeparable(gsFiltrado, xgradiente, width, height, Gx, gsize);
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("02-Gradiente en X", &t0,&t1,&tacc);
#endif

#ifdef TIMMING
		gettimeofday(&t0, NULL);
#endif
		//Obtenemos el cambio de intensidad del gradiente en Y
		aplicarFiltroNoSeparable(gsFiltrado, ygradiente, width, height, Gy, gsize);
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("03-Gradiente en Y", &t0,&t1,&tacc);
#endif
#ifdef TIMMING
		gettimeofday(&t0, NULL);
#endif
		//Obtenemos el modulo del gradiente
		obtenerModuloGradiente(xgradiente, ygradiente, mgradiente, width, height);
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("04-Modulo gradiente", &t0,&t1,&tacc);
#endif
#ifdef TIMMING

		gettimeofday(&t0, NULL);
#endif
		//Obtenemos la direccion del gradiente
		obtenerDireccionGradiente(xgradiente, ygradiente, dgradiente, width, height);
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("05-Direccion gradiente", &t0,&t1,&tacc);

#endif
#ifdef TIMMING

		gettimeofday(&t0, NULL);
#endif
		//Discretizamos la direccion del gradiente en 4 direcciones (0, 45, 90 y 135º), es decir: 0, 85, 170 y 255
		discretizarDireccionGradiente(dgradiente, dgdiscreta, width, height);
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("06-Direccion gradiente discreta", &t0,&t1,&tacc);

#endif
#ifdef TIMMING
		gettimeofday(&t0, NULL);
#endif
		//Supresion de los no maximos (para adelgazar los bordes resaltados antes de la umbralizacion)
		nonMaximum(mgradiente, dgdiscreta, mgthin, width, height);
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("07-Supresion no maximos", &t0,&t1,&tacc);
#endif
		uint8_t * xgescalado = (uint8_t*) malloc(pixelLength);
		uint8_t * ygescalado = (uint8_t*) malloc(pixelLength);
		uint8_t * mgescalado = (uint8_t*) malloc(pixelLength);
		uint8_t * dgescalado = (uint8_t*) malloc(pixelLength);
		uint8_t * dgdescalado = (uint8_t*) malloc(pixelLength);
		uint8_t * mgthinescalado = (uint8_t *) malloc(pixelLength);

		escalar_Double_Uint8(xgradiente, xgescalado, width, height);
		escalar_Double_Uint8(ygradiente, ygescalado, width, height);
		escalar_Double_Uint8(mgradiente, mgescalado, width, height);
		escalar_Double_Uint8(dgradiente, dgescalado, width, height);
		escalar_Uint8_Uint8(dgdiscreta, dgdescalado, width, height);
		escalar_Double_Uint8(mgthin, mgthinescalado, width, height);

#ifdef TIMMING
		gettimeofday(&t0, NULL);
#endif
		//Paso final de Canny: Umbralizacion
		hysteresis(mgthinescalado, bordes, uth, lth, width, height);
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("08-Umbralizacion", &t0,&t1,&tacc);

#endif

#ifdef DEBUG
		fprintf(stderr, "alto: %d , bajo: %d\n", uth, lth);
#endif

		strcpy(outputFileName, "01-filtrada.pgm");
		int ffiltrado = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (ffiltrado < 0) showError();


		strcpy(outputFileName, "02-xgradiente.pgm");
 		int fxgradiente = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fxgradiente < 0) showError();

		strcpy(outputFileName, "03-ygradiente.pgm");
		int fygradiente = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fygradiente < 0) showError();

		strcpy(outputFileName, "04-mgradiente.pgm");
		int fmgradiente = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fmgradiente < 0) showError();

		strcpy(outputFileName, "05-dgradiente.pgm");
		int fdgradiente = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fdgradiente < 0) showError();

		strcpy(outputFileName, "06-dgradiente-discreta.pgm");
		int fdgradiente_discreta = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fdgradiente_discreta < 0) showError();

		strcpy(outputFileName, "07-mgradiente-nonmaximum.pgm");
		int fmgthin = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fmgthin < 0) showError();

		strcpy(outputFileName, "08-bordes.pgm");
		int fbordes = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fbordes < 0) showError();



	
		saveImage(ffiltrado, pgm, pgmhl, gsFiltrado, pixelLength);
		free(gsFiltrado);
		saveImage(fxgradiente, pgm, pgmhl, xgescalado, pixelLength);
		free(xgescalado);
		saveImage(fygradiente, pgm, pgmhl, ygescalado, pixelLength);
		free(ygescalado);
		saveImage(fmgradiente, pgm, pgmhl, mgescalado, pixelLength);
		free(mgescalado);
		saveImage(fdgradiente, pgm, pgmhl, dgescalado, pixelLength);
		free(dgescalado);
		saveImage(fdgradiente_discreta, pgm, pgmhl, dgdescalado, pixelLength);
		free(dgdescalado);
		saveImage(fmgthin, pgm, pgmhl, mgthinescalado, pixelLength);
		free(mgthinescalado);
		saveImage(fbordes, pgm, pgmhl, bordes, pixelLength);

		free(ppm);
		free(xgradiente);
		free(ygradiente);
		free(mgradiente);
		free(dgradiente);

		close(ffiltrado);
		close(fxgradiente);
		close(fygradiente);
		close(fmgradiente);
		close(fdgradiente);
		close(fdgradiente_discreta);
		close(fmgthin);
		close(fbordes);

		//FIN CANNY
		//HOUGH
		unsigned int rhoMax = sqrt(width*width + height*height);
		unsigned int ndistancias = rhoMax;

#ifdef DEBUG
		fprintf(stderr, "Angulos: %d, Distancias: %d\n", nangulos, ndistancias);
#endif

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
#ifdef TIMMING
		gettimeofday(&t0, NULL);
#endif
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
#ifdef TIMMING
		gettimeofday(&t1, NULL);
		mostrarTiempo("09-Obtencion Hough space", &t0,&t1,&tacc);
		fprintf(stderr, "TOTAL:\t%lld us\n", tacc);
#endif

		pgmhl = sprintf((char*)pgm, "P5\n%d %d\n255\n", nangulos, ndistancias);

		strcpy(outputFileName, "09-houghSp.pgm");
		int fhoughSp = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fhoughSp < 0) showError();


		saveImage(fhoughSp, pgm, pgmhl, houghSp, houghSpLength);

		uint8_t * houghSpAccEscalado = (uint8_t *) malloc(houghSpLength);
		
		escalar_Int_Uint8(acc, houghSpAccEscalado, houghSpLength);

		strcpy(outputFileName, "10-houghSpAcc.pgm");
		int fhoughSpAcc = open(filePath, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fhoughSpAcc < 0) showError();

		invertirValores(houghSpAccEscalado, houghSpLength);
		saveImage(fhoughSpAcc, pgm, pgmhl, houghSpAccEscalado, houghSpLength);

		if(re)
			saveImage(1, pgm, pgmhl, houghSpAccEscalado, houghSpLength);

		close(fhoughSp);
		close(fhoughSpAcc);

		free(accM);
		free(acc);
		free(bordes);
		free(bordesM);
		free(pgm);
		free(houghSpAccEscalado);

	}
	free(filtro);
	free(Gx);
	free(Gxc);
	free(Gy);
	free(Gyc);
	free(cosTable);
	free(sinTable);
	return 0;
}



