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
	fprintf(stderr, "%s [options] < infile > outfile\n"
			"Options:\n"
			"\t-l               - Tamano del filtro gausiano (>=3 e impar).\n"
			"\t-s               - Sigma del filtro gausiano.\n"
			"\t-j               - Umbral inferior para deteccion de borde (0 < x < 255).\n"
			"\t-k               - Umbral superior para deteccion de borde (0 < x < 255).\n", pgmname);
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
getOptions(int argc, char *argv[], int * filterSize, float *sigma, unsigned int *lth, unsigned int * uth)
{
	int opt;
	int error = 0;
	*filterSize = -1; *sigma = -1;
	while ((opt = getopt(argc, argv, "l:s:j:k:")) != -1) {
		switch (opt) {
		case 'l':
			error += optGetIntError(optarg, filterSize, 0);
			break;
		case 's':
			error += optGetFloatError(optarg, sigma, 0);
			break;
		case 'j':
			error += optGetIntError(optarg, lth, -1);
			break;
		case 'k':
			error += optGetIntError(optarg, uth, -1);
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

	//uint8_t * dptr;
	double * dptr;

	//dptr = dst + offsetInicial;
	dptr = bufferDeTrabajo + offsetInicial;

	//uint8_t * maxf = dptr + width * filas;
	double * maxf = dptr + width * filas;
	

	float * centroFiltro = hfiltro + foffset;

#ifdef DEBUG
	fprintf(stderr, "Aplicando filtro horizontal...\n");
	double valorMaximo = -9999, valorMinimo = 9999;
#endif
	while(dptr < maxf)
	{
		//uint8_t * maxc = dptr + columnas;
		double * maxc = dptr + columnas;
		while(dptr < maxc)
		{
			//Aplicamos el filtro horizontalmente
			//uint8_t valorFinal = 0;
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
			dptr += 1;
		}
		dptr += ioffset;
	}

	//dptr = dst + offsetInicial;
	dptr = bufferDeTrabajo + offsetInicial;
//	uint8_t * maxc = dptr + columnas;
	double * maxc = dptr + columnas;

#ifdef TEST
	fprintf(stderr, "Valor maximo: %f, Valor minimo: %f, Aplicando filtro vertical...\n", valorMaximo, valorMinimo);
#endif

	centroFiltro = vfiltro + foffset;
	while(dptr < maxc)
	{
		//uint8_t * dptrc = dptr;
		double * dptrc = dptr;
		maxf = dptr + filas * width;
		while(dptr < maxf)
		{
			//Aplicamos el filtro verticalmente
			//uint8_t valorFinal = 0;
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
			dptr += width;
		}
		dptr = dptrc + 1;
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

static aplicarFiltroNoSeparable(uint8_t * src, double * dst, unsigned int width, unsigned int height, double ** filtro, unsigned int tfiltro)
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
	double * xptr = xg, *yptr = yg, *mptr = mg;
	double * maxxptr = xptr + width * height;

	while(xptr < maxxptr)
	{
		*mptr = sqrt(*xptr**xptr +  *yptr**yptr);
		mptr++;		
		xptr++;
		yptr++;
	}
}

static void obtenerDireccionGradiente(double * xg, double * yg, double * mg, unsigned int width, unsigned int height)
{
	unsigned int length = width * height;
	double * xptr = xg, *yptr = yg, *mptr = mg;
	double * maxxptr = xptr + width * height;

	while(xptr < maxxptr)
	{
		*mptr = atan(*yptr / *xptr);
		mptr++;		
		xptr++;
		yptr++;
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
				//miramos los vecinos
				if(srcM[f-1][c-1] > alto 
				|| srcM[f-1][c] > alto
				|| srcM[f-1][c+1] > alto
				|| srcM[f][c-1] > alto
				|| srcM[f][c+1] > alto
				|| srcM[f+1][c-1] > alto
				|| srcM[f+1][c] > alto
				|| srcM[f+1][c+1] > alto)
					dstM[f][c] = vmax;
				else
					dstM[f][c] = vmax;
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
	fprintf(stderr, "Maximo: %f , Minimo: %f\n", vmax, vmin);

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
	fprintf(stderr, "Maximo: %f , Minimo: %f\n", vmax, vmin);

	for(sptr = ygradiente, dptr = ygescalado; sptr < maxsptr; sptr++, dptr++)
	{
		*dptr = (uint8_t)round(*sptr * M + B);	
	}

}


static void saveImage(int fd, uint8_t * header, unsigned int hlength, uint8_t * content, unsigned int clength)
{
	int writen = write(fd, header, hlength);
	writen += write(fd, content, clength);
	fprintf(stderr, "Escritos: %d\n", writen);
}

int main(int argc, char ** argv)
{

	int width, height;
	unsigned int rgbLength, pixelLength;
	uint8_t *ppm, *rgb, *pgm, *gs;
	
	unsigned int tamFiltro;
	float * filtro;
	float sigma;

	unsigned int lth, uth;
	getOptions(argc, argv, &tamFiltro, &sigma, &lth, &uth);

	filtro = (float*) malloc(tamFiltro * sizeof(float));

	getGaussianFilter(filtro, tamFiltro, sigma);

	fprintf(stderr, "Sigma: %f , Size: %d\n", sigma, tamFiltro);
	showFilter(stderr, filtro, tamFiltro);

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
/*
	Gx[0][0] = 1; Gx[0][1] = 0; Gx[0][2] = -1;
	Gx[1][0] = 2; Gx[1][1] = 0; Gx[1][2] = -2;
	Gx[2][0] = 1; Gx[2][1] = 0; Gx[2][2] = -1;
*/
	double ** Gy = (double **) malloc(gsize * sizeof(double*));
	double * Gyc = (double *) malloc(gsize * gsize *  sizeof(double));
	for(f = 0, gptr = Gyc; f < gsize; f++, gptr += gsize)
	{
		Gy[f] = gptr;
	}

	Gy[0][0] = -1; Gy[0][1] = -2; Gy[0][2] = -1;
	Gy[1][0] =  0; Gy[1][1] =  0; Gy[1][2] =  0;
	Gy[2][0] =  1; Gy[2][1] =  2; Gy[2][2] =  1;
/*
	Gy[0][0] =  1; Gy[0][1] =  2; Gy[0][2] =  1;
	Gy[1][0] =  0; Gy[1][1] =  0; Gy[1][2] =  0;
	Gy[2][0] = -1; Gy[2][1] = -2; Gy[2][2] = -1;
*/

	unsigned int gSize = 3;
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


		int fppm = open("original-color.ppm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fppm < 0) showError();
 		int fpgm = open("original-gs.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
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

		//Filtramos el ruido con la gausiana especificada por el usuario
		aplicarFiltro_Uint8_ConPrecision(gs, gsFiltrado, width, height, filtro, filtro, tamFiltro);
	
		//Obtenemos el cambio de intensidad del gradiente en X
		aplicarFiltroNoSeparable(gsFiltrado, xgradiente, width, height, Gx, gsize);
		uint8_t * xgescalado = (uint8_t*) malloc(pixelLength);
		escalar_Double_Uint8(xgradiente, xgescalado, width, height);

		//Obtenemos el cambio de intensidad del gradiente en Y
		aplicarFiltroNoSeparable(gsFiltrado, ygradiente, width, height, Gy, gsize);
		uint8_t * ygescalado = (uint8_t*) malloc(pixelLength);
		escalar_Double_Uint8(ygradiente, ygescalado, width, height);
		
		//Obtenemos el modulo del gradiente
		obtenerModuloGradiente(xgradiente, ygradiente, mgradiente, width, height);
		uint8_t * mgescalado = (uint8_t*) malloc(pixelLength);
		escalar_Double_Uint8(mgradiente, mgescalado, width, height);

		//Obtenemos la direccion del gradiente
		obtenerDireccionGradiente(xgradiente, ygradiente, dgradiente, width, height);
		uint8_t * dgescalado = (uint8_t*) malloc(pixelLength);
		escalar_Double_Uint8(dgradiente, dgescalado, width, height);

		//Discretizamos la direccion del gradiente en 4 direcciones (0, 45, 90 y 135º), es decir: 0, 85, 170 y 255
		uint8_t * dgdiscreta = (uint8_t*) malloc(pixelLength);
		discretizarDireccionGradiente(dgradiente, dgdiscreta, width, height);
		uint8_t * dgdescalado = (uint8_t*) malloc(pixelLength);
		escalar_Uint8_Uint8(dgdiscreta, dgdescalado, width, height);

		//Supresion de los no maximos (para adelgazar los bordes resaltados antes de la umbralizacion)
		double * mgthin = (double*) malloc(pixelLength * sizeof(double));
		nonMaximum(mgradiente, dgdiscreta, mgthin, width, height);
		uint8_t * mgthinescalado = (uint8_t *) malloc(pixelLength);
		escalar_Double_Uint8(mgthin, mgthinescalado, width, height);

		//Paso final de Canny: Umbralizacion
		uint8_t * bordes = (uint8_t *) malloc(pixelLength);
		hysteresis(mgthinescalado, bordes, uth, lth, width, height);
		fprintf(stderr, "alto: %d , bajo: %d\n", uth, lth);
		int ffiltrado = open("01-filtrada.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (ffiltrado < 0) showError();
 		int fxgradiente = open("02-xgradiente.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fxgradiente < 0) showError();
		int fygradiente = open("03-ygradiente.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fygradiente < 0) showError();
		int fmgradiente = open("04-mgradiente.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fmgradiente < 0) showError();
		int fdgradiente = open("05-dgradiente.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fdgradiente < 0) showError();
		int fdgradiente_discreta = open("06-dgradiente-discreta.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fdgradiente_discreta < 0) showError();
		int fmgthin = open("07-mgradiente-nonmaximum.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fmgthin < 0) showError();
		int fbordes = open("08-bordes.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fbordes < 0) showError();



	
		saveImage(ffiltrado, pgm, pgmhl, gsFiltrado, pixelLength);
		saveImage(fxgradiente, pgm, pgmhl, xgescalado, pixelLength);
		saveImage(fygradiente, pgm, pgmhl, ygescalado, pixelLength);
		saveImage(fmgradiente, pgm, pgmhl, mgescalado, pixelLength);
		saveImage(fdgradiente, pgm, pgmhl, dgescalado, pixelLength);
		saveImage(fdgradiente_discreta, pgm, pgmhl, dgdescalado, pixelLength);
		saveImage(fmgthin, pgm, pgmhl, mgthinescalado, pixelLength);
		saveImage(fbordes, pgm, pgmhl, bordes, pixelLength);

		close(ffiltrado);
		close(fxgradiente);
		close(fygradiente);
		close(fmgradiente);
		close(fdgradiente);
		close(fdgradiente_discreta);
		close(fmgthin);
		close(fbordes);



	}
	return 0;
}



