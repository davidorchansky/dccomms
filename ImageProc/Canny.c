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

void aplicarFiltro_Uint8_ConPrecision(uint8_t * src, uint8_t * dst, unsigned int width, unsigned int height, float * hfiltro, float * vfiltro, unsigned int tamFiltro)
{
	unsigned int size = height * width;
	
	double * bufferDeTrabajo = (double *) malloc(size * sizeof(double));
	//Copiamos todo en bufferDeTrabajo para trabajar con mayor precision
	int pos;
	for(pos = 0; pos < size; pos++)
	{
		bufferDeTrabajo[pos] = (double) src[pos];
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
#ifdef TEST
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
	for(pos = 0; pos < size; pos++)
	{
		dst[pos] = (uint8_t) bufferDeTrabajo[pos];
	}

	free(bufferDeTrabajo);

}

void aplicarFiltro_Double(uint8_t * src, double * dst, unsigned int width, unsigned int height, float * hfiltro, float * vfiltro, unsigned int tamFiltro)
{
	unsigned int size = height * width;
	
	//Copiamos todo en dst
	int pos;
	for(pos = 0; pos < size; pos++)
	{
		dst[pos] = (double) src[pos];
	}

	int foffset = tamFiltro >> 1;
	int ioffset = foffset << 1;


	//Numero de filas y columnas de la zona central (descartamos bordes de la imagen del grosor del filtro)
	int filas    = height - ioffset;
	int columnas = width - ioffset;

	
	//Pasada por filas (aplicación del filtro horizontal)
	unsigned int offsetInicial = foffset * width + foffset;

	double * dptr;

	dptr = dst + offsetInicial;

	double * maxf = dptr + width * filas;
	
#ifdef DEBUG
	fprintf(stderr, "Aplicando filtro horizontal...\n");
	double valorMaximo = -9999, valorMinimo = 9999;
#endif

	float * centroFiltro = hfiltro + foffset;
	
	while(dptr < maxf)
	{
		double * maxc = dptr + columnas;
		while(dptr < maxc)
		{
			//Aplicamos el filtro horizontalmente
			double valorFinal = 0;
			int idx;
			for(idx = -foffset; idx <= foffset; idx++)
			{
				valorFinal += *(dptr+idx) * *(centroFiltro+idx);

			}
			*dptr = valorFinal;
			dptr += 1;
#ifdef DEBUG
			if(valorFinal >= valorMaximo) valorMaximo = valorFinal;
			if(valorFinal <= valorMinimo) valorMinimo = valorFinal;
#endif

		}
		dptr += ioffset;
	}

//	return;
	dptr = dst + offsetInicial;
	double * maxc = dptr + columnas;

	fprintf(stderr, "Aplicando filtro vertical...\n");

	centroFiltro = vfiltro + foffset;
	while(dptr < maxc)
	{
		double * dptrc = dptr;
		maxf = dptr + filas * width;
		while(dptr < maxf)
		{
			//Aplicamos el filtro verticalmente
			double valorFinal = 0;
			int idx;
			for(idx = -foffset; idx <= foffset; idx++)
			{
				valorFinal += *(dptr+idx*(int)width) * *(centroFiltro+idx);

			}
			*dptr = valorFinal;
			dptr += width;
#ifdef DEBUG
			if(valorFinal >= valorMaximo) valorMaximo = valorFinal;
			if(valorFinal <= valorMinimo) valorMinimo = valorFinal;
#endif

		}
		dptr = dptrc + 1;
	}
#ifdef DEBUG
	fprintf(stderr, "Valor maximo: %f, Valor minimo: %f\n", valorMaximo, valorMinimo);
#endif


}
void aplicarFiltro_Uint8(uint8_t * src, uint8_t * dst, unsigned int width, unsigned int height, float * hfiltro, float * vfiltro, unsigned int tamFiltro)
{
	unsigned int size = height * width;
	
	//Copiamos todo en dst
	memcpy(dst, src, size);

	int foffset = tamFiltro >> 1;
	int ioffset = foffset << 1;


	//Numero de filas y columnas de la zona central (descartamos bordes de la imagen del grosor del filtro)
	int filas    = height - ioffset;
	int columnas = width - ioffset;

	
	//Pasada por filas (aplicación del filtro horizontal)
	unsigned int offsetInicial = foffset * width + foffset;

	uint8_t * dptr;

	dptr = dst + offsetInicial;

	uint8_t * maxf = dptr + width * filas;
	
#ifdef DEBUG
	fprintf(stderr, "Aplicando filtro horizontal...\n");
	double valorMaximo = -9999, valorMinimo = 9999;
#endif

	float * centroFiltro = hfiltro + foffset;
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
				valorFinal += *(dptr+idx) * *(centroFiltro+idx);
			}
			*dptr = valorFinal;
			dptr += 1;
#ifdef DEBUG
			if(valorFinal >= valorMaximo) valorMaximo = valorFinal;
			if(valorFinal <= valorMinimo) valorMinimo = valorFinal;
#endif


		}
		dptr += ioffset;
	}

	dptr = dst + offsetInicial;
	uint8_t * maxc = dptr + columnas;

	fprintf(stderr, "Aplicando filtro vertical...\n");

	centroFiltro = vfiltro + foffset;
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
#ifdef DEBUG
			if(valorFinal >= valorMaximo) valorMaximo = valorFinal;
			if(valorFinal <= valorMinimo) valorMinimo = valorFinal;
#endif
		}
		dptr = dptrc + 1;
	}
#ifdef DEBUG
	fprintf(stderr, "Valor maximo: %f, Valor minimo: %f\n", valorMaximo, valorMinimo);
#endif


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

}
static void obtenerModuloGradiente(uint8_t * xg, uint8_t * yg, uint8_t * mg, unsigned int width, unsigned int height)
{
	unsigned int length = width * height;
	uint8_t * xptr = xg, *yptr = yg, *mptr = mg;
	uint8_t * maxxptr = xptr + width * height;

	while(xptr < maxxptr)
	{

		*mptr = sqrt(*xptr**xptr +  *yptr**yptr);

	//	fprintf(stderr, "XG: %d , XG: %d, MG = %d (%f)\n", *xptr, *yptr, *mptr,sqrt(*xptr**xptr +  *yptr**yptr));
		mptr++;		
		xptr++;
		yptr++;
	}
}

static void obtenerDireccionGradiente(uint8_t * xg, uint8_t * yg, uint8_t * mg, unsigned int width, unsigned int height)
{

}

static void showError()
{
	fprintf(stderr, "Ha ocurrido algun error: %s\n", strerror(errno));
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
	
	getOptions(argc, argv, &tamFiltro, &sigma);

	filtro = (float*) malloc(tamFiltro * sizeof(float));

	getGaussianFilter(filtro, tamFiltro, sigma);

	fprintf(stderr, "Sigma: %f , Size: %d\n", sigma, tamFiltro);
	showFilter(stderr, filtro, tamFiltro);

	int gsize = 3;

	double ** Gx = (double **) malloc(gsize * sizeof(double*));

	int f;
	for(f = 0; f < gsize; f++)
	{
		Gx[f] = (double *) malloc(gsize* sizeof(double));
	}

	Gx[0][0] = -1; Gx[0][1] = 0; Gx[0][2] = 1;
	Gx[1][0] = -2; Gx[1][1] = 0; Gx[1][2] = 2;
	Gx[2][0] = -1; Gx[2][1] = 0; Gx[2][2] = 1;


/*
	float Gxh[3] = {-1, 0, 1};
	float Gxv[3] = { 1, 2, 1};

	float Gyh[3] = { 1, 2, 1};
	float Gyv[3] = {-1, 0, 1};
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
		//aplicarFiltroConPrecision(gs, gsFiltrado, width, height, filtro, filtro, tamFiltro);
	
		//Obtenemos el cambio de intensidad del gradiente en X
		//aplicarFiltro_Double(gsFiltrado, xgradiente, width, height, Gxh, Gxv, gSize);
		aplicarFiltroNoSeparable(gsFiltrado, xgradiente, width, height, Gx, 3);
		uint8_t * xgescalado = (uint8_t*) malloc(pixelLength);
		escalar_Double_Uint8(xgradiente, xgescalado, width, height);
		//aplicarFiltroConPrecision(gsFiltrado, xgradiente, width, height, Gxh, Gxv, gSize);

		//Obtenemos el cambio de intensidad del gradiente en Y
		//aplicarFiltro_Double(gsFiltrado, ygradiente, width, height, Gyh, Gyv, gSize);
		uint8_t * ygescalado = (uint8_t*) malloc(pixelLength);
		//escalar_Double_Uint8(ygradiente, ygescalado, width, height);
		//aplicarFiltroConPrecision(gsFiltrado, ygradiente, width, height, Gyh, Gyv, gSize);
		
		//Obtenemos el modulo del gradiente
		//obtenerModuloGradiente(xgradiente, ygradiente, mgradiente, width, height);

		//Obtenemos la direccion del gradiente
		//obtenerDireccionGradiente(xgradiente, ygradiente, dgradiente, width, height);


		int ffiltrado = open("01-filtrada.ppm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (ffiltrado < 0) showError();
 		int fxgradiente = open("02-xgradiente.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fxgradiente < 0) showError();
		int fygradiente = open("03-ygradiente.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fygradiente < 0) showError();
		int fmgradiente = open("04-mgradiente.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fmgradiente < 0) showError();
		int fdgradiente = open("05-dgradiente.pgm", O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR | S_IROTH );
		if (fdgradiente < 0) showError();

	
		saveImage(ffiltrado, pgm, pgmhl, gsFiltrado, pixelLength);
		saveImage(fxgradiente, pgm, pgmhl, xgescalado, pixelLength);
		saveImage(fygradiente, pgm, pgmhl, ygescalado, pixelLength);
		//saveImage(fmgradiente, pgm, pgmhl, mgradiente, pixelLength);
		//saveImage(fdgradiente, pgm, pgmhl, dgradiente, pixelLength);

		close(ffiltrado);
		close(fxgradiente);
		close(fygradiente);
		close(fmgradiente);
		close(fdgradiente);




	}
	return 0;
}



