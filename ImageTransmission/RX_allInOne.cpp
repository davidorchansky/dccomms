//============================================================================
// Name        : TX2.cpp
// Author      : Diego
// Version     :
// Copyright   :
// Description : Sends an image per bucle iteration
//============================================================================

#include <iostream>
#include <SerialPortInterface.h>
#include <Radio.h>
#include <BlockRadioTransmitter.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>			// std::chrono::seconds
#include <DataLinkFrame.h>
#include <stdint.h>
#include <Utils.h>
#include <RadioException.h>
#include <stdlib.h>
#include <cstdio>
#include <sys/stat.h>
#include <cstring>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <Arduino.h>
#include <videoServer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

extern "C"
{
#include <libdebt.h>
#include <grabberDebter.h>
}

using namespace std;
using namespace radiotransmission;

int
readId(char* id, int len)
{
	char c;
	for(int i = 0; i < len; i++)
	{
		c = getchar();
	//	std::cout << "#"<<c<<std::endl;
		if(c != id[i])
			return -1;

	}
	return 0;
}

unsigned int getMilliseconds(struct timeval * t)
{
        return  (*t).tv_sec *1000 + (*t).tv_usec/1000;
}

static void
usage(char *pgmname, struct debtEncParam *e, struct debtDecParam *d)
{
	fprintf(stderr,
			"usage: %s (-d|-e)[options] < infile > outfile\n"
			"Options:\n"
			"\t-v [0|1]         - Use vector instructions if possible <%d>\n"
			"\n"
			"Decoding: -d\n"
			"\t-b [bias]        - Bias used when decoding (default, zero, mid, exp, midexp) <%s>\n"
			"\t-a [bias weight] - Bias weight to use with bias <%f>\n"
			"\t-c [size]        - Truncate input file to [size] bytes <%d>\n"
			"\nEncoding: -e\n"
			"\t-t transform     - Wavelet used (CDF-9/7) and (5/3, 9/7-M, 13/7-T, haar) <%s>\n"
			"\t-n nbands        - Number of transformations <%d>\n"
			"\t-k depth         - Decompose for depth <= otherwise partition <%d>\n"
			"\t-y val           - Dynamic decomposition/partition <%d>\n"
			"\t-r refmul        - Refinement mutliplier <%f>\n"
			"\t-q quality       - Quality value (0 = max) <%f>\n"
			"\t-u resolution    - Resolution order <%d>\n"
			"\t-x maxsize       - Truncate output at maxsize bytes <%d>\n"
			"\n"
			"\t-p pad           - Pad codes when mapping <%d>\n"
			"\t-m map           - Output map codes <%d>\n"
			"\t-s esc           - Escape 0xff on output stream <%d>\n"
			"\n"
			"\t-l left-shift    - ROI left shift <%d>\n"
			"\t-i x0,y0,x1,y1   - ROI rectangle (Top-Left and Bottom-Right) (repeat)\n"
			"\n"
			"\t-h               - Benchmark <%d>\n"
			"\t-f               - Print encoding statistics <%d>\n"
			"\t-I               - Specify an ASCII identifier for the image (it is like a header)\n",

			pgmname,
			e->vector,
			bias_name(d->bias), d->weight, d->bitlen >> 3,
			transform_name(e->transform), e->nbands, e->maxk, e->dynamic, e->refmul, e->quality, e->resolution, e->maxsize,
			e->pad, e->map, e->esc,
			e->uroi_shift,
			e->benchmark, e->timestats);
	exit(-1);
}

/* -d to decode or -e to encode */
/* returns -1 to graph, 0 to decode, 1 to encode */
static int
getOptions(int argc, char *argv[], struct debtEncParam *e, struct debtDecParam *d, char** imId, int* imIdSize, 	videoTransmissionConfig* config)
{
	int opt;
	int error = 0;
	int xi = 0;
	int dec = 0;
	int enc = 0;
	int graph = 0;

	while ((opt = getopt(argc, argv, "v:db:a:c:et:n:k:r:q:u:x:p:m:s:l:i:y:gf:h:I:W:H:F:L:D:A:")) != -1) {
		switch (opt) {
		case 'I':
			*imId = optarg;
			*imIdSize = strlen(optarg);
			break;
		case 'v':
			error += optGetIntError(optarg, &e->vector, 0);
			break;
		case 'h':
			error += optGetIntError(optarg, &e->benchmark, 0);
			break;
		case 'f':
			error += optGetIntError(optarg, &e->timestats, 0);
			break;
		case 'g':
			/* special hidden option to calculate rate x psnr curve */
			graph = 1;
			break;
		case 'd':
			++dec;
			break;
		case 'b':
			if (!optarg) {
				d->bias = BIAS_DEF;
			} else {
				int b = bias_code(optarg);
				if (b != -1)
					d->bias = b;
				else
					++error;
			}
			break;
		case 'a':
			error += optGetFloatError(optarg, &d->weight, 0.0);
			break;
		case 'c':
			if (optGetIntError(optarg, &xi, 0))
				++error;
			else
				d->bitlen = xi << 3;
			break;
		case 'e':
			++enc;
			break;
		case 't':
			if (!optarg) {
				e->transform = TRANSFORM_I_B_13x7T;
			} else {
				int t = transform_code(optarg);
				if (t != -1)
					e->transform = t;
				else
					++error;
			}
			break;
		case 'n':
			error += optGetIntError(optarg, &e->nbands, 0);
			break;
		case 'k':
			error += optGetIntError(optarg, &e->maxk, 0);
			break;
		case 'r':
			error += optGetFloatError(optarg, &e->refmul, 0.0);
			break;
		case 'q':
			error += optGetFloatError(optarg, &e->quality, 0.0);
			break;
		case 'u':
			error += optGetIntError(optarg, &e->resolution, 0);
			break;
		case 'x':
			error += optGetIntError(optarg, &e->maxsize, 0);
			break;
		case 'p':
			error += optGetIntError(optarg, &e->pad, 0);
			break;
		case 'm':
			error += optGetIntError(optarg, &e->map, 0);
			break;
		case 's':
			error += optGetIntError(optarg, &e->esc, 0);
			break;
		case 'l':
			error += optGetIntError(optarg, &e->uroi_shift, 0);
			break;
		case 'i':
			error += optGetRect(e, optarg);
			break;
		case 'y':
			error += optGetIntError(optarg, &e->dynamic, 0);
			break;
		default: /* '?' */
			usage(argv[0], e, d);
		}
	}

	if (error) {
		usage(pgmName(argv[0]), e, d);
	}
	if (optind < argc) {
		fprintf(stderr, "Unexpected argument(s) after options\n");
		usage(pgmName(argv[0]), e, d);
	}
	if (dec && (enc || graph)) {
		fprintf(stderr, "Decode and Encode flags cannot be used simultaneously\n");
		usage(pgmName(argv[0]), e, d);
	}
	return graph ? -1 : !dec;
}

static void
defaultParams(struct debtEncParam *e, struct debtDecParam *d)
{
	debtEncParam_init(e);
	e->vector = 1;
	debtDecParam_init(d);
}


int main(int argc, char ** argv) {
	struct debtEncParam e;
	struct debtDecParam d;

	defaultParams(&e, &d);

	char * imId;

	videoTransmissionConfig config;

	int bIdLength;
	int isEncoder = 0, isDecoder = 0;
	int act = getOptions(argc, argv, &e, &d, &imId, &bIdLength, &config);

	int ret = (act == -1) ? graph(&e, &d) : (!act ? isDec(&isDecoder) : isEnc(&isEncoder));
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction = segfault_sigaction;
	sa.sa_flags   = SA_SIGINFO;

	sigaction(SIGSEGV, &sa, NULL);

	try
	{
		Arduino arduRx = Arduino::FindArduino(Arduino::BAUD_115200,
				"Hello, are you RX?\n",
				"Yes, I'm RX");

		if(!arduRx.IsOpen())
		{
			std::cerr << "No ha sido posible encontrar la arduino" << std::endl;
			exit(2);
		}

		std::cerr <<"RX listo\n";

		uint8_t rxbuffer[1280*720*3*2+30]; //Unos 30 bytes mas para el header (que no sabemos, de momento, cuanto ocupa, y que depende de la resolucion de cada imagen recibida

		Radio radioRx(1,arduRx);


		BlockRadioTransmitter fileRx(radioRx);

		uint32_t fsize;
		struct timeval time0, time1;

		unsigned long long t0;
		unsigned long long t1;
		unsigned long long tdif;

		double isize = 0;
		double itime = 0;
		uint32_t it = 0;
		uint32_t cont = 0;
		double aitime;
		double aisize;

		double abstime;

		namedWindow("ROV", CV_WINDOW_NORMAL);
		while(1)
		{
			try
			{
				std::cerr << "BUSCANDO BLOQUE..." << std::endl;

				arduRx.FlushInput();

				gettimeofday(&time0, NULL);
				t0 = time0.tv_sec*1000 + time0.tv_usec/1000;
				

				//Recepción en modo bloqueante (Se bloquea hasta recibir un bloque válido)
				fsize = fileRx.Receive(imId, rxbuffer);


				gettimeofday(&time1, NULL);
				t1 = time1.tv_sec*1000 + time1.tv_usec/1000;
	

				/*Recepción del bloque en modo "Timeout", establecido a 10 segundos.
				 *Nota: el timeout indica el tiempo máximo que se debe esperar para la re
				 * cepción del siguiente byte. En caso de transcurrir el tiempo indicado,
				 * se producirá una RadioException de tipo RADIO_TIMEOUT.

				fileRx.Receive("Imagen", rxbuffer, 10000);

				*/
				std::cerr << "NUEVO BLOQUE RECIBID0!" << std::endl;

				tdif = t1 - t0;
				std::cerr << "Tiempo de recepción: " << tdif << std::endl <<std::endl;

				std::cerr << "DECODER: decodificando..." << std::endl;

				struct imgBuffer * img = imgBuffer_new();

				d.bitlen = fsize << 3;
				decode(&d, rxbuffer, img);
				
				int ysize = img->width * img->height;
				int uvsize = img->uvwidth * img->uvheight;
				int yuvsize = ysize + (uvsize << 1);
			

				uint8_t *y = img->buffer;
				uint8_t *u = y + ysize;
				uint8_t *v = u + uvsize;

				std::cerr << "yb: " << (unsigned long) y << std::endl;
				std::cerr << "Width: " << img->width << " Height: " << img->height << std::endl;
				std::cerr << "Y SIZE: " << ysize << " U and V SIZE: " << uvsize << std::endl;

				uint8_t * ppm = rxbuffer;
				int hl = sprintf((char*)ppm,"P6\n%d %d\n255\n", img->width, img->height);

				std::cerr << "Head length: " << hl << std::endl;
				uint8_t * ppmImg = ppm + hl;
				yuv420p_to_rgb(img->width, img->height, y, u, v, ppmImg);

				FILE * outputFile = fopen("RES.ppm", "wb");

				fwrite(ppm, 1, hl+img->width*img->height*3, outputFile);

				fclose(outputFile);

				imgBuffer_del(img);
				
				//std::vector<uint8_t> ppmv;
				//ppmv.assign(ppm, ppm + hl + img->width * img->height * 3);
				Mat decimg = imread("RES.ppm", CV_LOAD_IMAGE_COLOR);

				imshow("ROV",decimg);

				waitKey(1);

				it++;
				if(it > 3)
				{
					cont++;
					isize += fsize;
					itime += tdif;
					aitime = itime/cont;
					aisize = isize/cont;
					abstime = aitime/aisize;
					std::cerr << "Bloques. recibidao:\t" << cont << std::endl;
					std::cerr << "Tamaño medio (bytes):\t" << aisize << std::endl;
					std::cerr << "Tiempo medio (ms):\t" << aitime << std::endl;
					std::cerr << "ms/byte:\t" << abstime << std::endl;

				}

				
			}
			catch(RadioException& e) //Control de excepciones
			{
				//std::cerr << "Radio Exception: " << e.what() << std::endl << std::flush;
				switch(e.code)
				{
					case RADIO_RXLINEDOWN: //Se ha perdido la comunicación con la arduino receptora
						std::cerr << "Intentando reconectar con RX..." << std::endl << std::flush;
						while(!arduRx.TryReconnect()){};
						std::cerr << "ÉXITO!!" << std::endl << std::flush;
						break;
					case RADIO_TIMEOUT:
						std::cerr << "RADIO EXCEPTION: TIMEOUT!" << std::endl << std::flush;
						break;
					case RADIO_CORRUPTBLOCK:
						std::cerr << "RADIO EXCEPTION: BLOQUE CON ERRORES (SE DESCARTA)" << std::endl << std::flush;
						break;
					default:
						std::cerr << "RADIO EXCEPTION: " << e.what() << std::endl << std::flush;
						break;
				}
			}
			catch(std::exception & e)
			{
				std::cerr <<"CUIDADO!!!!" <<std::endl  << std::flush;

			}
			catch(int e)
			{
				std::cerr <<"CUIDADO intero!!!!" <<std::endl  << std::flush;

			}
		}

	}catch(RadioException& e)
	{
		switch(e.code)
		{
			case RADIO_RXLINEDOWN:
				std::cerr << "CUIDAO RX!" << std::endl << std::flush;
				break;
			case RADIO_TXLINEDOWN:
				std::cerr << "CUIDAO TX!" << std::endl << std::flush;
				break;
			default:
				std::cerr << "CUIDAO!" << std::endl << std::flush;
				break;
		}
		std::cerr << "Radio Exception: " << e.what() << std::endl << std::flush;
		exit(1);
	}



	debtDecParam_destroy(&d);
	debtEncParam_destroy(&e);


	return 0;

}




