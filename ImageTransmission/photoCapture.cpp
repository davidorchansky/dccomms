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
			"\t-I               - Specify an ASCII identifier for the image (it is like a header)\n"
			"\t-W               - Specify the width of the image\n"
			"\t-H               - Specify the height of the image\n"
			"\t-F		    - Encoded frame size\n"
			"\t-L		    - Max. packet length\n"
			"\t-D		    - Delay between packets\n"
			"\t-A		    - Max. frame Age\n",

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
		case 'W':
			width_G = atoi(optarg);
			break;
		case 'H':
			height_G = atoi(optarg);
			break;
		case 'F':
			config->frameSize = atoi(optarg);
			break;
		case 'L':
			config->maxPacketLength = atoi(optarg);
			break;
		case 'D':
			config->delayBetweenPackets = atoi(optarg);
			break;
		case 'A':
			config->maxFrameAge = atoi(optarg);
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
	

	//////////// GRABBER AND ENCODER SETUP
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction = segfault_sigaction;
	sa.sa_flags   = SA_SIGINFO;

	sigaction(SIGSEGV, &sa, NULL);

	const char * v4linuxDevice = "/dev/video0";
	int fd;

	fd = open(v4linuxDevice, O_RDWR | O_NONBLOCK);
	if (fd == -1)
	{
		perror("Opening video device");
		return 1;
	}

	if(print_caps(fd))
		return 1;

	/*
	if(init_mmap(fd))
		return 1;
	*/

	struct imgBuffer *img = NULL;
	/* prepare an image buffer for the input image */
	if ((img = imgBuffer_aligned_new(1))) {
		if (!imgBuffer_reinit(img, width_G, height_G, COLORFORMAT_FMT_420)) 
			return 1;
	}
	else return 1;

	long yuyv422_size = width_G * height_G * 2;
	long inputBuffer_size = yuyv422_size; //El driver para raspicam captura 3 imagenes de golpe. Nosotros enviaremos la ultima

	long yuv420p_size = width_G*height_G + width_G*height_G/2;

	uint8_t * y = img->buffer;
	uint8_t * u = y + width_G * height_G;
	uint8_t * v = u + width_G * height_G / 4;

	/*
	if(start_capturing(fd))
		return 1;
	*/
	uint8_t * inputBuffer = (uint8_t *) malloc(inputBuffer_size);
	fd_set fds;

	/* setup the output buffer */
	unsigned char *buffer = NULL;
	int buflen = 0;
	int fixed = 0;
	if (e.maxsize) {
		buffer = (unsigned char *) malloc(buflen = e.maxsize);
		fixed = 1;
	}

	/////////// END - GRABBER AND ENCODER SETUP	

	try
	{
		Arduino arduTx = Arduino::FindArduino(Arduino::BAUD_115200,
				"Hello, are you TX?\n",
				"Yes, I'm TX");

		if(!arduTx.IsOpen())
		{
			std::cerr << "No ha sido posible encontrar la arduino" << std::endl;
			exit(4);
		}

		std::cout <<"TX listo\n";

		Radio radioTx(0,arduTx);

		BlockRadioTransmitter fileTx(radioTx);

		std::cout << "Block name: " << imId << std::endl;
		std::cout << "Block size: " << config.frameSize << " bytes" << std::endl;

		struct timeval time0, time1;

		unsigned long long t0;
		unsigned long long t1;
		unsigned long long tdif;

		uint32_t bsize;

		struct timeval start, stop;

		uint32_t age;

		fd_set fds;

		unsigned long imagenesEnviadas = 0;

		int newestFrame = 0;
		bool capturar = false;
		char fileName[50];
		char rxbuffer[200];

		while(1)
		{
			try
			{
		
				if(capturar)
				{
					capturar = false;
					/////////CAPTURA
					FD_ZERO(&fds);
					FD_SET(fd, &fds);
					struct timeval tv = {0};
					tv.tv_sec = 2;
					int r = select(fd+1, &fds, NULL, NULL, NULL);

					int frameReady = 1;	

					if(-1 == r)
					{
						perror("Waiting for Frame");
						frameReady = 0;
					}

					
					if(frameReady)
					{
						fprintf(stderr, "GRABBER: capturando imagen...\n");
						if(read(fd,inputBuffer,yuyv422_size)>0)
						{
							int n, cont = 0;
							do //Limpiamos frames antiguos...
							{
								n = read(fd, inputBuffer, yuyv422_size);
								printf("N: %d\n", n);
							}
							while(n == yuyv422_size);
							if(newestFrame == 2)  
							{
								newestFrame = 0;
								/////////COMPRESION
								fprintf(stderr, "GRABBER: Convirtiendo imagen de yuyv422 a yuv420p\n");
								yuyv422_to_yuv420p(width_G, height_G, inputBuffer, y, u, v);

								int res = encode(&e, &buffer, buflen, fixed, imId, bIdLength, img);
								/////////ENVIO
								if(res)
								{

									std::cout << "ENVIANDO BLOQUE..." <<std::endl;

									fileTx.Send(imId, buffer, config.frameSize, 255, config.maxPacketLength, config.delayBetweenPackets);
									std::this_thread::sleep_for(std::chrono::milliseconds(config.delayBetweenPackets));


									std::cout << "BLOQUE ENVIADO!" <<std::endl;
									std::cout << "------------------" <<std::endl;
									imagenesEnviadas++;
									std::cout << "Imagenes enviadas: "<< imagenesEnviadas << std::endl;

								}

							}
							else newestFrame++;
						}
					}
				}
				else
				{
	
					fsize = fileTx.Receive(imId, rxbuffer);
					memcpy(fileName, rxbuffer, fsize);
					capturar = true;
				}
			}
			catch(RadioException& e) //Control de excepciones
			{

				switch(e.code)
				{
					case RADIO_TXLINEDOWN: //Se ha perdido la comunicación con la arduino transmisora
						std::cout << "Intentando reconectar con TX..." << std::endl << std::flush;
						while(!arduTx.TryReconnect()){};
						std::cout << "Éxito!" << std::endl << std::flush;
						break;

					case RADIO_TIMEOUT:
						std::cout << "TIMEOUT!" << std::endl << std::flush;
						break;
					default:
						std::cout << "CUIDAO!" << std::endl << std::flush;
						break;
				}
			}
			catch(std::exception & e)
			{
				std::cout <<"CUIDADO!" <<std::endl  << std::flush;

			}
			catch(int e)
			{
				std::cout <<"CUIDADO!" <<std::endl  << std::flush;

			}
		}

	}catch(RadioException& e)
	{
		switch(e.code)
		{
			case RADIO_TXLINEDOWN:
				std::cout << "CUIDAO TX!" << std::endl << std::flush;
				break;
			default:
				std::cout << "CUIDADO!" << std::endl << std::flush;
				break;
		}
		std::cout << "Radio Exception: " << e.what() << std::endl << std::flush;
		exit(1);
	}


	return 0;
}




