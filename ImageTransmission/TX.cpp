//============================================================================
// Name        : TX.cpp
// Author      : Diego
// Version     :
// Copyright   :
// Description : Sends an image per bucle iteration
//============================================================================

#include <iostream>
#include <Arduino.h>
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

using namespace std;
using namespace radiotransmission;

int main(int argc, char ** argv) {
	if(argc != 4)
	{
		std::cerr << "Numero de argumentos incorrecto" << std::endl;
		std::cout << "Usage:\n\targs: <imageFile> <packetSize> <delayBetweenPackets>" << std::endl;
		exit(1);
	}

	struct stat finfo;


	int frameSize = atoi(argv[2]);
	int milis = atoi(argv[3]);


	unsigned char * buffer;

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

		std::cout << "File: " << argv[1] << std::endl;

		struct timeval time0, time1;

		unsigned long long t0;
		unsigned long long t1;
		unsigned long long tdif;

		while(1)
		{
			try
			{

				if(lstat(argv[1], &finfo) < 0)
				{
					std::cerr << "Error al obtener los metadatos del fichero " << argv[1] << std::endl;
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
					throw 1;
				}

				FILE * file = fopen(argv[1], "rb");

				if(file == NULL)
				{
					std::cerr << "Error al abrir el fichero " << argv[1] << std::endl;
					exit(3);
				}

				std::cout << "File size: " << finfo.st_size << " bytes" << std::endl;

				gettimeofday(&time0, NULL);
				t0 = time0.tv_sec*1000 + time0.tv_usec/1000;

				buffer = new uint8_t[finfo.st_size];
				fread(buffer, 1, finfo.st_size, file);
				fclose(file);

				gettimeofday(&time1, NULL);
				t1 = time1.tv_sec*1000 + time1.tv_usec/1000;
				tdif = t1 - t0;
				std::cout << "Imagen cargada en: " << tdif << " ms" <<std::endl;

				std::cout << "------------------" <<std::endl;
				std::cout << "ENVIANDO BLOQUE..." <<std::endl;

				gettimeofday(&time0, NULL);
				t0 = time0.tv_sec*1000 + time0.tv_usec/1000;

				fileTx.Send("Imagen", buffer, finfo.st_size, 255, frameSize, milis);
				std::this_thread::sleep_for(std::chrono::milliseconds(milis));

				gettimeofday(&time1, NULL);
				t1 = time1.tv_sec*1000 + time1.tv_usec/1000;
				std::cout << "Imagen enviada en: " << tdif << " ms" <<std::endl;

				std::cout << "BLOQUE ENVIADO!" <<std::endl;
				std::cout << "------------------" <<std::endl;

				delete buffer;

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




