//============================================================================
// Name        : TX2.cpp
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
#include <cstring>
#include <unistd.h>

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

int main(int argc, char ** argv) {
	if(argc != 5)
	{
		std::cerr << "Numero de argumentos incorrecto" << std::endl;
		std::cout << "Usage:\n\targs: <blockIdentifier> <blockSize> <packetSize> <delayBetweenPackets>" << std::endl;
		exit(1);
	}



	int bIdLength = strlen(argv[1]);
	uint32_t blockSize = atoi(argv[2]);
	int frameSize = atoi(argv[3]);
	int milis = atoi(argv[4]);


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

		std::cout << "Block name: " << argv[1] << std::endl;
		std::cout << "Block size: " << blockSize << " bytes" << std::endl;

		struct timeval time0, time1;

		unsigned long long t0;
		unsigned long long t1;
		unsigned long long tdif;

		uint32_t bsize;

		while(1)
		{
			try
			{

				if(readId(argv[1],bIdLength)==0)
				{
					std::cout << "Bloque encontrado, esperando bytes..."  <<std::endl;

					buffer = new uint8_t[blockSize];
					int r = fread(buffer, 1, blockSize, stdin);

					std::cout << "Leidos: " << r << std::endl;

					std::cout << "ENVIANDO BLOQUE..." <<std::endl;

					fileTx.Send(argv[1], buffer, blockSize, 255, frameSize, milis);
					std::this_thread::sleep_for(std::chrono::milliseconds(milis));


					std::cout << "BLOQUE ENVIADO!" <<std::endl;
					std::cout << "------------------" <<std::endl;

					delete buffer;
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




