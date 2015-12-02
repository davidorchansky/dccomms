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
#include <cstring>

using namespace std;
using namespace radiotransmission;

int main(int argc, char ** argv) {
	int frameSize = 100;//atoi(argv[2]);
	int milis = 500;//atoi(argv[3]);


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

		Radio radioTx(0,arduTx, Radio::fcsType::crc16);

		struct timeval time0, time1;

		unsigned long long t0;
		unsigned long long t1;
		unsigned long long tdif;

		while(1)
		{
			try
			{
				const char fw[50] = "fw 1,2,3,4,5 75\n";
				const char rv[50] = "rv 1,2,3,4,5 75\n";
				const char stop[50] = "stop 1,2,3,4,5\n";

				radioTx.SendBytes(fw, strlen(fw), 255, frameSize, milis);
				radioTx.SendBytes(rv, strlen(rv), 255, frameSize, milis);
				radioTx.SendBytes(stop, strlen(stop), 255, frameSize, milis);

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




