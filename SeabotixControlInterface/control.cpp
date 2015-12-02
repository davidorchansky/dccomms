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
	int milis = 5000;//atoi(argv[3]);


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
				const char fw1[50] = "fw 1,2,3,4,5,6 50\n";
				const char fw2[50] = "fw 1,2,3,4,5,6 75\n";
				const char fw3[50] = "fw 1,2,3,4,5,6 100\n";
				
				const char rv1[50] = "rv 1,2,3,4,5,6 50\n";
				const char rv2[50] = "rv 1,2,3,4,5,6 75\n";
				const char rv3[50] = "rv 1,2,3,4,5,6 100\n";
				
				const char stop[50] = "sp 1,2,3,4,5,6\n";

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));

				std::cout << "Enviando orden de MARCHA hacia ADELANTE a los motores 1,2,3,4,5 y 6 al 50%..." << std::endl;
				radioTx.SendBytes(fw1, strlen(fw1), 255, frameSize, milis);

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de PARADA a los motores 1,2,3,4,5 y 6" << std::endl;
				radioTx.SendBytes(stop, strlen(stop), 255, frameSize, milis);

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de MARCHA hacia ATRAS a los motores 1,2,3,4,5 y 6 al 50%..." << std::endl;
				radioTx.SendBytes(rv1, strlen(rv1), 255, frameSize, milis);
				
				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de PARADA a los motores 1,2,3,4,5 y 6" << std::endl;
				radioTx.SendBytes(stop, strlen(stop), 255, frameSize, milis);

				//-----------------------------------

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de MARCHA hacia ADELANTE a los motores 1,2,3,4,5 y 6 al 75%..." << std::endl;
				radioTx.SendBytes(fw2, strlen(fw2), 255, frameSize, milis);

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de PARADA a los motores 1,2,3,4,5 y 6" << std::endl;
				radioTx.SendBytes(stop, strlen(stop), 255, frameSize, milis);

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de MARCHA hacia ATRAS a los motores 1,2,3,4,5 y 6 al 75%..." << std::endl;
				radioTx.SendBytes(rv2, strlen(rv2), 255, frameSize, milis);
				
				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de PARADA a los motores 1,2,3,4,5 y 6" << std::endl;
				radioTx.SendBytes(stop, strlen(stop), 255, frameSize, milis);


				//-----------------------------------

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de MARCHA hacia ADELANTE a los motores 1,2,3,4,5 y 6 al 100%..." << std::endl;
				radioTx.SendBytes(fw3, strlen(fw3), 255, frameSize, milis);

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de PARADA a los motores 1,2,3,4,5 y 6" << std::endl;
				radioTx.SendBytes(stop, strlen(stop), 255, frameSize, milis);

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de MARCHA hacia ATRAS a los motores 1,2,3,4,5 y 6 al 100%..." << std::endl;
				radioTx.SendBytes(rv3, strlen(rv3), 255, frameSize, milis);

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				std::cout << "Enviando orden de PARADA a los motores 1,2,3,4,5 y 6" << std::endl;
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




