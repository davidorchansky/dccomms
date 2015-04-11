//============================================================================
// Name        : RX.cpp
// Author      : Diego
// Version     :
// Copyright   :
// Description : Receives one image per bucle iteration
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
#include <sys/stat.h>

using namespace std;
using namespace radiotransmission;

int main(int argc, char **argv) {
	if(argc != 3)
	{
		std::cerr << "Numero de argumentos incorrecto" << std::endl;
		std::cerr << "Usage:\n\targs: <blockIdentifier> <maxBlockSize>" << std::endl;
		exit(1);
	}

	uint32_t size = atoi(argv[2]);

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

		uint8_t rxbuffer[size];

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

		while(1)
		{
			try
			{
				std::cerr << "BUSCANDO BLOQUE..." << std::endl;

				arduRx.FlushInput();

				gettimeofday(&time0, NULL);
				t0 = time0.tv_sec*1000 + time0.tv_usec/1000;
				

				//Recepción en modo bloqueante (Se bloquea hasta recibir un bloque válido)
				fsize = fileRx.Receive(argv[1], rxbuffer);


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
				std::cerr << "Enviando bloque por la salida estandar..." << std::endl <<std::endl;

				gettimeofday(&time0, NULL);
				t0 = time0.tv_sec*1000 + time0.tv_usec/1000;

				fwrite(rxbuffer, 1, fsize, stdout);

				gettimeofday(&time1, NULL);
				t1 = time1.tv_sec*1000 + time1.tv_usec/1000;
				tdif = t1 - t0;
				std::cerr << "Bloque enviado en : " << tdif << " ms" <<std::endl;

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




	return 0;
}




