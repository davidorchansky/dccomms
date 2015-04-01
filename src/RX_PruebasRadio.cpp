//============================================================================
// Name        : PruebasRadio.cpp
// Author      : Diego
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
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

using namespace std;
using namespace radiotransmission;



void test()
{
	try
	{
		Arduino arduRx = Arduino::FindArduino(Arduino::BAUD_115200,
				"Hello, are you RX?\n",
				"Yes, I'm RX");

		if(!arduRx.IsOpen())
			return;

		std::cout <<"RX listo\n";

		uint32_t size = 5000;
		uint8_t rxbuffer[size];

		Radio radioRx(1,arduRx);


		BlockRadioTransmitter fileRx(radioRx);

		while(1)
		{
			try
			{
				std::cout << "#### RECIBIENDO... ####" << std::endl;
				//arduRx.FlushIO();
				/*Recepción del bloque en modo bloqueante*/
				fileRx.Receive("Imagen", rxbuffer, 5000);
				//arduRx.FlushIO();

				/*Recepción del bloque en modo "Timeout", establecido a 10 segundos.
				 *Nota: el timeout indica el tiempo máximo que se debe esperar para la re
				 * cepción del siguiente byte. En caso de transcurrir el tiempo indicado,
				 * se producirá una RadioException de tipo RADIO_TIMEOUT.

				fileRx.Receive("Imagen", rxbuffer, 10000);

				*/

				std::cout << "#### RECIBIDO! ####" << std::endl <<std::endl;


			}
			catch(RadioException& e) //Control de excepciones
			{
				//std::cout << "Radio Exception: " << e.what() << std::endl << std::flush;
				switch(e.code)
				{
					case RADIO_RXLINEDOWN: //Se ha perdido la comunicación con la arduino receptora
						std::cout << "Intentando reconectar con RX..." << std::endl << std::flush;
						while(!arduRx.TryReconnect()){};
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
				std::cout <<"CUIDAO!!!!" <<std::endl  << std::flush;

			}
			catch(int e)
			{
				std::cout <<"CUIDAO intero!!!!" <<std::endl  << std::flush;

			}
		}

	}catch(RadioException& e)
	{
		switch(e.code)
		{
			case RADIO_RXLINEDOWN:
				std::cout << "CUIDAO RX!" << std::endl << std::flush;
				break;
			case RADIO_TXLINEDOWN:
				std::cout << "CUIDAO TX!" << std::endl << std::flush;
				break;
			default:
				std::cout << "CUIDAO!" << std::endl << std::flush;
				break;
		}
		std::cout << "Radio Exception: " << e.what() << std::endl << std::flush;
		exit(1);
	}




}
int main() {
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	test();
	return 0;
}




