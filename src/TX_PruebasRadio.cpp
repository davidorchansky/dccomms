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
		Arduino arduTx = Arduino::FindArduino(Arduino::BAUD_115200,
				"Hello, are you TX?\n",
				"Yes, I'm TX");

		if(!arduTx.IsOpen())
			return;

		std::cout <<"TX listo\n";



		uint32_t size = 2000; //Tamaño del bloque a enviar
		uint8_t buffer[size];

		//"Radio" permite enviar secuencias de bytes
		Radio radioTx(0,arduTx);

		//"BlockRadioTransmitter" permite enviar secuencias de bytes con identificador
		BlockRadioTransmitter fileTx(radioTx);


		int milis = 500;
		while(1)
		{
			try
			{
				std::cout << "#### ENVIANDO... ####" <<std::endl;

				//Envío del bloque de tamaño "size" en paquetes de 500 bytes y un delay de 400 ms entre envío de cada paquete
				fileTx.Send("Imagen", buffer, size, 400, milis);

				std::this_thread::sleep_for(std::chrono::milliseconds(milis));
				/*Envío del bloque de tamaño "size" en un solo paquete.*/

				//fileTx.Send("Imagen", buffer, size);
				std::cout << "#### ENVIADO... ####" <<std::endl;

				//arduTx.FlushIO();

			}
			catch(RadioException& e) //Control de excepciones
			{
				//std::cout << "Radio Exception: " << e.what() << std::endl << std::flush;
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




