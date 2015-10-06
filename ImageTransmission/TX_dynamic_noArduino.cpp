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
#include <videoServer.h>

using namespace std;
using namespace radiotransmission;


#define VIDEO_SERVER_RX 5

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


int main(int argc, char ** argv) {
	if(argc != 6)
	{
		std::cerr << "Numero de argumentos incorrecto" << std::endl;
		std::cout << "Usage:\n\targs: <blockIdentifier> <blockSize> <packetSize> <delayBetweenPackets> <MaxAgeOfTheFrameToSendInMillis>" << std::endl;
		exit(1);
	}


	videoTransmissionConfig config;

	int bIdLength = strlen(argv[1]);
	
	config.frameSize = atoi(argv[2]);
	config.maxPacketLength = atoi(argv[3]);
	config.delayBetweenPackets = atoi(argv[4]);
	config.maxFrameAge = atoi(argv[5]);


	unsigned char * buffer;

	try
	{
		SerialPortInterface::PortSettings ps;
		ps.baudrate = SerialPortInterface::BaudRate::BAUD_19200;
		ps.parity = SerialPortInterface::NOPARITY;
		ps.stopBits = SerialPortInterface::SB1;
		ps.dataBits = SerialPortInterface::CHAR8;

		SerialPortInterface sp("/dev/ttyO5", ps);
		sp.Open();

		if(!sp.IsOpen())
		{
			std::cerr << "No ha sido posible encontrar la arduino" << std::endl;
			exit(4);
		}

		std::cout <<"TX listo\n";

		Radio radioTx(0,sp);

		BlockRadioTransmitter fileTx(radioTx);

		std::cout << "Block name: " << argv[1] << std::endl;
		std::cout << "Block size: " << config.frameSize << " bytes" << std::endl;

		struct timeval time0, time1;

		unsigned long long t0;
		unsigned long long t1;
		unsigned long long tdif;

		uint32_t bsize;

		struct timeval start, stop;

		uint32_t age;

		fd_set fds;
		char c;
		char msg[100];
		int n;

		while(1)
		{
			FD_ZERO(&fds);
			FD_SET(VIDEO_SERVER_RX, &fds);
			struct timeval tv = {0};
			tv.tv_sec = 0;
			int r = select(VIDEO_SERVER_RX+1, &fds, NULL, NULL, &tv);

			if(r == 1)
			{
				read(VIDEO_SERVER_RX,&c,1);
				if( c == '{')
				{
					if(getVideoTransmissionConfig(VIDEO_SERVER_RX, &config)==-1)continue;
					n = buildVideoTransmissionConfigMsg(msg,&config);
					fprintf(stdout, "TRANSMISOR: Recibida configuración de video (%d): %s\n",n,msg);
				}

			}
	
			try
			{

				if(readId(argv[1],bIdLength)==0)
				{
					std::cout << "Bloque encontrado, comprobando antigüedad..." << std::endl;
					fread(&start, 1, sizeof(struct timeval), stdin);

					gettimeofday(&stop, NULL);

					unsigned int t1 = getMilliseconds(&stop);
					unsigned int t0 = getMilliseconds(&start);
					
					std::cout << "T0: "<<t0 <<std::endl;
					std::cout << "T1: " <<t1 <<std::endl;
					age = t1-t0;

					std::cout << "Age: " << age << std::endl;
					std::cout << "MaxFrameAge: " << config.maxFrameAge << std::endl;
					if(age <= config.maxFrameAge)
					{
						std::cout << "Esperando bytes del bloque..."  <<std::endl;

						buffer = new uint8_t[config.frameSize];
						int r = fread(buffer, 1, config.frameSize, stdin);

						std::cout << "Leidos: " << r << std::endl;

						std::cout << "ENVIANDO BLOQUE..." <<std::endl;

						fileTx.Send(argv[1], buffer, config.frameSize, 255, config.maxPacketLength, config.delayBetweenPackets);
						std::this_thread::sleep_for(std::chrono::milliseconds(config.delayBetweenPackets));


						std::cout << "BLOQUE ENVIADO!" <<std::endl;
						std::cout << "------------------" <<std::endl;

						delete buffer;
					}
				}

			}
			catch(RadioException& e) //Control de excepciones
			{

				switch(e.code)
				{
					case RADIO_TXLINEDOWN: //Se ha perdido la comunicación con la arduino transmisora
						std::cout << "CONEXION PERDIDA CON EL TRANSMISOR!" << std::endl << std::flush;
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




