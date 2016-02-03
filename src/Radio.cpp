/*
 * Radio.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: diego
 */


#include <Radio.h>
#include <Arduino.h>
#include <DataLinkFrame.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>
#include <iostream>
#include <RadioException.h>
using namespace radiotransmission;

Radio::Radio(unsigned char d, SerialPortInterface & s, Radio::fcsType fcst, uint32_t maxRxBufferSize):serial(s),dir(d)
{
	_maxRxBufferSize = maxRxBufferSize;
	_rxBuffer = new uint8_t[_maxRxBufferSize];
	_bytesInBuffer = 0;
	_rxBufferLastPos = 0;
	_rxBufferFirstPos = 0;
	TotalErrors = 0;

	switch(fcst)
	{
		case Radio::fcsType::crc32:
			FCSType = DataLinkFrame::fcsType::crc32;
#ifdef DEBUG
		std::cout << "Configurado crc32" << std::endl;
#endif

			break;
		case Radio::fcsType::crc16:
			FCSType = DataLinkFrame::fcsType::crc16;
#ifdef DEBUG
		std::cout << "Configurado crc16" << std::endl;
#endif
			break;
		case Radio::fcsType::nofcs:
			FCSType = DataLinkFrame::fcsType::nofcs;
#ifdef DEBUG
		std::cout << "Configurado crc16" << std::endl;
#endif
			break;
	}
	//FCSType = (DataLinkFrame::fcsType) fcst;
}

Radio::~Radio()
{
	if(_rxBuffer != NULL)
		delete _rxBuffer;
}

void Radio::SendBytes(const void * buf, uint32_t size, uint8_t dirTo, uint32_t packetSize, unsigned long ms)
{
	uint8_t * buffer = (uint8_t*) buf;
	uint32_t numPackets = size / packetSize;
	uint32_t np;
	for(np = 1 ; np < numPackets; np++)
	{
		DataLinkFrame dlf(dirTo, dir, packetSize, buffer, FCSType);
#ifdef DEBUG
		std::cout << "Enviando paquete..." << std::endl;
		dlf.printFrame(std::cout);
#endif
		serial << dlf;
		buffer += packetSize;
		serial.FlushIO(); //Lo ideal seria FlushOutput, pero en algun lado hay algo que hace que se llene el buffer de entrada
						   //y al final llega a bloquearse la comunicación... (TODO: comprobar qué es lo que hace que se llene el buffer de entrada)
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));

	}
	if(numPackets > 0)
	{

		DataLinkFrame dlf(dirTo, dir, packetSize, buffer, FCSType);
#ifdef DEBUG
		std::cout << "Enviando paquete..." << std::endl;
		dlf.printFrame(std::cout);
#endif
		serial << dlf;
		buffer += packetSize;
		serial.FlushIO(); //Lo ideal seria FlushOutput, pero en algun lado hay algo que hace que se llene el buffer de entrada
						   //y al final llega a bloquearse la comunicación... (TODO: comprobar qué es lo que hace que se llene el buffer de entrada)
	}

	uint32_t bytesLeft = size % packetSize;
	if(bytesLeft)
	{
		if(numPackets > 0)
			std::this_thread::sleep_for(std::chrono::milliseconds(ms));
		DataLinkFrame dlf(dirTo, dir, bytesLeft, buffer, FCSType);
#ifdef DEBUG
		std::cout << "Enviando paquete..." << std::endl;
		dlf.printFrame(std::cout);
#endif
		serial << dlf;
		serial.FlushIO();//Lo ideal seria FlushOutput, pero en algun lado hay algo que hace que se llene el buffer de entrada
		   	   	   	   	  //y al final llega a bloquearse la comunicación... (TODO: comprobar qué es lo que hace que se llene el buffer de entrada)
		//std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

}

void Radio::_IncreaseBytesInBuffer()
{
	if(_bytesInBuffer < _maxRxBufferSize)
	{
		_rxBufferLastPos = (_rxBufferLastPos + 1) % _maxRxBufferSize;
		_bytesInBuffer++;
	}
}

void Radio::_DecreaseBytesInBuffer()
{
	if(_bytesInBuffer)
	{
		_rxBufferFirstPos = (_rxBufferFirstPos + 1) % _maxRxBufferSize;
		_bytesInBuffer--;
	}

}

void Radio::ReceiveBytes(void * buf, uint32_t size, uint8_t dirFrom, unsigned long ms)
{
	uint8_t * buffer = (uint8_t *) buf;
	uint32_t bytes = 0;
	DataLinkFrame dlf(FCSType);
	uint16_t i;
	unsigned long currentTimeout = serial.GetTimeout();
	serial.SetTimeout(ms >= 0 ? ms : 0);

	if(_bytesInBuffer) //SI hay bytes en buffer, los leemos primero
	{
		while(bytes < size && _bytesInBuffer)
		{
			*buffer = _rxBuffer[_rxBufferFirstPos];
			_DecreaseBytesInBuffer();
			buffer++;
			bytes++;

		}

	}

	try
	{
		while(bytes < size) //Si faltan bytes, esperamos recibirlos
		{

			serial >> dlf;
	#ifdef DEBUG
			dlf.printFrame(std::cerr);
			std::cerr << std::flush;
	#endif
			if(dlf.checkFrame())
			{
	#ifdef DEBUG
				std::cout << "Frame de radio correcto!" <<std::endl;
				std::cout << std::flush;
	#endif
				uint16_t bytesToRead = (bytes + dlf.dataSize) <= size ? dlf.dataSize : size - bytes;

				for(i=0; i<bytesToRead; i++)
				{
					*buffer = dlf.payload[i];
					buffer++;
				}
				bytes += dlf.dataSize;
			}
			else
			{

				TotalErrors += 1;
				std::cerr << "Error en frame de radio (Total Errors: "<<TotalErrors<< ")" <<std::endl << std::flush;
	#ifdef DEBUG
			//	std::this_thread::sleep_for(std::chrono::milliseconds(50000));
	#endif
			}

		}
		if(bytes > size) //Si hemos recibido más de los que necesitamos, los guardamos en el buffer
		{
			uint32_t bytesLeft = bytes - size;
			uint8_t * ptr = _rxBuffer;
			_bytesInBuffer = bytesLeft <= _maxRxBufferSize ? bytesLeft : _maxRxBufferSize;
			uint8_t * maxPtr = _rxBuffer + _bytesInBuffer;
			while(ptr != maxPtr)
			{
				*ptr = dlf.payload[i++];
				ptr++;
			}
			_rxBufferLastPos = 0;
			_rxBufferFirstPos = 0;
		}

		serial.SetTimeout(currentTimeout);
	}
	catch(RadioException &e)
	{
		_rxBufferLastPos = 0;
		_rxBufferFirstPos = 0;
		serial.SetTimeout(currentTimeout);
		throw;
	}
	catch(std::exception &e)
	{
		std::cerr << "Excepcion no esperada" <<std::endl << std::flush;
		_rxBufferLastPos = 0;
		_rxBufferFirstPos = 0;
		serial.SetTimeout(currentTimeout);

	}
	catch(int &e)
	{
		std::cerr << "Excepcion no esperada" <<std::endl << std::flush;
		_rxBufferLastPos = 0;
		_rxBufferFirstPos = 0;
		serial.SetTimeout(currentTimeout);
	}


}
