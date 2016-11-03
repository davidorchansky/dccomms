/*
 * Radio.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: diego
 */


#include <Radio.h>
#include <Arduino.h>
#include <CommsException.h>
#include <DataLinkFrame.h>
#include <ICommsLink.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>
#include <iostream>

using namespace dccomms;

Radio::Radio(unsigned char d, ICommsLink & s, Radio::fcsType fcst, uint32_t maxRxBufferSize):serial(s),dir(d)
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
		std::cerr << "Configurado crc32" << std::endl;
#endif

			break;
		case Radio::fcsType::crc16:
			FCSType = DataLinkFrame::fcsType::crc16;
#ifdef DEBUG
		std::cerr << "Configurado crc16" << std::endl;
#endif
			break;
		case Radio::fcsType::nofcs:
			FCSType = DataLinkFrame::fcsType::nofcs;
#ifdef DEBUG
		std::cerr << "Configurado crc16" << std::endl;
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
bool Radio::BusyTransmitting()
{
	return serial.BusyTransmitting();
}

void Radio::SendBytes(const void * buf, uint32_t size, uint8_t dirTo, uint32_t packetSize, unsigned long ms)
{
	uint8_t * buffer = (uint8_t*) buf;
	uint32_t numPackets = size / packetSize;
	uint32_t np;
	for(np = 1 ; np < numPackets; np++)
	{
		while(serial.BusyTransmitting());

		DataLinkFramePtr dlfPtr = DataLinkFrame::BuildDataLinkFrame(dirTo, dir, packetSize, buffer, FCSType); //TODO: Deberiamos reservar memoria solo 1 vez para guardar una trama
#ifdef DEBUG
		std::cerr << "Enviando paquete..." << std::endl;
		dlfPtr->printFrame(std::cerr);
#endif
		serial << dlfPtr;
		buffer += packetSize;
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));


	}
	if(numPackets > 0)
	{
		while(serial.BusyTransmitting());
		DataLinkFramePtr dlfPtr = DataLinkFrame::BuildDataLinkFrame(dirTo, dir, packetSize, buffer, FCSType);
#ifdef DEBUG
		std::cerr << "Enviando paquete..." << std::endl;
		dlfPtr->printFrame(std::cerr);
#endif
		serial << dlfPtr;
		buffer += packetSize;
	}

	uint32_t bytesLeft = size % packetSize;
	if(bytesLeft)
	{
		while(serial.BusyTransmitting());
		if(numPackets > 0)
			std::this_thread::sleep_for(std::chrono::milliseconds(ms));
		DataLinkFramePtr dlfPtr = DataLinkFrame::BuildDataLinkFrame(dirTo, dir, bytesLeft, buffer, FCSType);
#ifdef DEBUG
		std::cerr << "Enviando paquete..." << std::endl;
		dlfPtr->printFrame(std::cerr);
#endif
		serial << dlfPtr;
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
	DataLinkFramePtr dlfPtr = DataLinkFrame::BuildDataLinkFrame(FCSType);
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

			serial >> dlfPtr;
	#ifdef DEBUG
			dlfPtr->printFrame(std::cerr);
			std::cerr << std::flush;
	#endif
			if(dlfPtr->checkFrame())
			{
	#ifdef DEBUG
				std::cerr << "Frame de radio correcto!" <<std::endl;
				std::cerr << std::flush;
	#endif
				uint16_t bytesToRead = (bytes + dlfPtr->dataSize) <= size ? dlfPtr->dataSize : size - bytes;

				for(i=0; i<bytesToRead; i++)
				{
					*buffer = dlfPtr->payload[i];
					buffer++;
				}
				bytes += dlfPtr->dataSize;
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
				*ptr = dlfPtr->payload[i++];
				ptr++;
			}
			_rxBufferLastPos = 0;
			_rxBufferFirstPos = 0;
		}

		serial.SetTimeout(currentTimeout);
	}
	catch(CommsException &e)
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
