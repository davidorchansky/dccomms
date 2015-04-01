/*
 * BlockRadioTransmitter.cpp
 *
 *  Created on: 01/03/2015
 *      Author: diego
 */

#include <BlockRadioTransmitter.h>
#include <Radio.h>
#include <cstring>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>
#include <Utils.h>
#include <openssl/md5.h>
#include <iostream>
#include <RadioException.h>

#include <iostream>
using namespace radiotransmission;

BlockRadioTransmitter::BlockRadioTransmitter(Radio & rad):radio(rad) {
	_bigEndian = Utils::IsBigEndian();

}

BlockRadioTransmitter::~BlockRadioTransmitter() {

}


void BlockRadioTransmitter::Send(const char* identifier, void * buf, uint32_t size, uint8_t dest, uint32_t ps, unsigned long ms)
{
	uint32_t idSize = strlen(identifier);
	uint16_t infoSize = idSize+4;
    uint32_t totalSize = infoSize+size+16;

	uint8_t * buffer = new uint8_t[totalSize];

	uint8_t * id = buffer;
	uint8_t * dsize = id + idSize;
	uint8_t * data = dsize + 4;
	uint8_t * md5 = data + size;

	memcpy(id, identifier, idSize);

    if(_bigEndian)
    	*((uint32_t *)dsize) = size;
    else
    	Utils::IntSwitchEndian(dsize, size);

	memcpy(data, buf, size);

#ifndef CHCK_BLOCKINTEGRITY
	MD5((unsigned char*)data,size,md5);
#endif
	uint32_t packetSize = ps > 0 ? ps : totalSize;

	radio.SendBytes(buffer, totalSize , dest, packetSize, ms);

}


uint32_t BlockRadioTransmitter::Receive(const char* identifier, void * buf, unsigned long ms)
{
	uint16_t idSize = strlen(identifier);
	uint8_t * buffer = (uint8_t *) buf;
	char c;
	const uint8_t * idPtr = (const uint8_t *) identifier;
	const uint8_t * idMaxPtr = ((const uint8_t *) identifier)+idSize;
	while(idPtr != idMaxPtr)
	{
		radio.ReceiveBytes(&c, 1, 255, ms);
		if(c == *idPtr)
			idPtr++;
		else
			idPtr = (const uint8_t *) identifier;

	}
#ifdef TIMMING
	struct timeval time0, time1;

	unsigned long long t0;
	unsigned long long t1;
	unsigned long long tdif;
	gettimeofday(&time0, NULL);
	t0 = time0.tv_sec*1000 + time0.tv_usec/1000;

#endif
	uint32_t s, size;
	uint8_t md5[16]; //MD5
	uint8_t rmd5[16];

	radio.ReceiveBytes(&s, 4, 255, ms);
	if(_bigEndian)
		size = s;
	else
		Utils::IntSwitchEndian(&size, s);

	radio.ReceiveBytes(buffer, size, 255, ms);
	radio.ReceiveBytes(rmd5, 16, 255, ms);
	MD5((unsigned char*)buffer,size,md5);

#ifdef DEBUG
	if(memcmp(md5, rmd5, 16)==0)
		std::cout << "Integridad del bloque total" << std::endl;
	else
	{
		std::cout << "Bloque no válido" << std::endl;
		throw 88;
	}

#else
	if(memcmp(md5, rmd5, 16)!=0)
	{
		throw RadioException("Bloque no válido", RADIO_CORRUPTBLOCK);
	}
#endif
#ifdef TIMMING
	gettimeofday(&time1, NULL);
	t1 = time1.tv_sec*1000 + time1.tv_usec/1000;
	tdif = t1 - t0;
	std::cout << "Imagen recibida en: "<< tdif << " ms" <<std::endl;
#endif

	return size;

}
