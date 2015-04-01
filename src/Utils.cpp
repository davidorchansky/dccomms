/*
 * Utils.cpp
 *
 *  Created on: 05/03/2015
 *      Author: diego
 */

#include <Utils.h>
#include <cstdint>

namespace radiotransmission {

Utils::Utils() {
	// TODO Auto-generated constructor stub

}

Utils::~Utils() {
	// TODO Auto-generated destructor stub
}

bool Utils::IsBigEndian()
{
	uint32_t word = 0x1;
	uint8_t * byte = (uint8_t *)&word;
	return *byte != 0x1;
}

void Utils::IntSwitchEndian(void * b, uint32_t entero)
{
	uint8_t * buf = (uint8_t*) b;
	*buf = (uint8_t)((entero >> 24) & 0xff);
	*(buf+1) = (uint8_t)((entero >> 16) & 0xff);
	*(buf+2) = (uint8_t)((entero >> 8) & 0xff);
	*(buf+3) = (uint8_t)(entero & 0xff);
}

} /* namespace radiotransmission */