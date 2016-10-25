/*
 * Utils.cpp
 *
 *  Created on: 05/03/2015
 *      Author: diego
 */

#include <Utils.h>
#include <cstdint>
#include <thread>

namespace dccomms {

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

std::string Utils::BuildString(std::initializer_list<std::string> list )
{
	std::string res = "";
    for( auto elem : list )
    {
        res += elem;
    }
    return res;
}

void Utils::Sleep(int millis)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

void Utils::Debug(std::ostream & o, std::string & msg)
{
#ifdef DEBUG
	o << msg << std::endl;
#endif
}

} /* namespace radiotransmission */
