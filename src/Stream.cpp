/*
 * Stream.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <Stream.h>
#include <iostream>
namespace dccomms {

Stream::Stream() {
	// TODO Auto-generated constructor stub

}

Stream::~Stream() {
	// TODO Auto-generated destructor stub
}

void Stream::WaitFor(const uint8_t * expected, uint32_t size)
{
	const uint8_t * ptr = expected;
	const uint8_t * max = expected + size;
	uint8_t c;
	while(ptr != max)
	{
		this->Read(&c, 1);
		if(c == *ptr)
			ptr++;
		else
			ptr = expected;
	}
}

/*
int Stream::Read(uint8_t * buf, uint32_t size, uint32_t to)
{


}

int Stream::Write(const uint8_t * buf, uint32_t size, uint32_t to)
{


}
int Stream::Available()
{

}
Stream & Stream::operator >> (uint8_t &byte)
{

}

Stream & Stream::operator << (uint8_t byte)
{

}
Stream & Stream::operator << (const char * stt)
{

}
*/
} /* namespace radiotransmission */
