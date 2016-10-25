/*
 * Stream.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <IStream.h>
#include <iostream>
namespace dccomms {

IStream::IStream() {
	// TODO Auto-generated constructor stub

}

IStream::~IStream() {
	// TODO Auto-generated destructor stub
}

void IStream::WaitFor(const uint8_t * expected, uint32_t size)
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
*/
IStream & IStream::operator << (uint8_t byte)
{
	Write(&byte, sizeof(uint8_t));
	return *this;
}

IStream & IStream::operator << (const char * str)
{
	int n = sizeof(str);
	Write(str, n);
	return *this;
}

} /* namespace radiotransmission */
