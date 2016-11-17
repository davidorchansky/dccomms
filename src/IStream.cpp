/*
 * Stream.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <dccomms/IStream.h>
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

int IStream::ReadInt(int & num, char & nextByte)
{
	int n = 0; //number of [0-9] read
	char * ptr = buffer;

	Read(ptr, 1);
	nextByte = *ptr;
	if(*ptr == '-' || *ptr == '+')
	{
		ptr++;
		Read(ptr, 1);
		nextByte = *ptr;
	}
	while(*ptr >= '0' && *ptr <= '9')
	{
		ptr++;
		n++;
		Read(ptr, 1);
		nextByte = *ptr;
	}
	if(n)
	{
		num = atoi(buffer);
		return ptr-buffer; //Number of bytes read
	}
	return -1;
}

int IStream::ReadUInt(int & num, char & nextByte)
{
	int n = 0; //number of [0-9] read
	char * ptr = buffer;

	Read(ptr, 1);
	nextByte = *ptr;
	while(*ptr >= '0' && *ptr <= '9')
	{
		ptr++;
		n++;
		Read(ptr, 1);
		nextByte = *ptr;
	}
	if(n)
	{
		num = atoi(buffer);
		return ptr-buffer; //Number of bytes read
	}
	return -1;
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
