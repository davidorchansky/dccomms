/*
 * DataLinkStream.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <DataLinkStream.h>


#include <string>

#include <stdio.h>   /* Standard input/output definitions */
#include <cstring>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <sys/time.h> /*para timeout*/
#include <sys/ioctl.h>
#include <RadioException.h>

#include <iostream>

namespace radiotransmission {

DataLinkStream::DataLinkStream(){}
DataLinkStream::DataLinkStream(const char * p)
{
	int s = strlen(p);
	port = new char[s+1];
	strcpy(port, p);
}
DataLinkStream::DataLinkStream(DataLinkStream::PortSettings ps)
{
	int s = ps.file.size();
	port = new char[s+1];
	strcpy(port, ps.file.c_str());
	portSettings = ps;
}


bool DataLinkStream::Open()
{
	struct termios options;

	fd = open(port, O_RDWR );
	if (fd != -1)
	{
		SetBufferSize(DLS_INBUFFER_SIZE);
		bufferSize = GetBufferSize();
		return true;
	}
	_open = false;
	return false;

}

void DataLinkStream::Close()
{

	close(fd);
	_open = false;
}



int DataLinkStream::Read(void * buf, uint32_t size, unsigned long ms)
{
	struct timeval time0, time1;
	gettimeofday(&time0, NULL);
	unsigned long long t0 = time0.tv_sec*1000 + time0.tv_usec/1000;
	unsigned long long t1 = t0;

	unsigned char* ptr = (uint8_t*) buf;
	unsigned char* max = (uint8_t*) buf + size;

	int n = 0;
	int bytesLeft = size - n;

	unsigned long m = ms ? ms : _timeout;

	if (m == 0)
	{
		//Bloqueado hasta coger m bytes
		while(true)
		{
#ifndef SERIAL_DISCONNECT_TEST
			if(Available()>0)
			{
				n += read(fd, ptr, bytesLeft);
				ptr = (uint8_t*)buf + n;
				if(ptr == max)
					return n; // == size
				bytesLeft = size - n;
			}
#else
			int res = Available();
			if(res>0)
			{
				n += read(fd, ptr, bytesLeft);
				ptr = (uint8_t*)buf + n;
				if(ptr == max)
					return n; // == size
				bytesLeft = size - n;
			}
			else
			{
				char sig = '-'; //Un byte aleatorio...
				res = write(fd, &sig, 1);
				if(res < 0)
				{
					close(fd);
					throw RadioException("Fallo de comunicacion al leer", RADIO_RXLINEDOWN);
				}
			}
#endif
		}

	}

#ifndef CHECK_TIMEOUTEXCEPTION
	while(t1 - t0 < m)
	{
		if(Available()>0)
		{

#ifdef PRINTSERIAL
			int SZ = Available();
			std::cout << std::endl << SZ <<std::endl;
#endif
			n += read(fd, ptr, bytesLeft);
			ptr = (uint8_t*)buf + n;
			if(ptr == max)
				return n; // == size
			bytesLeft = size - n;
		}
		gettimeofday(&time1, NULL);
		t1 = time1.tv_sec*1000 + time1.tv_usec/1000;
	}
#endif

	//Si se llega hasta este punto, es que ha transcurrido el timeout
	char sig = '-'; //Un byte aleatorio...
	int res = write(fd, &sig, 1);
	if(res < 0)
	{
		close(fd);
		throw RadioException("Fallo de comunicacion al leer", RADIO_RXLINEDOWN);
	}

	throw RadioException("Read Timeout", RADIO_TIMEOUT);

}

void DataLinkStream::SetTimeout(unsigned long ms)
{
	_timeout = ms >= 0 ? ms : 0;
	if (ms)
		fcntl(fd, F_SETFL, FNDELAY);
	else
		fcntl(fd, F_SETFL, 0);
}

int DataLinkStream::GetBufferSize()
{
	return fcntl(fd, F_GETPIPE_SZ);
}

void DataLinkStream::SetBufferSize(int bs)
{
	fcntl(fd, F_SETPIPE_SZ, bs);
	bufferSize = GetBufferSize();
}

void DataLinkStream::FlushInput()
{
	int n;
	n = read(fd, tmp, DLS_INBUFFER_SIZE_FLUSH);

	std::cerr << "N: " << n << " Buff. Size: " << bufferSize << std::endl;

	/*
	int flags=fcntl(fd, F_GETFL,0);
	int flags2 = flags;
	flags2 |= O_NONBLOCK;
	fcntl(fd, F_SETFL, flags2);
	*/

	//while((n = read(fd, tmp, DLS_INBUFFER_SIZE_FLUSH))>0)
	//{
	//	std::cerr << "N: " << n << " Buff. Size: " << bufferSize << std::endl;
	//}
	//std::cerr << "N: " << n << " Buff. Size: " << bufferSize << std::endl;
	//fcntl(fd, F_SETFL, flags); /* set the new flagts */


}

void DataLinkStream::FlushIO()
{
	//tcflush(fd, TCIOFLUSH);
}

void DataLinkStream::FlushOutput()
{
	//tcflush(fd, TCOFLUSH);
}

int DataLinkStream::Write(const void * buf, uint32_t size, uint32_t to)
{
	int w = write(fd, (const uint8_t*)buf, size);
	if(w < 0)
	{
		close(fd);
		throw RadioException("Fallo de comunicacion al escribir", RADIO_TXLINEDOWN);
	}

	return w;

}

int DataLinkStream::Available()
{
	int n;
	if(ioctl(fd, FIONREAD, &n)<0)
		return -1;
	return n;
}

bool DataLinkStream::IsOpen()
{
	return _open;
}

Stream & DataLinkStream::operator >> (uint8_t & byte )
{
	read(fd, &byte, sizeof(uint8_t));
	return *this;
}

Stream & DataLinkStream::operator >> (char & byte )
{
	read(fd, &byte, sizeof(uint8_t));
	return *this;
}

Stream & DataLinkStream::operator >> (uint16_t & data16 )
{
	read(fd, &data16, sizeof(uint16_t));
	return *this;
}

Stream & DataLinkStream::operator >> (uint32_t & data32 )
{
	read(fd, &data32, sizeof(uint32_t));
	return *this;
}

Stream & DataLinkStream::operator << (uint8_t byte)
{
	write(fd, &byte, sizeof(uint8_t));
	return *this;
}

Stream & DataLinkStream::operator << (const char * str)
{
	int n = strlen(str);
	write(fd, str, n);
	return *this;
}
} /* namespace radiotransmission */
