/*
 * TCPStream.cpp
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#include <TCPStream.h>
#include <CommsException.h>
#include <boost/algorithm/string.hpp>
#include <sys/ioctl.h>
#include <sys/time.h> /*para timeout*/

namespace dccomms {

TCPStream::TCPStream(std::string address) {
	// TODO Auto-generated constructor stub
	std::vector<std::string> strs;
	boost::split(strs, address, boost::is_any_of(":"));
	portno = std::stoi(strs[1]);
	ip = strs[0];
}

TCPStream::~TCPStream() {
	// TODO Auto-generated destructor stub
	CloseConnection();
}
void TCPStream::CloseConnection()
{
	close(sockfd);
}

void TCPStream::OpenConnection()
{
	device = gethostbyname(ip.c_str());
	if (device == NULL)
	{
	  throw CommsException("TCP ERROR: No such host", PHYLAYER_ERROR);
	}
	bzero((char *) &device_addr, sizeof(device_addr));
	device_addr.sin_family = AF_INET;
	bcopy((char *)device->h_addr,
	      (char *)&device_addr.sin_addr.s_addr,
	      device->h_length);
	device_addr.sin_port = htons(portno);

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0)
		throw CommsException("TCP ERROR: Creating a TCP socket", PHYLAYER_ERROR);

	if (connect(sockfd,(struct sockaddr *) &device_addr,sizeof(device_addr)) < 0)
		throw CommsException("TCP ERROR: Connection to device", PHYLAYER_ERROR);
}


void TCPStream::Close()
{
	CloseConnection();
}

bool TCPStream::Open()
{
	OpenConnection();
}

int TCPStream::Write(const void * buf, uint32_t size, uint32_t to)
{
	int w = write(sockfd, (const uint8_t*)buf, size);
	if(w < 0)
	{
		close(sockfd);
		throw CommsException("Fallo de comunicacion al escribir", TXLINEDOWN);
	}
	return w;
}

int TCPStream::Read(void * buf, uint32_t size, unsigned long ms)
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
				n += read(sockfd, ptr, bytesLeft);
				ptr = (uint8_t*)buf + n;
				if(ptr == max)
					return n; // == size
				bytesLeft = size - n;
			}
#else
			int res = Available();
			if(res>0)
			{
				n += read(sockfd, ptr, bytesLeft);
				ptr = (uint8_t*)buf + n;
				if(ptr == max)
					return n; // == size
				bytesLeft = size - n;
			}
			else
			{
				char sig = '-'; //Un byte aleatorio...
				res = write(sockfd, &sig, 1);
				if(res < 0)
				{
					close(fd);
					throw CommsException("Fallo de comunicacion al leer", RXLINEDOWN);
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
			n += read(sockfd, ptr, bytesLeft);
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
	int res = write(sockfd, &sig, 1);
	if(res < 0)
	{
		close(sockfd);
		throw CommsException("Fallo de comunicacion al leer", RXLINEDOWN);
	}

	throw CommsException("Read Timeout", TIMEOUT);

}

int TCPStream::Available()
{
	int n;
	if(ioctl(sockfd, FIONREAD, &n)<0)
		return -1;
	return n;
}

void TCPStream::FlushInput()
{
	throw CommsException("Not implemented", NOTIMPLEMENTED);
}

void TCPStream::FlushIO()
{
	throw CommsException("Not implemented", NOTIMPLEMENTED);
}

void TCPStream::FlushOutput()
{
	throw CommsException("Not implemented", NOTIMPLEMENTED);
}

bool TCPStream::IsOpen()
{
	throw CommsException("Not implemented", NOTIMPLEMENTED);
}

IStream & TCPStream::operator >> (uint8_t & byte )
{
	throw CommsException("Operator not implemented", NOTIMPLEMENTED);
}

IStream & TCPStream::operator >> (char & byte )
{
	throw CommsException("Operator not implemented", NOTIMPLEMENTED);
}

IStream & TCPStream::operator >> (uint16_t & data16 )
{
	throw CommsException("Operator not implemented", NOTIMPLEMENTED);
}

IStream & TCPStream::operator >> (uint32_t & data32 )
{
	throw CommsException("Operator not implemented", NOTIMPLEMENTED);
}

}
