/*
 * SerialPortInterface.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <SerialPortInterface.h>


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

SerialPortInterface::SerialPortInterface(){}
SerialPortInterface::SerialPortInterface(const char * p)
{
	int s = strlen(p);
	port = new char[s+1];
	strcpy(port, p);
}

SerialPortInterface::SerialPortInterface(const char * p, SerialPortInterface::BaudRate baud)
{
	int s = strlen(p);
	port = new char[s+1];
	strcpy(port, p);
	portSettings.baudrate = baud;

}

SerialPortInterface::SerialPortInterface(const char * p, SerialPortInterface::PortSettings ps)
{
	int s = strlen(p);
	port = new char[s+1];
	strcpy(port, p);
	portSettings = ps;
}
IPhyLayerService & SerialPortInterface::operator << (const DataLinkFramePtr & dlf)
{
	Stream & s = *this;
	s << dlf;
	return *this;
}
IPhyLayerService & SerialPortInterface::operator >> (DataLinkFramePtr & dlf)
{
	Stream & s = *this;
	s >> dlf;
	return *this;
}

bool SerialPortInterface::BusyTransmitting()
{
	return false;
}

bool SerialPortInterface::Open(const char * p, SerialPortInterface::BaudRate baud)
{
	int s = strlen(p);
	port = new char[s+1];
	strcpy(port, p);
	portSettings.baudrate = baud;
	return Open();
}


bool SerialPortInterface::Open(const char * p, SerialPortInterface::PortSettings ps)
{
	int s = strlen(p);
	port = new char[s+1];
	strcpy(port, p);
	portSettings = ps;
	return Open();
}

bool SerialPortInterface::Open()
{
	struct termios options;

	fd = open(port, O_RDWR );
	if (fd != -1)
	{
		fcntl(fd, F_SETFL, FNDELAY);
		SetTimeout(_timeout);
		/*
		 * Get the current options for the port...
		 */
		tcgetattr(fd, &options);
		/*
		 * Set the baud rates to 19200...
		 */
		cfsetispeed(&options, portSettings.baudrate);
		cfsetospeed(&options, portSettings.baudrate);
		/*
		 * Enable the receiver and set local mode...
		 */
		options.c_cflag |= (CLOCAL | CREAD);
		/*
		 * Set the new options for the port...
		 */
		switch(portSettings.parity)
		{
		case NOPARITY:
			options.c_cflag &= ~PARENB;
			break;
		case EVEN:
			options.c_cflag |= PARENB;
			options.c_cflag &= ~PARODD;
			break;
		case ODD:
			options.c_cflag |= PARENB;
			options.c_cflag |= PARODD;
			break;

		}
		switch(portSettings.stopBits)
		{
		case SB2:
			options.c_cflag |= CSTOPB;
			break;
		case SB1:
			options.c_cflag &= ~CSTOPB;
			break;
		}

		options.c_cflag &= ~CSIZE;
		options.c_cflag |= portSettings.dataBits;

		//disable hardware flow control:
		options.c_cflag &= ~CRTSCTS;

		//disable software flow controls:
		options.c_iflag &= ~(IXON | IXOFF | IXANY);

		//...
		options.c_iflag &= ~ICRNL;

		//Choosing Raw Input
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

		//Choosing Raw Output
		options.c_oflag &= ~OPOST;

		tcsetattr(fd, TCSAFLUSH, &options);
		_open = true;
		return true;
	}
	_open = false;
	return false;

}

void SerialPortInterface::Close()
{

	close(fd);
	_open = false;
}



int SerialPortInterface::Read(void * buf, uint32_t size, unsigned long ms)
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
			else
			{
				char sig = '-'; //Un byte aleatorio...
				int res = write(fd, &sig, 1);
				if(res < 0)
				{
					close(fd);
					throw RadioException("Fallo de comunicacion al leer", RADIO_RXLINEDOWN);
				}
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

void SerialPortInterface::SetTimeout(unsigned long ms)
{

	_timeout = ms >= 0 ? ms : 0;
	if (ms)
		fcntl(fd, F_SETFL, FNDELAY);
	else
		fcntl(fd, F_SETFL, 0);
}


void SerialPortInterface::FlushInput()
{
	tcflush(fd, TCIFLUSH);
}

void SerialPortInterface::FlushIO()
{
	tcflush(fd, TCIOFLUSH);
}

void SerialPortInterface::FlushOutput()
{
	tcflush(fd, TCOFLUSH);
}
int SerialPortInterface::Write(const void * buf, uint32_t size, uint32_t to)
{

#ifdef BADWRITE
	int w = 0;
	for(int i = 0; i < size; i++)
	{
		w = write(fd, ((const uint8_t*)buf)+i, 1 );
		if(w < 0)
		{
			close(fd);
			throw RadioException("Fallo de comunicacion al escribir", RADIO_TXLINEDOWN);
		}
	}
	return w;
#elif BADWRITE2
	int w = 0;
	int increment = BADWRITE2;
	int left = size % increment;
	std::cerr << "Size: " << size << std::endl;
	std::cerr << "Left: " << left << std::endl;
	std::cerr << "Increment: " << increment << std::endl;
	int its = size / increment;
	std::cerr << "Its: " << its  << std::endl;
	uint8_t * ptr = (uint8_t *) buf;
	for(int i = 0; i < its; i++)
	{
		w = write(fd, ptr, increment);
		
		ptr += increment;
		if(w < 0)
		{
			close(fd);
			throw RadioException("Fallo de comunicacion al escribir", RADIO_TXLINEDOWN);
		}
	}
	if(left)
	{
		w = write(fd, ptr, left);

		if(w < 0)
		{
			close(fd);
			throw RadioException("Fallo de comunicacion al escribir", RADIO_TXLINEDOWN);
		}
	}
	return w;

#elif GOODWRITE
	int tbw = 0, bw;
	for(int i = 0; i < size; i += bw)
	{
		bw = write(fd, ((const uint8_t*) buf)+tbw, size-tbw);
		std::cerr << "Bytes written: " << bw << std::endl;
		tbw += bw;
		if(bw < 0)
		{
			close(fd);
			throw RadioException("Fallo de comunicacion al escribir", RADIO_TXLINEDOWN);
		}
	}
	return tbw;
#else
	int w = write(fd, (const uint8_t*)buf, size);
	if(w < 0)
	{
		close(fd);
		throw RadioException("Fallo de comunicacion al escribir", RADIO_TXLINEDOWN);
	}

	return w;
#endif

}

int SerialPortInterface::Available()
{
	int n;
	if(ioctl(fd, FIONREAD, &n)<0)
		return -1;	
	return n;
}

bool SerialPortInterface::IsOpen()
{
	return _open;
}

Stream & SerialPortInterface::operator >> (uint8_t & byte )
{
	fcntl(fd, F_SETFL, 0);
	read(fd, &byte, sizeof(uint8_t));
	fcntl(fd, F_SETFL, FNDELAY);
	return *this;
}

Stream & SerialPortInterface::operator >> (char & byte )
{
	fcntl(fd, F_SETFL, 0);
	read(fd, &byte, sizeof(uint8_t));
	fcntl(fd, F_SETFL, FNDELAY);
	return *this;
}

Stream & SerialPortInterface::operator >> (uint16_t & data16 )
{
	fcntl(fd, F_SETFL, 0);
	read(fd, &data16, sizeof(uint16_t));
	fcntl(fd, F_SETFL, FNDELAY);
	return *this;
}

Stream & SerialPortInterface::operator >> (uint32_t & data32 )
{
	fcntl(fd, F_SETFL, 0);
	read(fd, &data32, sizeof(uint32_t));
	fcntl(fd, F_SETFL, FNDELAY);
	return *this;
}

Stream & SerialPortInterface::operator << (uint8_t byte)
{
	write(fd, &byte, sizeof(uint8_t));
	return *this;
}

Stream & SerialPortInterface::operator << (const char * str)
{
	int n = sizeof(str);
	write(fd, str, n);
	return *this;
}
} /* namespace radiotransmission */
