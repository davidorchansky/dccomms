/*
 * Arduino.cpp
 *
 *  Created on: Feb 21, 2015
 *      Author: diego
 */

#include <Arduino.h>
#include <SerialPortInterface.h>
#include <Stream.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <exception>
#include <iostream>
#include <cstring>
namespace radiotransmission {


Arduino::Arduino(const char * p, SerialPortInterface::BaudRate baud):SerialPortInterface(p, baud){}

Arduino::Arduino(SerialPortInterface s):SerialPortInterface(s){}

Arduino::Arduino(SerialPortInterface s, const char* p, Arduino::BaudRate b, const char * h, const char * vr):SerialPortInterface(s){
	// TODO Auto-generated constructor stub
	hello = std::string(h);
	validReply = std::string(vr);
	strcpy(port, p);
	baud = b;
}



Arduino Arduino::FindArduino(Arduino::BaudRate baud, const char * hello, const char * validReply)
{
	int fd; /* File descriptor for the port */
	char msg[200];
	const char *spfiles[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3",
			"/dev/ttyACM4", "/dev/ttyACM5", "/dev/ttyACM6", "/dev/ttyACM7"};

	int i;
	SerialPortInterface::PortSettings ps;
	ps.baudrate = (SerialPortInterface::BaudRate) baud;
	ps.parity = SerialPortInterface::NOPARITY;
	ps.stopBits = SerialPortInterface::SB1;
	ps.dataBits = SerialPortInterface::CHAR8;

	for(i = 0; i <8; i++)
	{
//		try
//		{
			SerialPortInterface sp(spfiles[i], ps);
			bool opened = sp.Open();
			if (opened)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
				bool res = Arduino::_checkDevice(&sp, hello, validReply, 7000);
				if (res)
				{
					sp.FlushIO();
					return Arduino(sp, spfiles[i], baud, hello, validReply);
				}
			}
//		}
//		catch(std::exception &e)
//		{
//
//		}
	}
	return Arduino(SerialPortInterface("",ps));

}

Arduino::~Arduino() {
	// TODO Auto-generated destructor stub

}

bool Arduino::TryReconnect()
{
	int fd; /* File descriptor for the port */
	char msg[200];
	const char *spfiles[] = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3",
			"/dev/ttyACM4", "/dev/ttyACM5", "/dev/ttyACM6", "/dev/ttyACM7"};

	int i;
	for(i = 0; i <8; i++)
	{
		try
		{
			strcpy(port, spfiles[i]);

			//std::cout <<"Intentando abrir " << port << std::endl;
			bool opened = Open();
			if (opened)
			{
				std::cout <<"Chequeando " << port << "..." << std::endl << std::flush;
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));
				bool res = Arduino::_checkDevice(this, hello.c_str(), validReply.c_str(), 7000);
				if (res)
				{
					FlushIO();
					return true;
				}
				std::cout <<"Cerrando  " << port << "..." << std::endl << std::flush;
				Close();
			}
		}
		catch(std::exception e)
		{}
	}
	return false;
}

bool Arduino::_checkDevice(Stream *s, const char* h, const char* r, unsigned long long m)
{
	unsigned int replySize = strlen(r);
	//char* buffer = malloc(replySize);
	const char* max = r + replySize;
	const char* ptr = r;
	struct timeval time0, time1;
	gettimeofday(&time0, NULL);
	unsigned long long t0 = time0.tv_sec*1000 + time0.tv_usec/1000;
	unsigned long long t1 = t0;

	unsigned int n;
	char c;
	int helloSize = strlen(h);
	s->Write((uint8_t*)h, helloSize);
	while(t1 - t0 < m)
	{
		n = s->Available();
		if(n>0)
		{
			s->Read((uint8_t*)&c,1);
			if(c == *ptr)
				ptr++;
			else
				ptr = r;
		}
		if(ptr == max)
			return true;
		gettimeofday(&time1, NULL);
		t1 = time1.tv_sec*1000 + time1.tv_usec/1000;

	}
	return false;
}

} /* namespace radiotransmission */