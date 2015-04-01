/*
 * Arduino.h
 *
 *  Created on: Feb 21, 2015
 *      Author: diego
 */

#ifndef ARDUINO_H_
#define ARDUINO_H_

#include <SerialPortInterface.h>
#include <Stream.h>

namespace radiotransmission {

class Arduino : public SerialPortInterface {
public:
	   enum BaudRate {
	        BAUD_50      = SerialPortInterface::BAUD_50,
	        BAUD_75      = SerialPortInterface::BAUD_75,
	        BAUD_110     = SerialPortInterface::BAUD_110,
	        BAUD_134     = SerialPortInterface::BAUD_134,
	        BAUD_150     = SerialPortInterface::BAUD_150,
	        BAUD_200     = SerialPortInterface::BAUD_200,
	        BAUD_300     = SerialPortInterface::BAUD_300,
	        BAUD_600     = SerialPortInterface::BAUD_600,
	        BAUD_1200    = SerialPortInterface::BAUD_1200,
	        BAUD_1800    = SerialPortInterface::BAUD_1800,
	        BAUD_2400    = SerialPortInterface::BAUD_2400,
	        BAUD_4800    = SerialPortInterface::BAUD_4800,
	        BAUD_9600    = SerialPortInterface::BAUD_9600,
	        BAUD_19200   = SerialPortInterface::BAUD_19200,
	        BAUD_38400   = SerialPortInterface::BAUD_38400,
	        BAUD_57600   = SerialPortInterface::BAUD_57600,
	        BAUD_115200  = SerialPortInterface::BAUD_115200
	    };

	Arduino(const char * p, SerialPortInterface::BaudRate baud);
	Arduino(SerialPortInterface s);
	Arduino(SerialPortInterface, const char* port, Arduino::BaudRate, const char * hello, const char * validReply);
	bool TryReconnect();
	virtual ~Arduino();

	static Arduino FindArduino(Arduino::BaudRate, const char * hello, const char * validReply);

	std::string hello;
	std::string validReply;
	Arduino::BaudRate baud;

private:

	Arduino();
	static bool _checkDevice(Stream *s, const char* h, const char* r, unsigned long long m=0);
};

} /* namespace radiotransmission */

#endif /* ARDUINO_H_ */
