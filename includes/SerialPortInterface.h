/*
 * SerialPortInterface.h
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#ifndef SERIALPORTINTERFACE_H_
#define SERIALPORTINTERFACE_H_


#include <Stream.h>
#include <DataLinkFrame.h>

#include <stdio.h>   /* Standard input/output definitions */

#include <string>

#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <IPhyLayerService.h>
#include <termios.h> /* POSIX terminal control definitions */

#include <sys/time.h> /*para timeout*/
#include <sys/ioctl.h>

namespace radiotransmission {

class SerialPortInterface : public Stream, public IPhyLayerService {
public:

    enum BaudRate {
        BAUD_50      = B50,
        BAUD_75      = B75,
        BAUD_110     = B110,
        BAUD_134     = B134,
        BAUD_150     = B150,
        BAUD_200     = B200,
        BAUD_300     = B300,
        BAUD_600     = B600,
        BAUD_1200    = B1200,
        BAUD_1800    = B1800,
        BAUD_2400    = B2400,
        BAUD_4800    = B4800,
        BAUD_9600    = B9600,
        BAUD_19200   = B19200,
        BAUD_38400   = B38400,
        BAUD_57600   = B57600,
        BAUD_115200  = B115200
    };
    enum Parity {
        EVEN,
        ODD,
        NOPARITY
    };

    enum StopBits {
        SB2,
        SB1
    };

    enum CharacterSize{
    	CHAR5 = CS5,
    	CHAR6 = CS6,
    	CHAR7 = CS7,
    	CHAR8 = CS8
    };

	struct PortSettings
	{
        BaudRate baudrate;
        Parity parity;
        StopBits stopBits;
        CharacterSize dataBits;

		PortSettings()
		{
			//Arduino default configuration
            baudrate = SerialPortInterface::BAUD_9600;
            parity   = SerialPortInterface::NOPARITY;
            stopBits = SerialPortInterface::SB1;
            dataBits = SerialPortInterface::CHAR8;
		}
	};

	SerialPortInterface();
	SerialPortInterface(const char *);
	SerialPortInterface(const char *, SerialPortInterface::BaudRate);
	SerialPortInterface(const char *, SerialPortInterface::PortSettings);
	bool Open();
	bool Open(const char *, SerialPortInterface::BaudRate);
	bool Open(const char *, SerialPortInterface::PortSettings);
	void Close();

	int Read(void *, uint32_t, unsigned long msTimeout=0);
	int Write(const void *, uint32_t, uint32_t msTimeout=0);

	Stream & operator >> (uint8_t &);
	Stream & operator >> (char &);
	Stream & operator >> (uint16_t &);
	Stream & operator >> (uint32_t &) ;
	Stream & operator << (uint8_t);
	Stream & operator << (const char * str);
	int Available();

	bool IsOpen();
	//void TimeoutMode(bool);
	void FlushInput();
	void FlushOutput();
	void FlushIO();
	virtual bool BusyTransmitting();
	virtual IPhyLayerService & operator << (const DataLinkFramePtr &);
	virtual IPhyLayerService & operator >> (DataLinkFramePtr &);

	void SetTimeout(unsigned long ms);

protected:
	PortSettings portSettings;
	int fd;
    char *  port;
    bool _open = false;


};

} /* namespace radiotransmission */

#endif /* SERIALPORTINTERFACE_H_ */
