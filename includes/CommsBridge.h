/*
 * DLinkInterface.h
 *
 *  Created on: 22 oct. 2016
 *      Author: centelld
 */

#ifndef COMMSTCP_H_
#define COMMSTCP_H_

#include <CommsDeviceService.h>
#include <ICommsDevice.h>
#include <pthread.h>
#include <iostream>
#include <Utils.h>
#include <TCPStream.h>




namespace dcent {

#define MAX_ODATA_BUF 4096

using namespace dccomms;

class CommsBridge {
public:
	CommsBridge(ICommsDevice *, int _baudrate = 0);
	virtual ~CommsBridge();
	void Start();
	void Stop();

private:
	void TxWork();
	void RxWork();
	void TransmitFrame();
	bool ReceiveFrame();

	Timer timer;
	unsigned int _frameTransmissionTime; //milis
	double _bitTransmissionTime; //milis

	pthread_mutex_t mutex;

	CommsDeviceService phyService;
	DataLinkFramePtr txdlf;
	DataLinkFramePtr rxdlf;

	uint8_t obuf[MAX_ODATA_BUF];

	bool transcurridoTiempoEnvio;
	int baudrate;
	ICommsDevice * device;

	ServiceThread<CommsBridge> txserv, rxserv;

	unsigned int GetMilliseconds(struct timeval * t)
	{
		  return  (*t).tv_sec *1000 + (*t).tv_usec/1000;
	}

	void PrintLog(std::string msg)
	{
		if (true)//elem != "SENDER")
		{
			std::cerr << "LOOP: " << msg << std::endl;
			std::cerr.flush();
		}
	}

};
} /* namespace dcent */

#endif /* COMMSTCP_H_ */
