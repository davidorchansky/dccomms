/*
 * DLinkInterface.h
 *
 *  Created on: 22 oct. 2016
 *      Author: centelld
 */

#ifndef COMMSTCP_H_
#define COMMSTCP_H_

#include <dccomms/CommsDeviceService.h>
#include <dccomms/ICommsDevice.h>
#include <pthread.h>
#include <iostream>
#include <dccomms/Utils.h>
#include <string>
#include <mutex>
#include <Loggable.h>

namespace dcent {

#define MAX_ODATA_BUF 4096

using namespace dccomms;

class CommsBridge: public Loggable {
public:
	CommsBridge(ICommsDevice *, int _baudrate = 0, DataLinkFrame::fcsType _chksum = DataLinkFrame::fcsType::crc32);
	virtual ~CommsBridge();
	void Start();
	void Stop();

	//Two instances of CommsBridge for the same purpose in the same machine (for debug reasons) must have different namespaces
	//This method must be called before Start
	void SetNamespace(std::string nspace);

private:
	void TxWork();
	void RxWork();
	void TransmitFrame();
	bool ReceiveFrame();
	bool TryToConnect();
	bool TryToReconnect();
	Timer timer;
	unsigned int _frameTransmissionTime; //milis
	double _byteTransmissionTime; //milis

	std::string serv_namespace;
	CommsDeviceService phyService;
	DataLinkFramePtr txdlf;
	DataLinkFramePtr rxdlf;

	uint8_t obuf[MAX_ODATA_BUF];

	mutex devicemutex;
	bool connected;
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
