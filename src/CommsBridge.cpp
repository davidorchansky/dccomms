/*
 * DLinkInterface.cpp
 *
 *  Created on: 22 oct. 2016
 *      Author: centelld
 */

#include <string>
#include <iostream>
#include <sys/time.h>
#include <sys/stat.h>
#include <Utils.h>
#include <CommsException.h>
#include <boost/algorithm/string.hpp>
#include <CommsBridge.h>
#include <ICommsDevice.h>

namespace dcent {
using namespace std;

CommsBridge::CommsBridge(ICommsDevice * _device, int _baudrate, DataLinkFrame::fcsType chksum): phyService(IPHY_TYPE_PHY, chksum), txserv(this), rxserv(this) {
	rxdlf = DataLinkFrame::BuildDataLinkFrame(chksum);
	txdlf = DataLinkFrame::BuildDataLinkFrame(chksum);
	baudrate = _baudrate;
	device = _device;
	txserv.SetWork(&CommsBridge::TxWork);
	rxserv.SetWork(&CommsBridge::RxWork);
	serv_namespace = "";
	connected = false;
}

CommsBridge::~CommsBridge() {
	Stop();
}

void CommsBridge::SetNamespace(std::string nspace)
{
	serv_namespace = nspace;
	phyService.SetNamespace(serv_namespace);
}

void CommsBridge::Start()
{
	_byteTransmissionTime =  1000./( baudrate / 8.);
	phyService.Start();
	TryToConnect();
	txserv.Start();
	rxserv.Start();
}

void CommsBridge::Stop()
{
	txserv.Stop();
	rxserv.Stop();
	while(txserv.IsRunning() || rxserv.IsRunning()){}
	connected = false;
	device->Close();
}

void CommsBridge::RxWork()
{
	LOG_DEBUG("RX: waiting for frame from the device...");
	bool noerrors = ReceiveFrame();
	if(noerrors)
	{
		//PACKET OK
		LOG_DEBUG("RX: received frame without errors");
		LOG_DEBUG("RX: delivering received frame to the upper layer...");
		phyService << rxdlf;
		LOG_DEBUG("RX: frame delivered to the upper layer");

	}
	else
	{
		//PACKET WITH ERRORS
		LOG_DEBUG("RX: received frame with errors. Frame will be discarded");
	}
}

void CommsBridge::TransmitFrame()
{
	LOG_DEBUG("TX: transmitting frame...");
	*device << txdlf;
	LOG_DEBUG("TX: frame transmitted");
}

bool CommsBridge::ReceiveFrame()
{
	try
	{
		*device >> rxdlf;
		return rxdlf->checkFrame();
	}
	catch(CommsException &e)
	{
		std::string msg = e.what();
		LOG_DEBUG("EXCEPTION!!!!!!!!!!!!!!!");
		switch (e.code)
		{
		case LINEDOWN:
			LOG_DEBUG("CONNECTION LOST WITH DEVICE WHEN READING: "+msg);
			TryToReconnect();
			break;
		}
	}
	return true;
}

void CommsBridge::TxWork()
{
	try
	{
		if(phyService.GetRxFifoSize() > 0)
		{
			phyService.SetPhyLayerState(CommsDeviceService::BUSY);
			while(phyService.GetRxFifoSize() > 0)
			{
				LOG_DEBUG("TX: FIFO size: " + std::to_string(phyService.GetRxFifoSize()));
				phyService >> txdlf;
				if(txdlf->checkFrame())
				{
					//PACKET OK
					LOG_DEBUG("TX: frame is OK, ready to send");
					TransmitFrame();
					unsigned int frameSize = txdlf->GetFrameSize();
					_frameTransmissionTime = ceil(frameSize * _byteTransmissionTime);
					LOG_DEBUG("frame transmission time: " + std::to_string(_frameTransmissionTime));
					timer.Reset();
					unsigned int elapsed = 0;
					while(elapsed < _frameTransmissionTime)
					{
						elapsed = timer.Elapsed();
					}
					LOG_DEBUG("Tiempo transcurrido: "+std::to_string(elapsed));

				}
				else
				{
					//PACKET WITH ERRORS
					LOG_DEBUG("TX: INTERNAL ERROR: frame received with errors from the upper layer!");
				}
			}
			phyService.SetPhyLayerState(CommsDeviceService::READY);
		}
	}
	catch(CommsException & e)
	{
		std::string msg = e.what();
		switch (e.code)
		{
		case LINEDOWN:
			LOG_DEBUG("CONNECTION LOST WITH DEVICE WHEN WRITTING: "+msg);
			//TryToReconnect();
			break;
		}
	}
}

bool CommsBridge::TryToReconnect()
{
	devicemutex.lock();
	connected = false;
	TryToConnect();
	devicemutex.unlock();
	return true;
}

bool CommsBridge::TryToConnect()
{
	phyService.SetPhyLayerState(CommsDeviceService::BUSY);
	while (!connected)
	{
		try
		{
			device->Open();
			connected = true;
		}
		catch(CommsException & e)
		{
			std::string msg = e.what();
			LOG_DEBUG("Problem happened when trying to connect with the comms device (" + msg +")... Trying again...");
			Utils::Sleep(1000);
		}

	}
	phyService.SetPhyLayerState(CommsDeviceService::READY);
	return connected;
}


} /* namespace dcent */
