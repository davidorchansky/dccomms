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
#include <dccomms/Utils.h>
#include <dccomms/CommsException.h>
#include <boost/algorithm/string.hpp>
#include <dccomms/CommsBridge.h>
#include <dccomms/IStreamCommsDevice.h>

namespace dccomms {
using namespace std;

CommsBridge::CommsBridge(IStreamCommsDevice * _device, int _baudrate, DataLinkFrame::fcsType chksum): phyService(IPHY_TYPE_PHY, chksum), txserv(this), rxserv(this) {
	rxdlf = DataLinkFrame::BuildDataLinkFrame(chksum);
	txdlf = DataLinkFrame::BuildDataLinkFrame(chksum);
	baudrate = _baudrate;
	device = _device;
	txserv.SetWork(&CommsBridge::TxWork);
	rxserv.SetWork(&CommsBridge::RxWork);
	serv_namespace = "";
	connected = false;
	SetLogName("CommsBridge");
    rxtrp = TransportPDU::BuildTransportPDU (0);
    txtrp = TransportPDU::BuildTransportPDU (0);
}

CommsBridge::~CommsBridge() {
	Stop();
}

void CommsBridge::SetNamespace(std::string nspace)
{
	serv_namespace = nspace;
	phyService.SetNamespace(serv_namespace);
}

void CommsBridge::SetLogName(std::string name)
{
	Loggable::SetLogName(name);
	phyService.SetLogName(name +":CommsDeviceService");
}
void CommsBridge::SetLogLevel(Loggable::LogLevel _level)
{
	Loggable::SetLogLevel(_level);
	phyService.SetLogLevel(_level);
}

void CommsBridge::LogToConsole(bool c)
{
    Loggable::LogToConsole(c);
    phyService.LogToConsole(c);
}

void CommsBridge::LogToFile(const string &filename)
{
    Loggable::LogToFile(filename);
    phyService.LogToFile(filename + "_service");
}

void CommsBridge::FlushLog()
{
    Loggable::FlushLog();
    phyService.FlushLog();
}

void CommsBridge::FlushLogOn(LogLevel level)
{
    Loggable::FlushLogOn(level);
    phyService.FlushLogOn(level);
}

void CommsBridge::Start()
{
	_byteTransmissionTime =  1000./( baudrate / 8.);
	phyService.Start();
	TryToConnect();
    Log->debug("starting TX service...");
	txserv.Start();
    Log->debug("starting RX service...");
	rxserv.Start();

    Log->debug("setting state ready...");
	phyService.SetPhyLayerState(CommsDeviceService::READY);
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
	Log->debug("RX: waiting for frame from the device...");
	bool noerrors = ReceiveFrame();
	if(noerrors)
	{
        //PACKET OK
        rxtrp->UpdateBuffer(rxdlf->GetPayloadBuffer ());
        Log->debug("RX: received frame without errors (Seq: {}) (FS: {}).", rxtrp->GetSeqNum (), rxdlf->GetFrameSize());
		Log->debug("RX: delivering received frame to the upper layer...");
		phyService << rxdlf;
		Log->debug("RX: frame delivered to the upper layer");

	}
	else
	{
		//PACKET WITH ERRORS
        Log->warn("RX: received frame with errors. Frame will be discarded (FS: {}).", rxdlf->GetFrameSize());
	}
}

void CommsBridge::TransmitFrame()
{
    txtrp->UpdateBuffer(txdlf->GetPayloadBuffer ());
    Log->debug("TX: transmitting frame... (Seq: {}) (FS: {}).", txtrp->GetSeqNum (), txdlf->GetFrameSize());
	*device << txdlf;
	Log->debug("TX: frame transmitted");
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
		switch (e.code)
		{
		case COMMS_EXCEPTION_LINEDOWN:
            Log->error("RX: CONNECTION LOST WITH DEVICE WHEN READING: {}", msg);
			TryToReconnect();
			break;
		default:
            Log->error("RX: Unknown error when receiving next frame: {}\n"
					"Considering errors in the frame."
					, msg);

			break;

		}
		return false;
	}
	return true;
}

void CommsBridge::TxWork()
{
	try
	{
        Log->debug("TX: waiting for frames to transmit");
		phyService.WaitForFramesFromRxFifo();
        Log->debug("TX: frames available. Setting phylayer state BUSY");
		phyService.SetPhyLayerState(CommsDeviceService::BUSY);
		do
		{
			phyService >> txdlf;
			Log->debug("TX: FIFO size: {}", phyService.GetRxFifoSize());

			if(txdlf->checkFrame())
			{
                //PACKET OK
				TransmitFrame();
				unsigned int frameSize = txdlf->GetFrameSize();
				_frameTransmissionTime = ceil(frameSize * _byteTransmissionTime);
                Log->debug("TX: estimated frame transmission time: {} ms (FS: {}).", _frameTransmissionTime, frameSize);
				timer.Reset();
				unsigned int elapsed = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(_frameTransmissionTime));
                elapsed = timer.Elapsed();
                Log->debug("TX: elapsed time: {} ms", elapsed);

			}
			else
			{
				//PACKET WITH ERRORS
				Log->critical("TX: INTERNAL ERROR: frame received with errors from the upper layer!");
			}
		}while(phyService.GetRxFifoSize() > 0);

        Log->debug("TX: transmitted all frames in FIFO. Setting phylayer state to READY");
		phyService.SetPhyLayerState(CommsDeviceService::READY);

	}
	catch(CommsException & e)
	{
		std::string msg = e.what();
		switch (e.code)
		{
		case COMMS_EXCEPTION_LINEDOWN:
            Log->error("TX: CONNECTION LOST WITH DEVICE WHEN WRITTING: {}", msg);
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
    Log->debug("Traying to connect with the device... Setting phylayer state as BUSY");
	phyService.SetPhyLayerState(CommsDeviceService::BUSY);
	while (!connected)
	{
		try
		{
			device->Open();
			connected = true;
            Log->info("Device connected");
		}
		catch(CommsException & e)
		{
			std::string msg = e.what();
            Log->error("Problem happened when trying to connect with the comms device ({})... Trying again...", msg);
			Utils::Sleep(1000);
		}

	}
    Log->debug("Setting the phylayer state to READY");
	phyService.SetPhyLayerState(CommsDeviceService::READY);
	return connected;
}


} /* namespace dcent */
