/*
 * DeviceInterface.cpp
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#include <dccomms/IStreamCommsDevice.h>
#include <dccomms/CommsException.h>

namespace dccomms {

IStreamCommsDevice::IStreamCommsDevice() {
	// TODO Auto-generated constructor stub

}

IStreamCommsDevice::~IStreamCommsDevice() {
	// TODO Auto-generated destructor stub
}

bool IStreamCommsDevice::Open() {
	// TODO Auto-generated destructor stub
	throw CommsException("Operator not implemented", COMMS_EXCEPTION_NOTIMPLEMENTED);
}

void IStreamCommsDevice::Close() {
	// TODO Auto-generated destructor stub
	throw CommsException("Operator not implemented", COMMS_EXCEPTION_NOTIMPLEMENTED);
}

IStreamCommsDevice& IStreamCommsDevice::operator << (const char * str)
{
	IStream::operator <<(str);
	return *this;
}

IStreamCommsDevice& IStreamCommsDevice::operator << (const std::string & str)
{
	IStream::operator <<(str);
	return *this;
}

ICommsLink& IStreamCommsDevice::operator >> (DataLinkFramePtr & dlf)
{
	WaitFor((const uint8_t*) dlf->pre, DLNK_PREAMBLE_SIZE);

	Read(dlf->ddir, DLNK_DIR_SIZE);
	Read(dlf->sdir, DLNK_DIR_SIZE);

	Read((uint8_t *)dlf->dsize, DLNK_DSIZE_SIZE);

	if(dlf->_BigEndian)
	{
		dlf->payloadSize  = *dlf->dsize;
	}
	else
	{
		dlf->payloadSize = ((*dlf->dsize) << 8) | ((*dlf->dsize) >> 8);
	}

    if(dlf->payloadSize > DLNK_MAX_PAYLOAD_SIZE)
    {
    	throw CommsException(std::string("DLNKLAYER_ERROR: El tamano del payload no puede ser mayor que ")+ std::to_string(DLNK_MAX_PAYLOAD_SIZE), COMMS_EXCEPTION_DLNKLAYER_ERROR);
    }

	Read(dlf->payload, dlf->payloadSize);

	dlf->fcs = ((uint8_t *) dlf->payload) + dlf->payloadSize;
	Read(dlf->fcs, dlf->fcsSize);

	dlf->frameSize = dlf->overheadSize + dlf->payloadSize;

	dlf->dataIn = true;
	return *this;
}

ICommsLink& IStreamCommsDevice::operator<< (const DataLinkFramePtr & dlf)
{
	if(dlf->dataIn)
	{
		Write(dlf->pre,DLNK_PREAMBLE_SIZE);
		Write(dlf->ddir,DLNK_DIR_SIZE);
		Write(dlf->sdir,DLNK_DIR_SIZE);
		Write(dlf->dsize,DLNK_DSIZE_SIZE);
		Write(dlf->payload,dlf->payloadSize);
		Write(dlf->fcs, dlf->fcsSize);

		//FlushIO();//Lo ideal seria FlushOutput, pero en algun lado hay algo que hace que se llene el buffer de entrada
						  //y al final llega a bloquearse la comunicación... (TODO: comprobar qué es lo que hace que se llene el buffer de entrada)
	}
	return *this;
}

}
