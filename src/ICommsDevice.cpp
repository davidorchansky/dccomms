/*
 * DeviceInterface.cpp
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#include <dccomms/ICommsDevice.h>
#include <dccomms/CommsException.h>

namespace dccomms {

ICommsDevice::ICommsDevice() {
	// TODO Auto-generated constructor stub

}

ICommsDevice::~ICommsDevice() {
	// TODO Auto-generated destructor stub
}

bool ICommsDevice::Open() {
	// TODO Auto-generated destructor stub
	throw CommsException("Operator not implemented", COMMS_EXCEPTION_NOTIMPLEMENTED);
}

void ICommsDevice::Close() {
	// TODO Auto-generated destructor stub
	throw CommsException("Operator not implemented", COMMS_EXCEPTION_NOTIMPLEMENTED);
}

ICommsLink& ICommsDevice::operator >> (DataLinkFramePtr & dlf)
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

ICommsLink& ICommsDevice::operator<< (const DataLinkFramePtr & dlf)
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
