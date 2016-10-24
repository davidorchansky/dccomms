/*
 * DeviceInterface.cpp
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#include <ICommsDevice.h>
#include <CommsException.h>

namespace dccomms {

ICommsDevice::ICommsDevice() {
	// TODO Auto-generated constructor stub

}

ICommsDevice::~ICommsDevice() {
	// TODO Auto-generated destructor stub
}

bool ICommsDevice::Open() {
	// TODO Auto-generated destructor stub
	throw CommsException("Operator not implemented", NOTIMPLEMENTED);
}

void ICommsDevice::Close() {
	// TODO Auto-generated destructor stub
	throw CommsException("Operator not implemented", NOTIMPLEMENTED);
}

ICommsLink& ICommsDevice::operator >> (DataLinkFramePtr & dlf)
{
	WaitFor((const uint8_t*) dlf->pre, DLNK_PREAMBLE_SIZE);

	Read(dlf->ddir, DLNK_DIR_SIZE);
	Read(dlf->sdir, DLNK_DIR_SIZE);

	Read((uint8_t *)dlf->dsize, DLNK_DSIZE_SIZE);

	if(dlf->_BigEndian)
	{
		dlf->dataSize  = *dlf->dsize;
	}
	else
	{
		dlf->dataSize = ((*dlf->dsize) << 8) | ((*dlf->dsize) >> 8);
	}

    if(dlf->dataSize > DLNK_MAX_PAYLOAD_SIZE)
    {
    	throw CommsException(std::string("DLNKLAYER_ERROR: El tamano del payload no puede ser mayor que ")+ std::to_string(DLNK_MAX_PAYLOAD_SIZE), DLNKLAYER_ERROR);
    }

	Read(dlf->payload, dlf->dataSize);

	dlf->fcs = ((uint8_t *) dlf->payload) + dlf->dataSize;
	Read(dlf->fcs, dlf->fcsSize);

	dlf->frameSize = dlf->overheadSize + dlf->dataSize;

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
		Write(dlf->payload,dlf->dataSize);
		Write(dlf->fcs, dlf->fcsSize);

		FlushIO();//Lo ideal seria FlushOutput, pero en algun lado hay algo que hace que se llene el buffer de entrada
						  //y al final llega a bloquearse la comunicación... (TODO: comprobar qué es lo que hace que se llene el buffer de entrada)
	}
	return *this;
}

}
