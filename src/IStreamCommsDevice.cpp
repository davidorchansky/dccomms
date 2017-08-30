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
    WaitFor((const uint8_t*) dlf->_pre, DLNK_PREAMBLE_SIZE);

    Read(dlf->_ddir, DLNK_DIR_SIZE);
    Read(dlf->_sdir, DLNK_DIR_SIZE);

    Read((uint8_t *)dlf->_dsize, DLNK_DSIZE_SIZE);

    if(dlf->_BigEndian)
    {
      dlf->_payloadSize  = *dlf->_dsize;
    }
    else
    {
      dlf->_payloadSize = ((*dlf->_dsize) << 8) | ((*dlf->_dsize) >> 8);
    }

    if(dlf->_payloadSize > DLNK_MAX_PAYLOAD_SIZE)
    {
      throw CommsException(std::string("DLNKLAYER_ERROR: El tamano del payload no puede ser mayor que ")+ std::to_string(DLNK_MAX_PAYLOAD_SIZE), COMMS_EXCEPTION_DLNKLAYER_ERROR);
    }

    Read(dlf->_payload, dlf->_payloadSize);

    dlf->_fcs = ((uint8_t *) dlf->_payload) + dlf->_payloadSize;
    Read(dlf->_fcs, dlf->_fcsSize);

    dlf->_frameSize = dlf->_overheadSize + dlf->_payloadSize;

    dlf->_dataIn = true;
    return *this;
  }

  ICommsLink& IStreamCommsDevice::operator<< (const DataLinkFramePtr & dlf)
  {
    if(dlf->_dataIn)
    {
      Write(dlf->_pre,DLNK_PREAMBLE_SIZE);
      Write(dlf->_ddir,DLNK_DIR_SIZE);
      Write(dlf->_sdir,DLNK_DIR_SIZE);
      Write(dlf->_dsize,DLNK_DSIZE_SIZE);
      Write(dlf->_payload,dlf->_payloadSize);
      Write(dlf->_fcs, dlf->_fcsSize);

      //FlushIO();//Lo ideal seria FlushOutput, pero en algun lado hay algo que hace que se llene el buffer de entrada
      //y al final llega a bloquearse la comunicación... (TODO: comprobar qué es lo que hace que se llene el buffer de entrada)
    }
    return *this;
  }

}
