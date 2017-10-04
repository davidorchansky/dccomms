/*
 * DeviceInterface.cpp
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#include <dccomms/CommsException.h>
#include <dccomms/IStreamCommsDevice.h>

namespace dccomms {

IStreamCommsDevice::IStreamCommsDevice() {
  // TODO Auto-generated constructor stub
}

IStreamCommsDevice::~IStreamCommsDevice() {
  // TODO Auto-generated destructor stub
}

bool IStreamCommsDevice::Open() {
  // TODO Auto-generated destructor stub
  throw CommsException("Operator not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}

void IStreamCommsDevice::Close() {
  // TODO Auto-generated destructor stub
  throw CommsException("Operator not implemented",
                       COMMS_EXCEPTION_NOTIMPLEMENTED);
}

IStreamCommsDevice &IStreamCommsDevice::operator<<(const char *str) {
  IStream::operator<<(str);
  return *this;
}

IStreamCommsDevice &IStreamCommsDevice::operator<<(const std::string &str) {
  IStream::operator<<(str);
  return *this;
}

ICommsLink &IStreamCommsDevice::operator>>(PacketPtr pkt) {
  pkt->Read(this);
  return *this;
}

ICommsLink &IStreamCommsDevice::operator<<(PacketPtr pkt) {
  pkt->Write(this);
  return *this;
}
}
