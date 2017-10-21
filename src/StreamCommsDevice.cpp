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

IStreamCommsDevice &IStreamCommsDevice::operator<<(const char *str) {
  IStream::operator<<(str);
  return *this;
}

IStreamCommsDevice &IStreamCommsDevice::operator<<(const std::string &str) {
  IStream::operator<<(str);
  return *this;
}

CommsDevice &IStreamCommsDevice::operator>>(const PacketPtr &pkt) {
  pkt->Read(this);
  return *this;
}

CommsDevice &IStreamCommsDevice::operator<<(const PacketPtr &pkt) {
  pkt->Write(this);
  return *this;
}
}