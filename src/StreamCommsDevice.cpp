/*
 * DeviceInterface.cpp
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#include <dccomms/CommsException.h>
#include <dccomms/StreamCommsDevice.h>

namespace dccomms {

StreamCommsDevice::StreamCommsDevice() {
  // TODO Auto-generated constructor stub
}

StreamCommsDevice::~StreamCommsDevice() {
  // TODO Auto-generated destructor stub
}

StreamCommsDevice &StreamCommsDevice::operator<<(const char *str) {
  Stream::operator<<(str);
  return *this;
}

StreamCommsDevice &StreamCommsDevice::operator<<(const std::string &str) {
  Stream::operator<<(str);
  return *this;
}

CommsDevice &StreamCommsDevice::operator>>(const PacketPtr &pkt) {
  pkt->Read(this);
  return *this;
}

CommsDevice &StreamCommsDevice::operator<<(const PacketPtr &pkt) {
  pkt->Write(this);
  return *this;
}
}
