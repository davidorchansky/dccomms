/*
 * DeviceInterface.h
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#ifndef DCCOMMS_ISTREAMCOMMSDEVICE_H_
#define DCCOMMS_ISTREAMCOMMSDEVICE_H_

#include <dccomms/CommsDevice.h>
#include <dccomms/Stream.h>

namespace dccomms {

class StreamCommsDevice : public Stream, public CommsDevice {
public:
  StreamCommsDevice();
  virtual ~StreamCommsDevice();

  virtual CommsDevice &operator>>(const PacketPtr &dlf);
  virtual CommsDevice &operator<<(const PacketPtr &dlf);

  virtual StreamCommsDevice &operator<<(const char *str);
  virtual StreamCommsDevice &operator<<(const std::string &);
};

} /* namespace dccomms */

#endif /* DCCOMMS_ISTREAMCOMMSDEVICE_H_ */
