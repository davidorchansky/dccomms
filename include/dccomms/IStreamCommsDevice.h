/*
 * DeviceInterface.h
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#ifndef DCCOMMS_ISTREAMCOMMSDEVICE_H_
#define DCCOMMS_ISTREAMCOMMSDEVICE_H_

#include <dccomms/ICommsDevice.h>
#include <dccomms/IStream.h>

namespace dccomms {

class IStreamCommsDevice : public IStream, public ICommsDevice {
public:
  IStreamCommsDevice();
  virtual ~IStreamCommsDevice();

  virtual ICommsLink &operator>>(PacketPtr dlf);
  virtual ICommsLink &operator<<(PacketPtr dlf);

  virtual IStreamCommsDevice &operator<<(const char *str);
  virtual IStreamCommsDevice &operator<<(const std::string &);

  virtual bool Open();
  virtual void Close();
};

} /* namespace dccomms */

#endif /* DCCOMMS_ISTREAMCOMMSDEVICE_H_ */
