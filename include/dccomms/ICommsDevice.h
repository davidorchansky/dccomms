#ifndef DCCOMMS_ICOMMSDEVICE_H
#define DCCOMMS_ICOMMSDEVICE_H

#include <dccomms/ICommsLink.h>

namespace dccomms {

class ICommsDevice : public ICommsLink {
public:
  ICommsDevice();

  virtual ICommsLink &operator>>(PacketPtr dlf) = 0;
  virtual ICommsLink &operator<<(PacketPtr dlf) = 0;

  virtual ICommsDevice &operator<<(const char *str) = 0;
  virtual ICommsDevice &operator<<(const std::string &) = 0;

  virtual bool Open() = 0;
  virtual void Close() = 0;
};

} /* namespace dccomms */

#endif // DCCOMMS_ICOMMSDEVICE_H
