/*
 * IPhyLayer.h
 *
 *  Created on: 15 set. 2016
 *      Author: diego
 */

#ifndef DCCOMMS_ICOMMSLINK_H_
#define DCCOMMS_ICOMMSLINK_H_

#include <cpplogging/Loggable.h>
#include <dccomms/Packet.h>

using namespace cpplogging;
namespace dccomms {

class ICommsLink : public virtual Loggable {
public:
  ICommsLink();
  virtual ~ICommsLink();
  virtual ICommsLink &operator<<(const PacketPtr &) = 0;
  virtual ICommsLink &operator>>(const PacketPtr &) = 0;
  virtual bool BusyTransmitting();
  virtual void SetTimeout(unsigned long);
  virtual unsigned long GetTimeout();

protected:
  unsigned long _timeout = 0;
};

} /* namespace radiotransmission */

#endif /* DCCOMMS_ICOMMSLINK_H_ */
