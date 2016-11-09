/*
 * IPhyLayer.h
 *
 *  Created on: 15 set. 2016
 *      Author: diego
 */

#ifndef INCLUDES_ICOMMSLINK_H_
#define INCLUDES_ICOMMSLINK_H_

#include <DataLinkFrame.h>
#include <Loggable.h>

namespace dccomms {

class ICommsLink: public Loggable{
public:
	ICommsLink();
	virtual ~ICommsLink();
	virtual ICommsLink & operator << (const DataLinkFramePtr &) = 0;
	virtual ICommsLink & operator >> (DataLinkFramePtr &) = 0;
	virtual bool BusyTransmitting();
	virtual void SetTimeout(unsigned long);
	virtual unsigned long GetTimeout();
protected:
	unsigned long _timeout = 0;
};

} /* namespace radiotransmission */

#endif /* INCLUDES_ICOMMSLINK_H_ */
