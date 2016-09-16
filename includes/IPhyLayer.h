/*
 * IPhyLayer.h
 *
 *  Created on: 15 set. 2016
 *      Author: diego
 */

#ifndef INCLUDES_IPHYLAYER_H_
#define INCLUDES_IPHYLAYER_H_

#include <DataLinkFrame.h>

namespace radiotransmission {

class IPhyLayer {
public:
	IPhyLayer();
	virtual ~IPhyLayer();
	virtual IPhyLayer & operator << (const DataLinkFrame &) = 0;
	virtual IPhyLayer & operator >> (DataLinkFrame &) = 0;
	virtual bool BusyTransmitting() = 0;
	virtual void SetTimeout(unsigned long);
	virtual unsigned long GetTimeout();
protected:
	unsigned long _timeout = 0;
};

} /* namespace radiotransmission */

#endif /* INCLUDES_IPHYLAYER_H_ */
