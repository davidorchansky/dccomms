/*
 * IPhyLayer.h
 *
 *  Created on: 15 set. 2016
 *      Author: diego
 */

#ifndef INCLUDES_IPHYLAYERSERVICE_H_
#define INCLUDES_IPHYLAYERSERVICE_H_

#include <DataLinkFrame.h>

namespace radiotransmission {

class IPhyLayerService {
public:
	IPhyLayerService();
	virtual ~IPhyLayerService();
	virtual IPhyLayerService & operator << (const DataLinkFrame &) = 0;
	virtual IPhyLayerService & operator >> (DataLinkFrame &) = 0;
	virtual bool BusyTransmitting() = 0;
	virtual void SetTimeout(unsigned long);
	virtual unsigned long GetTimeout();
protected:
	unsigned long _timeout = 0;
};

} /* namespace radiotransmission */

#endif /* INCLUDES_IPHYLAYERSERVICE_H_ */
