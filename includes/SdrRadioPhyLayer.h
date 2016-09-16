/*
 * SdrRadioPhyLayer.h
 *
 *  Created on: 16 set. 2016
 *      Author: diego
 */

#ifndef SRC_SDRRADIOPHYLAYER_H_
#define SRC_SDRRADIOPHYLAYER_H_

#include <IPhyLayer.h>
#include <fcntl.h> /* Defines O_* constants */
#include <sys/stat.h> /* Defines mode constants */
#include <mqueue.h>

namespace radiotransmission {

class SdrRadioPhyLayer: public IPhyLayer {
public:
	SdrRadioPhyLayer();
	virtual ~SdrRadioPhyLayer();
	virtual IPhyLayer & operator << (const DataLinkFrame &);
	virtual IPhyLayer & operator >> (DataLinkFrame &);
	virtual bool BusyTransmitting();
private:
	std::string txmqname, rxmqname;
	mqd_t txmqid, rxmqid;
};

} /* namespace radiotransmission */

#endif /* SRC_SDRRADIOPHYLAYER_H_ */
