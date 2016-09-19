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

#define TX_MQ 0
#define RX_MQ 1
#define RTS_MQ 2
#define CTS_MQ 3

#define MSG_TYPE_FRAME  0
#define MSG_TYPE_ISBUSY 1
#define MSG_TYPE_ISBUSY_REPLY 2
#define MSG_OVERHEAD 1

class SdrRadioPhyLayer: public IPhyLayer {
public:
	SdrRadioPhyLayer();
	virtual ~SdrRadioPhyLayer();
	virtual IPhyLayer & operator << (const DataLinkFrame &);
	virtual IPhyLayer & operator >> (DataLinkFrame &);
	virtual bool BusyTransmitting();
private:
	void UpdateMQAttr();
	void ShowMQAttr(std::ostream &, int);

	long GetMaxMsgOnQueue(int);
	long GetMaxMsgSize(int);
	long GetNumMsgOnQueue(int);
	bool GetNonblockFlag(int);

	void SetNonblockFlag(bool, int);

	struct mq_attr* GetMQAttr(int);
	mqd_t GetMQId(int);

	std::string txmqname, rxmqname, rtsmqname, ctsmqname;
	mqd_t txmqid, rxmqid, rtsmqid, ctsmqid;
	struct mq_attr txattr, rxattr, rtsattr, ctsattr;
	uint8_t * rxbuff;
	unsigned int rxbuffsize;
};

} /* namespace radiotransmission */

#endif /* SRC_SDRRADIOPHYLAYER_H_ */
