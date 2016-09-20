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

#define IPHY_TYPE_DLINK 0
#define IPHY_TYPE_PHY 1

class SdrRadioPhyLayer: public IPhyLayer {
public:
	SdrRadioPhyLayer(int iphytype = IPHY_TYPE_DLINK, int maxframesize = 7000);
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
	void Init(int type, struct mq_attr attr, int perm);

	std::string txmqname, rxmqname, rtsmqname, ctsmqname;
	mqd_t txmqid, rxmqid, rtsmqid, ctsmqid;
	struct mq_attr txattr, rxattr, rtsattr, ctsattr;
	uint8_t * rxbuff;
	unsigned int rxbuffsize;
};

} /* namespace radiotransmission */

#endif /* SRC_SDRRADIOPHYLAYER_H_ */
