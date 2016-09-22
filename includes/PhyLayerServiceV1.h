/*
 * SdrRadioPhyLayer.h
 *
 *  Created on: 16 set. 2016
 *      Author: diego
 */

#ifndef SRC_SDRRADIOPHYLAYER_H_
#define SRC_SDRRADIOPHYLAYER_H_

#include <fcntl.h> /* Defines O_* constants */
#include <sys/stat.h> /* Defines mode constants */
#include <mqueue.h>
#include <DataLinkFrame.h>
#include <IPhyLayerService.h>
#include <queue>
#include <mutex>

namespace radiotransmission {

#define IPHY_TYPE_DLINK 0
#define IPHY_TYPE_PHY 1


class PhyLayerServiceV1: public IPhyLayerService {
public:
	PhyLayerServiceV1(int iphytype = IPHY_TYPE_DLINK, int maxframesize = 7000);
	virtual ~PhyLayerServiceV1();
	virtual IPhyLayerService & operator << (const DataLinkFramePtr &);
	virtual IPhyLayerService & operator >> (DataLinkFramePtr &);

	virtual bool BusyTransmitting();
private:
	std::queue<DataLinkFramePtr> rxfifo;
	std::mutex rxfifo_mutex;

	int GetPhyLayerState();
	DataLinkFramePtr GetNextFrame();
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

	std::string txmqname, rxmqname;
	mqd_t txmqid, rxmqid;
	struct mq_attr txattr, rxattr;
	uint8_t * rxbuff;
	unsigned int rxbuffsize;
	int type;
};

} /* namespace radiotransmission */

#endif /* SRC_SDRRADIOPHYLAYER_H_ */
