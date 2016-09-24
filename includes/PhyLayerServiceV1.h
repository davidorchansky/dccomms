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
#include <thread>

namespace radiotransmission {

#define IPHY_TYPE_DLINK 0
#define IPHY_TYPE_PHY 1


class PhyLayerServiceV1: public IPhyLayerService {
public:
	PhyLayerServiceV1(int iphytype = IPHY_TYPE_DLINK, int maxframesize = 7000);
	virtual ~PhyLayerServiceV1();

	virtual IPhyLayerService & operator << (const DataLinkFramePtr &);
	virtual IPhyLayerService & operator >> (DataLinkFramePtr &);

	virtual void Start();
	virtual void Stop();

	//IPHY_TYPE_DLINK (DlinkLyaer to PhyLayer) exclusive methods:
	virtual bool BusyTransmitting();

	//IPHY_TYPE_PHY (PhyLayer to DlinkLyaer) exclusive methods:
	virtual void SendState(int);
private:
	int GetPhyLayerState();
	void SetPhyLayerState(int state);

	DataLinkFramePtr GetNextFrame();
	void PushNewFrame(DataLinkFramePtr);

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

	void ReciveMsg();
	void SendMsg();

	class ServiceMessage
	{
	public:
		static enum MsgType {FRAME, CMD_STATE, REPLY_STATE};
		static int MSG_OVERHEAD = 1;
		ServiceMessage(int maxsize);
		~ServiceMessage();
	private:
		void * buffer;
		void * payload;
		uint8_t * type;
	};

	class ServiceThread
	{
		//En C++11 una clase anidada puede acceder a los metodos de la "enclosing" class
	public:
		ServiceThread();
		~ServiceThread();
		bool IsRunning();
		void Start();
		void Stop();
		void Work();
	private:
		std::thread thread;
		bool mcontinue;
		bool terminated;
		bool started;

	};

	std::queue<DataLinkFramePtr> rxfifo;

	std::mutex rxfifo_mutex;
	std::mutex phyState_mutex;

	std::string txmqname, rxmqname;
	mqd_t txmqid, rxmqid;
	struct mq_attr txattr, rxattr;

	unsigned int maxmsgsize;
	int type;
	int phyState;

	ServiceMessage rxmsg, txmsg;
	ServiceThread service;
};

} /* namespace radiotransmission */

#endif /* SRC_SDRRADIOPHYLAYER_H_ */
