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
	enum PhyState {BUSY=0, READY};

	PhyLayerServiceV1(int iphytype = IPHY_TYPE_DLINK, const DataLinkFrame::fcsType & fcs = DataLinkFrame::crc32, int maxframesize = 7000);
	virtual ~PhyLayerServiceV1();

	virtual IPhyLayerService & operator << (const DataLinkFramePtr &);
	virtual IPhyLayerService & operator >> (DataLinkFramePtr &);

	virtual void Start();
	virtual void Stop();

	//IPHY_TYPE_DLINK (DlinkLyaer to PhyLayer) exclusive methods:
	virtual bool BusyTransmitting();

	//IPHY_TYPE_PHY (PhyLayer to DlinkLyaer) exclusive methods:
	virtual void SendState(const PhyState &);
private:

	int GetPhyLayerState();
	void SetPhyLayerState(PhyState & state);

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

	class ServiceMessage
	{
	public:
		enum MsgType {FRAME=0, REQ_STATE, CMD_STATE, NOTBUILT};

		ServiceMessage();
		~ServiceMessage();

		void Init(int maxmsgsize);

		MsgType GetMsgType() const {return (MsgType)*type;}
		PhyState GetPhyState() const {return (PhyState) *payload;}
		DataLinkFramePtr GetDataLinkFrame(const DataLinkFrame::fcsType &) const;
		void* GetBuffer() const {return buffer;}
		unsigned int GetSize() const {return size;}
		unsigned int GetMaxSize() const {return maxSize;}
		unsigned int GetMaxPayloadSize() const {return maxPayloadSize;}

		void BuildFrameMsg(const DataLinkFramePtr &);
		void BuildReqStateMsg();
		void BuildCmdStateMsg(const PhyState & state);



	private:
		void * buffer;
		uint8_t * payload;
		uint8_t * type;
		int size;
		int maxSize, maxPayloadSize;

	};

	class ServiceThread
	{
		//En C++11 una clase anidada puede acceder a los metodos de la "enclosing" class
	public:
		ServiceThread(PhyLayerServiceV1 *);
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
		PhyLayerServiceV1 * physervice;
	};

	void ReceiveMsg(ServiceMessage &);
	void SendMsg(const ServiceMessage &);

	void SavePhyStateFromMsg(const ServiceMessage &);
	void SaveFrameFromMsg(const ServiceMessage &);

	std::queue<DataLinkFramePtr> rxfifo;

	std::mutex rxfifo_mutex;
	std::mutex phyState_mutex;

	std::string txmqname, rxmqname;
	mqd_t txmqid, rxmqid;
	struct mq_attr txattr, rxattr;

	unsigned int maxmsgsize;
	int type;
	PhyState phyState;
	DataLinkFrame::fcsType fcsType;
	ServiceMessage rxmsg, txmsg;
	ServiceThread service;
};

} /* namespace radiotransmission */

#endif /* SRC_SDRRADIOPHYLAYER_H_ */
