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
#include <ICommsLink.h>
#include <queue>
#include <mutex>
#include <thread>

namespace dccomms {

#define IPHY_TYPE_DLINK 0
#define IPHY_TYPE_PHY 1

class CommsDeviceService: public ICommsLink {
public:
	enum PhyState {BUSY=0, READY};

	CommsDeviceService(int iphytype = IPHY_TYPE_DLINK, const DataLinkFrame::fcsType & fcs = DataLinkFrame::crc32, int maxframesize = 7000);
	virtual ~CommsDeviceService();

	virtual ICommsLink & operator << (const DataLinkFramePtr &);
	virtual ICommsLink & operator >> (DataLinkFramePtr &);

	virtual void Start();
	virtual void Stop();

	virtual unsigned int GetRxFifoSize();
	void SetChecksumType(DataLinkFrame::fcsType fcs);

	//Methods only for type IPHY_TYPE_DLINK
	virtual bool BusyTransmitting();

	//Methods only for type IPHY_TYPE_PHY
	virtual void SetPhyLayerState(const PhyState &);
	//SetPhyLayerState cambia el estado de la capa fisica y se lo hace saber a la capa de arriba

	//Two instances of CommsDeviceService for the same purpose in the same machine (for debug reasons) must have different namespaces
	//This method must be called before Start
	void SetNamespace(std::string nspace);
private:
	//El siguiente metodo es para Debug en la misma maquina y ha de llamarse antes que Start().
	//Especifica el prefijo de las colas de mensajes (sin contar el '/'
	void SetQueuePrefix(std::string _qprefix)
	{
		qprefix = _qprefix;
	}
	//Pide a la capa fisica su estado (solo para IPHY_TYPE_PHY)
	void ReqPhyLayerState();
	PhyState _GetPhyLayerState();

	void _SetPhyLayerState(const PhyState & state);

	DataLinkFramePtr GetNextFrame();
	void PushNewFrame(DataLinkFramePtr);

	void UpdateMQAttr();
	void ShowMQAttr(std::ostream &, int);

	long GetMaxMsgOnQueue(int);
	long GetMaxMsgSize(int);
	long GetNumMsgOnQueue(int);
	bool GetNonblockFlag(int);

	void SetNonblockFlag(bool, int);

	void SendPhyLayerState();
	void SendPhyLayerState(const PhyState &);

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
		ServiceThread(CommsDeviceService *);
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
		CommsDeviceService * physervice;
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

	struct mq_attr comattr;
	int comperm;
	std::string qprefix;

	unsigned int maxmsgsize;
	int type;
	PhyState phyState;
	DataLinkFrame::fcsType fcsType;

	//rxmsg y replymsg se tratan en un thread diferente a txmsg.
	//Concretamente, rxmsg y replymsg se tratan en un ServiceThread, y txmsg en el main thread
	ServiceMessage rxmsg, txmsg, replymsg;
	ServiceThread service;
};

} /* namespace radiotransmission */

#endif /* SRC_SDRRADIOPHYLAYER_H_ */
