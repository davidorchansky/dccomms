/*
 * SdrRadioPhyLayer.cpp
 *
 *  Created on: 16 set. 2016
 *      Author: diego
 */

#include <errno.h>
#include <RadioException.h>
#include <iostream>
#include <fcntl.h> /* Defines O_* constants */
#include <sys/stat.h> /* Defines mode constants */
#include <mqueue.h>
#include <PhyLayerServiceV1.h>
#include <chrono>
#include <Utils.h>

namespace radiotransmission {

#define TX_MQ 0
#define RX_MQ 1
#define RTS_MQ 2
#define CTS_MQ 3

#define MSG_OVERHEAD_SIZE 1

static void ThrowPhyLayerException(std::string msg)
{
	throw RadioException("PHYLAYER EXCEPTION: "+msg, RADIO_PHYLAYER_ERROR);
}


std::string GetMQErrorMsg(int e)
{
	switch(e)
	{
	case EACCES:
		return "The queue exists, but the caller does not have permission to open it in the specified mode / Name Contained more than one slash";
	case EEXIST:
		return "Both O_CREAT and O_EXCL were specified in oflag, but a queue with this name already exists";
	case EINVAL:
		return "O_CREAT  was  specified in oflag, and attr was not NULL, but attr->mq_maxmsg or attr->mq_msqsize was invalid.  Both of these\
fields must be greater than zero.  In a process that is  unprivileged  (does  not  have  the  CAP_SYS_RESOURCE  capability),\
attr->mq_maxmsg must be less than or equal to the msg_max limit, and attr->mq_msgsize must be less than or equal to the msg‐\
size_max limit.  In addition, even in a privileged process, attr->mq_maxmsg cannot exceed the HARD_MAX limit.  (See mq_over‐\
view(7) for details of these limits.)";
	case EMFILE:
		return "The process already has the maximum number of files and message queues open.";
	case ENAMETOOLONG:
		return "name was too long.";
	case ENFILE:
		return "The system limit on the total number of open files and message queues has been reached.";
	case ENOENT:
		return "The O_CREAT flag was not specified in oflag, and no queue with this name exists.";
	case ENOMEM:
		return "Insufficient memory.";
	case ENOSPC:
		return "Insufficient space for the creation of a new message queue.  This probably occurred because the queues_max limit was encoun‐\
tered; see mq_overview(7).";
	default:
		return "Unknown Error";
	}
}
PhyLayerServiceV1::PhyLayerServiceV1(int type, const DataLinkFrame::fcsType & _fcsType, int maxframesize): service(this){
	struct mq_attr attr;
	attr.mq_maxmsg = 10;
	attr.mq_msgsize = maxframesize + MSG_OVERHEAD_SIZE;
	int perm = 0644;
	fcsType = _fcsType;
	Init(type, attr, perm);
}

void PhyLayerServiceV1::Init(int _type, struct mq_attr attr, int perm)
{
	type = _type;
	switch(_type)
	{
	case IPHY_TYPE_DLINK:
		txmqname = "/tx_dlnk_phy";
		rxmqname = "/rx_dlnk_phy";
		_SetPhyLayerState(PhyState::BUSY);
		break;
	case IPHY_TYPE_PHY:
		rxmqname = "/tx_dlnk_phy";
		txmqname = "/rx_dlnk_phy";
		_SetPhyLayerState(PhyState::READY);
		break;
	default:
		ThrowPhyLayerException("Tipo de interfaz con la capa fisica incorrecto");
	}

	txattr = attr;
	rxattr = attr;

	int openops = O_CREAT;

	txmqid = mq_open(txmqname.c_str(), openops | O_WRONLY , perm, &txattr);
	std::string emsg;
	if(txmqid == -1)
	{
		emsg = GetMQErrorMsg(errno);
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola para envio de mensajes: ") + emsg);
	}
	rxmqid = mq_open(rxmqname.c_str(), openops | O_RDONLY, perm, &rxattr);
	if(rxmqid == -1)
	{
		emsg = GetMQErrorMsg(errno);
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola para recepcion de mensajes") + emsg);
	}


	SetNonblockFlag(false, TX_MQ);
	SetNonblockFlag(false, RX_MQ);

#ifdef DEBUG
	std::cerr << "TXMQ:" << std::endl;
	ShowMQAttr(std::cerr, TX_MQ);
	std::cerr << "RXMQ:" << std::endl;
	ShowMQAttr(std::cerr, RX_MQ);
#endif

	maxmsgsize = GetMaxMsgSize(RX_MQ);
	rxmsg.Init(maxmsgsize);
	txmsg.Init(maxmsgsize);
	replymsg.Init(maxmsgsize);
}

PhyLayerServiceV1::~PhyLayerServiceV1() {
	// TODO Auto-generated destructor stub
	mq_close(rxmqid);
	mq_close(txmqid);
	mq_unlink(txmqname.c_str());
	mq_unlink(rxmqname.c_str());
}

void PhyLayerServiceV1::UpdateMQAttr ()
{
	if(mq_getattr(txmqid, &txattr)==-1)
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible obtener los atributos de la cola de mensajes tx"));
	if(mq_getattr(rxmqid, &rxattr)==-1)
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible obtener los atributos de la cola de mensajes rx"));
}

struct mq_attr* PhyLayerServiceV1::GetMQAttr(int mq)
{
	UpdateMQAttr();

	struct mq_attr * attr;

	switch(mq)
	{
	case TX_MQ:
		attr = &txattr;
		break;
	case RX_MQ:
		attr = &rxattr;
		break;
	default:
		ThrowPhyLayerException("Error interno: la cola de mensajes no existe");
	}
	return attr;
}

mqd_t PhyLayerServiceV1::GetMQId(int mq)
{
	switch(mq)
	{
	case TX_MQ:
		return txmqid;
	case RX_MQ:
		return rxmqid;
	default:
		ThrowPhyLayerException("Error interno: la cola de mensajes no existe");
	}
	return 0; //nunca llegara aqui
}

void PhyLayerServiceV1::ShowMQAttr(std::ostream & o, int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);

	o << " - Maximum # of messages on queue:\t"<< attr->mq_maxmsg << std::endl;
	o << " - Maximum message size:\t"<< attr->mq_msgsize << std::endl;
	o << " - # of messages currently on queue:\t"<< attr->mq_curmsgs << std::endl;
	o << " - O_NONBLOCK:\t" << (attr->mq_flags & O_NONBLOCK ? "activado" : "desactivado") << std::endl;
}

long PhyLayerServiceV1::GetMaxMsgOnQueue(int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	return attr->mq_maxmsg;
}

long PhyLayerServiceV1::GetMaxMsgSize(int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	return attr->mq_msgsize;
}

long PhyLayerServiceV1::GetNumMsgOnQueue(int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	return attr->mq_curmsgs;
}

bool PhyLayerServiceV1::GetNonblockFlag(int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	return attr->mq_flags & O_NONBLOCK;
}

void PhyLayerServiceV1::SetNonblockFlag(bool v, int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	mqd_t id = GetMQId(mq);
	if(v)
		attr->mq_flags |= O_NONBLOCK;
	else
		attr->mq_flags &= ~O_NONBLOCK;

	if(mq_setattr(id, attr, NULL)==-1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible establecer los atributos de la cola de mensajes"));
	}
}

IPhyLayerService & PhyLayerServiceV1::operator << (const DataLinkFramePtr & dlf)
{
	txmsg.BuildFrameMsg(dlf);
	SendMsg(txmsg);

	return *this; //nunca llegara aqui

}


void PhyLayerServiceV1::SendMsg(const ServiceMessage & msg)
{
	if(mq_send(txmqid, (char*)  msg.GetBuffer(), msg.GetSize(), 0)==-1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no se ha podido enviar el mensaje"));
	}
}

void PhyLayerServiceV1::ReceiveMsg(ServiceMessage & msg)
{
	if(mq_receive(rxmqid, (char*)msg.GetBuffer(), msg.GetMaxSize(), NULL)==-1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: fallo al intentar recibir algun mensaje"));
	}
}

DataLinkFramePtr PhyLayerServiceV1::GetNextFrame()
{
	bool empty = true;
	while(empty)
	{
		rxfifo_mutex.lock();
		empty = rxfifo.empty();
		rxfifo_mutex.unlock();
	}

	rxfifo_mutex.lock();
	DataLinkFramePtr dlf = rxfifo.front();
	rxfifo.pop();
	rxfifo_mutex.unlock();

	return dlf;
}

void PhyLayerServiceV1::PushNewFrame(DataLinkFramePtr dlf)
{
	rxfifo_mutex.lock();

	rxfifo.push(dlf);

	rxfifo_mutex.unlock();
}

IPhyLayerService & PhyLayerServiceV1::operator >> (DataLinkFramePtr & dlf)
{
	dlf = GetNextFrame();

	return *this;
}

void PhyLayerServiceV1::_SetPhyLayerState(const PhyState & state)
{
	phyState_mutex.lock();

	phyState = state;

	phyState_mutex.unlock();
}

PhyLayerServiceV1::PhyState PhyLayerServiceV1::_GetPhyLayerState()
{
	PhyState state;
	phyState_mutex.lock();

	state = phyState;

	phyState_mutex.unlock();

	return state;
}

void PhyLayerServiceV1::SendPhyLayerState(const PhyState & state)
{
	replymsg.BuildCmdStateMsg(state);
	SendMsg(replymsg);
}

bool PhyLayerServiceV1::BusyTransmitting()
{
	if(type != IPHY_TYPE_DLINK)
		ThrowPhyLayerException("Method call not allowed");
	return _GetPhyLayerState() == PhyState::BUSY;
}

void PhyLayerServiceV1::ReqPhyLayerState()
{
	if(type != IPHY_TYPE_DLINK)
		ThrowPhyLayerException("Method call not allowed");
	txmsg.BuildReqStateMsg();
	SendMsg(txmsg);
}

void PhyLayerServiceV1::SetPhyLayerState(const PhyState & state)
{
	if(type != IPHY_TYPE_PHY)
		ThrowPhyLayerException("Method call not allowed");
	_SetPhyLayerState(state);
	SendPhyLayerState();
}

unsigned int PhyLayerServiceV1::GetRxFifoSize()
{
	unsigned int size;
	rxfifo_mutex.lock();
	size = rxfifo.size();
	rxfifo_mutex.unlock();
	return size;
}

void PhyLayerServiceV1::Start()
{
	service.Start();
	if(type == IPHY_TYPE_PHY)
	{
		//Enviamos el estado actual a la capa de arriba
		SendPhyLayerState();
	}
	else
	{
		ReqPhyLayerState();
	}
}

void PhyLayerServiceV1::Stop()
{
	service.Stop();
}

PhyLayerServiceV1::ServiceMessage::ServiceMessage()
{
	buffer = NULL;
}

PhyLayerServiceV1::ServiceMessage::~ServiceMessage()
{
	free(buffer);
}

void PhyLayerServiceV1::ServiceMessage::Init(int maxs)
{
	maxPayloadSize = maxs - MSG_OVERHEAD_SIZE;
	maxSize = maxs;
	buffer = malloc(maxs);
	type = (uint8_t*) buffer;
	*type = (uint8_t) NOTBUILT;
	payload = type + 1;
	size = 0;
}

void PhyLayerServiceV1::ServiceMessage::BuildFrameMsg(const DataLinkFramePtr & dlf)
{
	int frsize = dlf->GetFrameSize();
	if(frsize <= maxPayloadSize)
	{

		memcpy(payload, dlf->GetFrameBuffer(), frsize);
		*type = (uint8_t) MsgType::FRAME;
		size = frsize + MSG_OVERHEAD_SIZE;
	}
	else
	{
		*type = (uint8_t) MsgType::NOTBUILT;
		ThrowPhyLayerException("Error interno: la trama no cabe en el formato de mensaje de la cola");
	}
}

void PhyLayerServiceV1::ServiceMessage::BuildReqStateMsg()
{
	*type = (uint8_t) MsgType::REQ_STATE;
	size = MSG_OVERHEAD_SIZE;
}

void PhyLayerServiceV1::ServiceMessage::BuildCmdStateMsg(const PhyState & state)
{
	*type = (uint8_t) MsgType::CMD_STATE;
	*payload = (uint8_t) state;
	size = MSG_OVERHEAD_SIZE + 1;
}

DataLinkFramePtr PhyLayerServiceV1::ServiceMessage::GetDataLinkFrame(const DataLinkFrame::fcsType & _fcsType) const
{
	auto dlf = DataLinkFrame::BuildDataLinkFrame(_fcsType);
	dlf->GetInfoFromBufferWithPreamble(payload);
	return dlf;
}

PhyLayerServiceV1::ServiceThread::ServiceThread(PhyLayerServiceV1 * parent)
{
	mcontinue= true;
	terminated = false;
	started = false;
	physervice = parent;
}

PhyLayerServiceV1::ServiceThread::~ServiceThread()
{
	this->Stop();
}

void PhyLayerServiceV1::ServiceThread::Start()
{
	thread = std::thread(&ServiceThread::Work, this);
	started = true;
}

void PhyLayerServiceV1::ServiceThread::Stop()
{
	mcontinue = false;
}

bool PhyLayerServiceV1::ServiceThread::IsRunning()
{
	return started && !terminated;
}

void PhyLayerServiceV1::SaveFrameFromMsg(const ServiceMessage & msg)
{
	PushNewFrame(msg.GetDataLinkFrame(fcsType));
}

void PhyLayerServiceV1::SavePhyStateFromMsg(const ServiceMessage & msg)
{
	_SetPhyLayerState(msg.GetPhyState());
}

void PhyLayerServiceV1::SendPhyLayerState()
{
	SendPhyLayerState(_GetPhyLayerState());
}

void PhyLayerServiceV1::ServiceThread::Work()
{
	while(mcontinue)
	{
		LOG_DEBUG("Esperando mensaje...");
		physervice->ReceiveMsg(physervice->rxmsg);
		switch(physervice->rxmsg.GetMsgType())
		{
		case ServiceMessage::FRAME:
			LOG_DEBUG("Recibida trama desde la capa fisica");
			physervice->SaveFrameFromMsg(physervice->rxmsg);
			break;
		case ServiceMessage::CMD_STATE:
			LOG_DEBUG("Recibido mensaje de estado de la capa fisica");
			physervice->SavePhyStateFromMsg(physervice->rxmsg);
			break;
		case ServiceMessage::REQ_STATE:
			LOG_DEBUG("Recibida peticion de estado de la capa fisica");
			physervice->SendPhyLayerState();
			break;
		default:
			break;
		}
		//std::this_thread::sleep_for(std::chrono::seconds(2));
	}
	LOG_DEBUG("Terminando...");
	terminated = true;
}


} /* namespace radiotransmission */
