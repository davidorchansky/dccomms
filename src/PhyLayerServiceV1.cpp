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

namespace radiotransmission {

#define TX_MQ 0
#define RX_MQ 1
#define RTS_MQ 2
#define CTS_MQ 3

#define MSG_TYPE_FRAME  0
#define MSG_TYPE_ISBUSY 1
#define MSG_TYPE_ISBUSY_REPLY 2
#define PHY_STATE_BUSY 1
#define PHY_STATE_READY 0
#define MSG_OVERHEAD 1

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
PhyLayerServiceV1::PhyLayerServiceV1(int type, int maxframesize) {
	struct mq_attr attr;
	attr.mq_maxmsg = 10;
	attr.mq_msgsize = maxframesize+MSG_OVERHEAD;
	int perm = 0644;
	Init(type, attr, perm);
}

void PhyLayerServiceV1::Init(int _type, struct mq_attr attr, int perm)
{
	type = _type;
	switch(_type)
	{
	case IPHY_TYPE_DLINK:
		txmqname = "/txsdrvideo";
		rxmqname = "/rxsdrvideo";
		break;
	case IPHY_TYPE_PHY:
		rxmqname = "/txsdrvideo";
		txmqname = "/rxsdrvideo";
		break;
	default:
		ThrowPhyLayerException("Tipo de interfaz con la capa fisica incorrecto");
	}

	mq_unlink(txmqname.c_str());
	mq_unlink(rxmqname.c_str());

	txattr = attr;
	rxattr = attr;

	txmqid = mq_open(txmqname.c_str(), O_CREAT | O_WRONLY | O_EXCL, perm, &txattr);
	std::string emsg;
	if(txmqid == -1)
	{
		emsg = GetMQErrorMsg(errno);
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola para envio de mensajes: ") + emsg);
	}
	rxmqid = mq_open(rxmqname.c_str(), O_CREAT | O_RDONLY | O_EXCL, perm, &rxattr);
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

	rxbuffsize = GetMaxMsgSize(RX_MQ);
	rxbuff = (uint8_t*) malloc(rxbuffsize);
}

PhyLayerServiceV1::~PhyLayerServiceV1() {
	// TODO Auto-generated destructor stub
	mq_close(rxmqid);
	mq_close(txmqid);
	mq_unlink(txmqname.c_str());
	mq_unlink(rxmqname.c_str());
	if(rxbuff != NULL)
		free(rxbuff);
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

IPhyLayerService & PhyLayerServiceV1::operator << (const DataLinkFrame & dlf)
{
	uint8_t * fbuf = dlf.getFrameBuffer();

	int fsize = dlf.getFrameSize();
	int msize = fsize + MSG_OVERHEAD;

	uint8_t * mbuf, *data;
	mbuf = (uint8_t*) malloc(fsize + MSG_OVERHEAD);
    data = mbuf + MSG_OVERHEAD;

    memcpy(data, fbuf, fsize);

    *mbuf = MSG_TYPE_FRAME;

	long maxmsize = GetMaxMsgSize(TX_MQ);
	if (maxmsize >= msize)
	{
		if(mq_send(txmqid, (char*) mbuf, msize, 0)==-1)
		{
			free(mbuf);
			ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no se ha podido enviar el mensaje"));
		}
		free(mbuf);
		return *this;
	}
	else
	{
		free(mbuf);
		ThrowPhyLayerException("Error interno: la trama no cabe en el formato de mensaje de la cola");
	}

	return *this; //nunca llegara aqui

}

IPhyLayerService & PhyLayerServiceV1::operator >> (DataLinkFrame & dlf)
{
	if(rxfifo.size()>0)
	{
		dlf = rxfifo.front();
		rxfifo.pop();
	}
	return *this;
}

bool PhyLayerServiceV1::BusyTransmitting()
{
	return GetPhyLayerState() == PHY_STATE_BUSY;
}



} /* namespace radiotransmission */
