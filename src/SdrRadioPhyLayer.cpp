/*
 * SdrRadioPhyLayer.cpp
 *
 *  Created on: 16 set. 2016
 *      Author: diego
 */

#include <SdrRadioPhyLayer.h>
#include <errno.h>
#include <RadioException.h>
#include <iostream>

namespace radiotransmission {

void ThrowPhyLayerException(std::string msg)
{
	throw RadioException("PHYLAYER EXCEPTION: "+msg, RADIO_PHYLAYER_ERROR);
}

SdrRadioPhyLayer::SdrRadioPhyLayer() {
	txmqname = "/tmp/radio/msg/txsdrvideo";
	rxmqname = "/tmp/radio/msg/rxsdrvideo";
	rtsmqname = "/tmp/radio/msg/rtssdrvideo";
	ctsmqname = "/tmp/radio/msg/ctssdrvideo";

	txmqid = mq_open(txmqname.c_str(), O_CREAT, O_RDWR, NULL);
	if(txmqid == -1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola para envio de mensajes"));
	}
	rxmqid = mq_open(rxmqname.c_str(), O_CREAT, O_RDWR, NULL);
	if(rxmqid == -1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola para recepcion de mensajes"));
	}
	rtsmqid = mq_open(rtsmqname.c_str(), O_CREAT, O_RDWR, NULL);
	if(rtsmqid == -1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola rts"));
	}
	ctsmqid = mq_open(ctsmqname.c_str(), O_CREAT, O_RDWR, NULL);
	if(ctsmqid == -1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola cts"));
	}

	SetNonblockFlag(false, txmqid);
	SetNonblockFlag(false, rxmqid);
	SetNonblockFlag(false, rtsmqid);
	SetNonblockFlag(false, ctsmqid);

#ifdef DEBUG
	std::cerr << "TXMQ:" << std::endl;
	ShowMQAttr(std::cerr, TX_MQ);
	std::cerr << "RXMQ:" << std::endl;
	ShowMQAttr(std::cerr, RX_MQ);
	std::cerr << "RTSMQ:" << std::endl;
	ShowMQAttr(std::cerr, RTS_MQ);
	std::cerr << "CTSMQ:" << std::endl;
	ShowMQAttr(std::cerr, CTS_MQ);
#endif

	rxbuffsize = GetMaxMsgSize(RX_MQ);
	rxbuff = (uint8_t*) malloc(rxbuffsize);
}

SdrRadioPhyLayer::~SdrRadioPhyLayer() {
	// TODO Auto-generated destructor stub
	mq_close(rxmqid);
	mq_close(txmqid);
	mq_close(ctsmqid);
	mq_close(rtsmqid);
	if(rxbuff != NULL)
		free(rxbuff);
}

void SdrRadioPhyLayer::UpdateMQAttr ()
{
	if(mq_getattr(txmqid, &txattr)==-1)
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible obtener los atributos de la cola de mensajes tx"));
	if(mq_getattr(rxmqid, &rxattr)==-1)
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible obtener los atributos de la cola de mensajes rx"));
	if(mq_getattr(rtsmqid, &rtsattr)==-1)
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible obtener los atributos de la cola de mensajes rts"));
	if(mq_getattr(ctsmqid, &ctsattr)==-1)
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible obtener los atributos de la cola de mensajes cts"));
}

struct mq_attr* SdrRadioPhyLayer::GetMQAttr(int mq)
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
	case RTS_MQ:
		attr = &rtsattr;
		break;
	case CTS_MQ:
		attr = &ctsattr;
		break;
	default:
		ThrowPhyLayerException("Error interno: la cola de mensajes no existe");
	}
	return attr;
}

mqd_t SdrRadioPhyLayer::GetMQId(int mq)
{
	switch(mq)
	{
	case TX_MQ:
		return txmqid;
	case RX_MQ:
		return rxmqid;
	case RTS_MQ:
		return rtsmqid;
	case CTS_MQ:
		return ctsmqid;
	default:
		ThrowPhyLayerException("Error interno: la cola de mensajes no existe");
	}
	return 0; //nunca llegara aqui
}

void SdrRadioPhyLayer::ShowMQAttr(std::ostream & o, int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);

	o << " - Maximum # of messages on queue:\t"<< attr->mq_maxmsg << std::endl;
	o << " - Maximum message size:\t"<< attr->mq_msgsize << std::endl;
	o << " - # of messages currently on queue:\t"<< attr->mq_curmsgs << std::endl;
	o << " - O_NONBLOCK:\t" << (attr->mq_flags & O_NONBLOCK ? "activado" : "desactivado") << std::endl;
}

long SdrRadioPhyLayer::GetMaxMsgOnQueue(int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	return attr->mq_maxmsg;
}

long SdrRadioPhyLayer::GetMaxMsgSize(int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	return attr->mq_msgsize;
}

long SdrRadioPhyLayer::GetNumMsgOnQueue(int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	return attr->mq_curmsgs;
}

bool SdrRadioPhyLayer::GetNonblockFlag(int mq)
{
	struct mq_attr * attr = GetMQAttr(mq);
	return attr->mq_flags & O_NONBLOCK;
}

void SdrRadioPhyLayer::SetNonblockFlag(bool v, int mq)
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

IPhyLayer & SdrRadioPhyLayer::operator << (const DataLinkFrame & dlf)
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
		free(mbuf);
		if(mq_send(txmqid, (char*) mbuf, msize, 0)==-1)
		{
			ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no se ha podido enviar el mensaje"));
		}
		return *this;
	}
	else
	{
		free(mbuf);
		ThrowPhyLayerException("Error interno: la trama no cabe en el formato de mensaje de la cola");
	}

	return *this; //nunca llegara aqui

}

IPhyLayer & SdrRadioPhyLayer::operator >> (DataLinkFrame & dlf)
{
	//TODO: falta implementacion. De momento solo utilizaremos esta clase en la parte del transmisor
	return *this;
}

bool SdrRadioPhyLayer::BusyTransmitting()
{
	uint8_t msg[2] = { MSG_TYPE_ISBUSY, 0 };

	if(mq_send(rtsmqid, (char*) msg, 2, 0)==-1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: error al pedir confirmacion de disponibilidad"));
	}
	if(mq_receive(ctsmqid, (char*)msg, 2, NULL)==-1)
	{
		ThrowPhyLayerException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no se ha recibido una contestacion al mensaje de disponibilidad"));
	}
	if(msg[0] != MSG_TYPE_ISBUSY_REPLY)
	{
		ThrowPhyLayerException("Error interno: el codigo del mensaje no es correcto (error grave)");
	}
	return msg[1] == 1;

}

} /* namespace radiotransmission */
