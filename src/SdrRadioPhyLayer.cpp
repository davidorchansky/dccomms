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

SdrRadioPhyLayer::SdrRadioPhyLayer() {
	txmqname = "/tmp/radio/msg/txsdrvideo";
	rxmqname = "/tmp/radio/msg/rxsdrvideo";

	txmqid = mq_open(txmqname.c_str(), O_CREAT, O_RDWR, NULL);
	if(txmqid == -1)
	{
		throw RadioException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola para envio de mensajes"), RADIO_TXLINEDOWN);
	}
	rxmqid = mq_open(rxmqname.c_str(), O_CREAT, O_RDWR, NULL);
	if(rxmqid == -1)
	{
		throw RadioException(std::string("Error(")+std::to_string(errno)+std::string("): Error al abrir/crear la cola para recepcion de mensajes"), RADIO_RXLINEDOWN);
	}
#ifdef DEBUG
	std::cerr << "TXMQ:" << std::endl;
	ShowMQAttr(std::cerr, TX_MQ);
	std::cerr << "RXMQ:" << std::endl;
	ShowMQAttr(std::cerr, RX_MQ);
#endif
}

SdrRadioPhyLayer::~SdrRadioPhyLayer() {
	// TODO Auto-generated destructor stub
	mq_close(rxmqid);
	mq_close(txmqid);
}

void SdrRadioPhyLayer::UpdateMQAttr ()
{
	if(mq_getattr(txmqid, &txattr)==-1)
		throw RadioException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible obtener los atributos de la cola de mensajes tx"), RADIO_TXLINEDOWN);
	if(mq_getattr(rxmqid, &rxattr)==-1)
		throw RadioException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible obtener los atributos de la cola de mensajes rx"), RADIO_RXLINEDOWN);
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
	default:
		throw RadioException(std::string("Error interno: la cola de mensajes no existe"), RADIO_ERROR);
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
	default:
		throw RadioException(std::string("Error interno: la cola de mensajes no existe"), RADIO_ERROR);
	}
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
		throw RadioException(std::string("Error(")+std::to_string(errno)+std::string("): Error interno: no ha sido posible establecer los atributos de la cola de mensajes"), RADIO_ERROR);
	}
}

IPhyLayer & SdrRadioPhyLayer::operator << (const DataLinkFrame & dlf)
{

}

IPhyLayer & SdrRadioPhyLayer::operator >> (DataLinkFrame & dlf)
{

}

bool SdrRadioPhyLayer::BusyTransmitting()
{

}

} /* namespace radiotransmission */
