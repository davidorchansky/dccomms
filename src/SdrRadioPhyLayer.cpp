/*
 * SdrRadioPhyLayer.cpp
 *
 *  Created on: 16 set. 2016
 *      Author: diego
 */

#include <SdrRadioPhyLayer.h>
#include <errno.h>
#include <RadioException.h>

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

}

SdrRadioPhyLayer::~SdrRadioPhyLayer() {
	// TODO Auto-generated destructor stub
	mq_close(rxmqid);
	mq_close(txmqid);
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
