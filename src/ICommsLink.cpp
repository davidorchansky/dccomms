/*
 * IPhyLayer.cpp
 *
 *  Created on: 15 set. 2016
 *      Author: diego
 */

#include <ICommsLink.h>

namespace dccomms {

ICommsLink::ICommsLink() {
	// TODO Auto-generated constructor stub

}

ICommsLink::~ICommsLink() {
	// TODO Auto-generated destructor stub
}

unsigned long ICommsLink::GetTimeout()
{
	return _timeout;
}

void ICommsLink::SetTimeout(unsigned long ms)
{
	_timeout = ms;
}

bool ICommsLink::BusyTransmitting()
{
	return false;
}

} /* namespace radiotransmission */
