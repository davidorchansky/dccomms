/*
 * IPhyLayer.cpp
 *
 *  Created on: 15 set. 2016
 *      Author: diego
 */

#include <IPhyLayer.h>

namespace radiotransmission {

IPhyLayer::IPhyLayer() {
	// TODO Auto-generated constructor stub

}

IPhyLayer::~IPhyLayer() {
	// TODO Auto-generated destructor stub
}

unsigned long IPhyLayer::GetTimeout()
{
	return _timeout;
}

void IPhyLayer::SetTimeout(unsigned long ms)
{
	_timeout = ms;
}

} /* namespace radiotransmission */
