/*
 * IPhyLayer.cpp
 *
 *  Created on: 15 set. 2016
 *      Author: diego
 */

#include <IPhyLayerService.h>

namespace dccomms {

IPhyLayerService::IPhyLayerService() {
	// TODO Auto-generated constructor stub

}

IPhyLayerService::~IPhyLayerService() {
	// TODO Auto-generated destructor stub
}

unsigned long IPhyLayerService::GetTimeout()
{
	return _timeout;
}

void IPhyLayerService::SetTimeout(unsigned long ms)
{
	_timeout = ms;
}

} /* namespace radiotransmission */
