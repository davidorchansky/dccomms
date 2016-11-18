/*
 * DeviceInterface.h
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#ifndef INCLUDES_ICOMMSDEVICE_H_
#define INCLUDES_ICOMMSDEVICE_H_

#include <dccomms/ICommsLink.h>
#include <dccomms/IStream.h>

namespace dccomms {

class ICommsDevice : public IStream, public ICommsLink{
public:
	ICommsDevice();
	virtual ~ICommsDevice();

	virtual ICommsLink& operator >> (DataLinkFramePtr & dlf);
	virtual ICommsLink& operator<< (const DataLinkFramePtr & dlf);

	virtual ICommsDevice & operator << (const char * str);
	virtual ICommsDevice & operator << (const std::string &);

	virtual bool Open();
	virtual void Close();
};

} /* namespace dccomms */

#endif /* INCLUDES_ICOMMSDEVICE_H_ */
