/*
 * DeviceInterface.h
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#ifndef INCLUDES_ICOMMSDEVICE_H_
#define INCLUDES_ICOMMSDEVICE_H_

#include <dccomms/ICommsDevice.h>
#include <dccomms/IStream.h>

namespace dccomms {

class IStreamCommsDevice : public IStream, public ICommsDevice{
public:
        IStreamCommsDevice();
        virtual ~IStreamCommsDevice();

	virtual ICommsLink& operator >> (DataLinkFramePtr & dlf);
	virtual ICommsLink& operator<< (const DataLinkFramePtr & dlf);

        virtual IStreamCommsDevice & operator << (const char * str);
        virtual IStreamCommsDevice & operator << (const std::string &);

	virtual bool Open();
	virtual void Close();
};

} /* namespace dccomms */

#endif /* INCLUDES_ICOMMSDEVICE_H_ */
