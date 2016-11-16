/*
 * DataLinkFrame.h
 *
 *  Created on: Feb 12, 2015
 *      Author: diego
 */

#ifndef DATALINKFRAME_H_
#define DATALINKFRAME_H_


#include <DataLinkFrame.h>
#include <ostream>
#include <cstring>
#include <boost/shared_ptr.hpp>
#include <CommsException.h>

namespace dccomms {

#define DLNK_PREAMBLE_SIZE 10
#define DLNK_DIR_SIZE   1
#define DLNK_DSIZE_SIZE 2
#define DLNK_MAX_PAYLOAD_SIZE 8000



class DataLinkFrame;

typedef boost::shared_ptr<DataLinkFrame> DataLinkFramePtr;

class DataLinkFrame{
	friend class ICommsDevice;
public:
	enum fcsType { crc16, crc32, nofcs };

	static DataLinkFramePtr BuildDataLinkFrame(fcsType fcst);
	static DataLinkFramePtr BuildDataLinkFrame(
			uint8_t, //destination dir
			uint8_t, //source dir
			uint16_t, //data size
			uint8_t *, //data
			fcsType //fcstype
			);

	virtual ~DataLinkFrame();

	inline uint8_t GetDesDir(){return *ddir;}
	inline uint8_t GetSrcDir(){return *sdir;}
	inline int GetFrameSize() const {return frameSize;}
	inline uint8_t * GetFrameBuffer() const {return buffer;}
	inline fcsType GetFcsType() const {return fcstype;}

	void SetDesDir(uint8_t _ddir);
	void SetSrcDir(uint8_t _sdir);

	void UpdateFrame(
			uint8_t, //destination dir
			uint8_t, //source dir
			uint16_t, //data size
			uint8_t * //data
			);

	void PayloadUpdated(unsigned int datasize);
	inline uint8_t * GetPayloadBuffer();

	void GetInfoFromBuffer(void *);
	void GetInfoFromBufferWithPreamble(void *o);

	uint8_t* GetFrameBits(void *dst);

	int frameSize = 0;

	uint16_t payloadSize = 0;
	uint8_t* payload = NULL;

	void printFrame(std::ostream &);
	bool checkFrame();

	static bool IsBigEndian();
	static const unsigned char * manchesterPre;
private:
	DataLinkFrame(fcsType fcst);
	DataLinkFrame(
			uint8_t, //destination dir
			uint8_t, //source dir
			uint16_t, //data size
			uint8_t *, //data
			fcsType //fcstype
			);

	void Init(DataLinkFrame::fcsType fcst);

	uint8_t * pre,* ddir,*sdir,* fcs;
	uint16_t * dsize;

	uint8_t *buffer = NULL;
	uint16_t overheadSize = 0;

	fcsType fcstype;
	uint32_t fcsSize = 0;


	uint8_t totalInfoSize = 0;

	void _calculateCRC();
	void _SetPayloadSizeInBuffer(unsigned int datasize);
	bool _BigEndian;
	bool dataIn = false;

};

} /* namespace radiotransmission */

#endif /* DATALINKFRAME_H_ */
