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
#include <IStream.h>
#include <cstring>
#include <boost/shared_ptr.hpp>

namespace dccomms {

#define DLNK_PREAMBLE_SIZE 10
#define DLNK_DIR_SIZE   1
#define DLNK_DSIZE_SIZE 2
#define DLNK_MAX_PAYLOAD_SIZE 8000

class DataLinkFrame;

typedef boost::shared_ptr<DataLinkFrame> DataLinkFramePtr;

class DataLinkFrame {
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

	uint8_t getDesDir(){return *ddir;}
	uint8_t getSrcDir(){return *sdir;}
	uint16_t getDataSize(){return *dsize;}
	int GetFrameSize() const {return frameSize;}
	uint8_t * GetFrameBuffer() const {return buffer;}

	void GetInfoFromBuffer(void *);
	void GetInfoFromBufferWithPreamble(void *o);

	friend IStream& operator >> (IStream & i, DataLinkFramePtr & dlf);
	friend IStream& operator << (IStream & i, const DataLinkFramePtr & dlf);

	uint8_t* GetFrameBits(void *dst);

	int frameSize = 0;

	uint16_t dataSize = 0;
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

	bool _BigEndian;
	bool dataIn = false;
};


} /* namespace radiotransmission */

#endif /* DATALINKFRAME_H_ */
