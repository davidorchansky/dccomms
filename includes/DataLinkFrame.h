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
#include <Stream.h>

namespace radiotransmission {

#define DLNK_PREAMBLE_SIZE 10
#define DLNK_DIR_SIZE   1
#define DLNK_DSIZE_SIZE 2

class DataLinkFrame {
public:
	enum fcsType { crc16, crc32, nofcs };
	DataLinkFrame(fcsType fcst);
	DataLinkFrame(
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

	void getInfoFromBuffer(void *);

	friend Stream& operator >> (Stream & i, DataLinkFrame & dlf);
	friend Stream& operator << (Stream & i, const DataLinkFrame & dlf);

	int frameSize = 0;

	uint16_t dataSize = 0;
	uint8_t* payload = NULL;

	void printFrame(std::ostream &);
	bool checkFrame();

	static bool IsBigEndian();
	static const unsigned char * manchesterPre;
private:
	uint8_t * pre,* ddir,*sdir,* fcs;
	uint16_t * dsize;

	uint8_t *buffer = NULL;
	uint16_t overheadSize = 0;

	fcsType fcstype;
	uint32_t fcsSize = 0;


	uint8_t totalInfoSize = 0;

	void _calculateCRC();

	void _deletePayloadBuffer();
	bool _BigEndian;
	bool _canDeletePayload  = false;


};

} /* namespace radiotransmission */

#endif /* DATALINKFRAME_H_ */
