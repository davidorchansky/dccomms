/*
 * DataLinkFrame.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: diego
 */

#include <DataLinkFrame.h>
#include <cstdlib>
#include <cstring>
#include <Checksum.h>
#include <Stream.h>

namespace radiotransmission {


DataLinkFrame::DataLinkFrame(DataLinkFrame::fcsType fcst)
{
	overheadSize = DLNK_PREAMBLE_SIZE +
				   DLNK_DIR_SIZE * 2  +
				   DLNK_DSIZE_SIZE;

	_BigEndian = DataLinkFrame::IsBigEndian();
	fcstype = fcst;
	switch(fcstype)
	{
	case crc16:
		fcsSize = 2;
		break;

	case crc32:
		fcsSize = 4;
		break;

	case nofcs:
		fcsSize = 0;
		break;
	}

	overheadSize += fcsSize;
	buffer = (uint8_t*) malloc(overheadSize);

    pre   = buffer;
    ddir  = pre    + DLNK_PREAMBLE_SIZE;
    sdir  = ddir   + DLNK_DIR_SIZE;
    dsize = (uint16_t *) (sdir   + DLNK_DIR_SIZE);
    fcs   = ((uint8_t *) dsize)  + DLNK_DSIZE_SIZE;

    memcpy(pre, "juanito", DLNK_PREAMBLE_SIZE);
    frameSize = overheadSize;
    totalInfoSize = DLNK_DIR_SIZE*2 + DLNK_DSIZE_SIZE;
}

DataLinkFrame::DataLinkFrame(
		uint8_t desdir,
		uint8_t srcdir,
		uint16_t datasize,
		uint8_t * data,
		fcsType fcst
		)
{
	_BigEndian = DataLinkFrame::IsBigEndian();

	overheadSize = DLNK_PREAMBLE_SIZE +
				   DLNK_DIR_SIZE * 2  +
				   DLNK_DSIZE_SIZE;

	fcstype = fcst;
	switch(fcstype)
	{
	case crc16:
		fcsSize = 2;
		break;

	case crc32:
		fcsSize = 4;
		break;

	case nofcs:
		fcsSize = 0;
		break;
	}
	overheadSize += fcsSize;
	buffer = (uint8_t*) malloc(overheadSize);

	dataSize = datasize;

    pre   = buffer;
    ddir  = pre    + DLNK_PREAMBLE_SIZE;
    sdir  = ddir   + DLNK_DIR_SIZE;
    dsize = (uint16_t *) (sdir   + DLNK_DIR_SIZE);
    fcs   = ((uint8_t *) dsize)  + DLNK_DSIZE_SIZE;

    memcpy(pre, "juanito", DLNK_PREAMBLE_SIZE);
    *ddir = desdir;
    *sdir = srcdir;
    if(_BigEndian)
    	*dsize = datasize;
    else
    {
    	*(uint8_t*)dsize = (uint8_t)(datasize >> 8);
    	*(((uint8_t*)dsize)+1) = (uint8_t)(datasize & 0xff);
    }
    payload = data;

    frameSize = overheadSize + dataSize;
    totalInfoSize = DLNK_DIR_SIZE*2 + DLNK_DSIZE_SIZE;

    _calculateCRC();
}


DataLinkFrame::~DataLinkFrame() {
	if(buffer != NULL)
		delete buffer;
	if(payload != NULL and _canDeletePayload)
		delete payload;
}

void DataLinkFrame::_calculateCRC()
{
	switch(fcstype)
	{
	case crc16:
		uint16_t crc1;
		crc1 = Checksum::crc16(ddir, totalInfoSize);
		crc1 = Checksum::crc16(payload, dataSize, crc1);

		*fcs = (uint8_t)(crc1 >> 8);
		*(fcs+1) = (uint8_t)(crc1 & 0x00ff);
		break;

	case crc32:
		uint32_t crc2;
		crc2 = Checksum::crc32(ddir, totalInfoSize);
		crc2 = Checksum::crc32(payload, dataSize, crc2);

        *(fcs)    = (uint8_t)((crc2 >> 24) & 0x000000ff);
        *(fcs+1)  = (uint8_t)((crc2 >> 16) & 0x000000ff);
        *(fcs+2)  = (uint8_t)((crc2 >> 8) & 0x000000ff);
        *(fcs+3)  = (uint8_t)(crc2 & 0x000000ff);
        break;
        //*(uint32_t*)fcs = crc2;
	case nofcs:
		break;


		break;
	}
}

bool DataLinkFrame::checkFrame()
{
	switch(fcstype)
	{
	case crc16:
		uint16_t crc1;
		crc1 = Checksum::crc16(ddir, totalInfoSize);
		crc1 = Checksum::crc16(payload, dataSize, crc1);
		crc1 = Checksum::crc16(fcs, fcsSize, crc1);
		return crc1 == 0;
		break;

	case crc32:
		uint32_t crc2;
		crc2 = Checksum::crc32(ddir, totalInfoSize);
		crc2 = Checksum::crc32(payload, dataSize, crc2);
		crc2 = Checksum::crc32(fcs, fcsSize, crc2);
		return crc2 == 0;
		break;

	case nofcs:
		return true;
		break;


	}
	return true;
}

void DataLinkFrame::_deletePayloadBuffer()
{
	if (payload != NULL and _canDeletePayload)
	{
		delete payload;
		payload = NULL;
	}
}

Stream& operator >> (Stream & i, DataLinkFrame & dlf)
{
	dlf._canDeletePayload = true;
	dlf._deletePayloadBuffer();
	i.WaitFor((const uint8_t*) "juanito", 7);

	i.Read(dlf.ddir, DLNK_DIR_SIZE);
	i.Read(dlf.sdir, DLNK_DIR_SIZE);

	i.Read((uint8_t *)dlf.dsize, DLNK_DSIZE_SIZE);

	if(dlf._BigEndian)
	{
		dlf.dataSize  = *dlf.dsize;
	}
	else
	{
		dlf.dataSize = ((*dlf.dsize) << 8) | ((*dlf.dsize) >> 8);
	}

	dlf.payload = (uint8_t*) malloc(dlf.dataSize);
	i.Read(dlf.payload, dlf.dataSize);

	i.Read(dlf.fcs, dlf.fcsSize);

	dlf.frameSize = dlf.overheadSize + dlf.dataSize;
	return i;
}
Stream& operator << (Stream & i, const DataLinkFrame & dlf)
{

	i.Write(dlf.pre,DLNK_PREAMBLE_SIZE);
	i.Write(dlf.ddir,DLNK_DIR_SIZE);
	i.Write(dlf.sdir,DLNK_DIR_SIZE);
	i.Write(dlf.dsize,DLNK_DSIZE_SIZE);
	i.Write(dlf.payload,dlf.dataSize);
	i.Write(dlf.fcs, dlf.fcsSize);

	return i;
}

void DataLinkFrame::printFrame(std::ostream & o)
{
	o << std::hex;
	o << "Preamble: 0x";

	uint8_t * p = pre;
	for(int i = 0; i < DLNK_PREAMBLE_SIZE; i++)
	{
		o.width(2);
		o.fill('0');
		o << (int)*p;
		p++;
	}
	o << std::endl;
	o << "Dest. Dir: 0x";
	p = ddir;
	for(int i = 0; i < DLNK_DIR_SIZE; i++)
	{
		o.width(2);
		o.fill('0');
		o << (uint32_t)*p;
		p++;
	}
	o << std::endl;
	o << "Source. Dir: 0x";
	p = sdir;
	for(int i = 0; i < DLNK_DIR_SIZE; i++)
	{
		o.width(2);
		o.fill('0');
		o << (uint32_t)*p;
		p++;
	}

	o << std::endl;
	o << "Data size: 0x";
	p = (uint8_t*) dsize;
	for(int i = 0; i < DLNK_DSIZE_SIZE; i++)
	{
		o.width(2);
		o.fill('0');
		o << (int)*p;
		p++;
	}

	uint16_t psize;
	if(_BigEndian)
	{
		psize = *dsize;
	}
	else
	{
		psize = ((*dsize) << 8) | ((*dsize) >> 8);
	}
	o << std::endl << std::dec << "Data (";
	o << psize << " bytes): 0x" << std::hex;
	p = payload;
	for(int i = 0; i < psize; i++)
	{
		o.width(2);
		o.fill('0');
		o << (int)*p;
		p++;
	}

	o << std::endl << "FCS: 0x";
	p = fcs;
	for(int i = 0; i < fcsSize; i++)
	{
		o.width(2);
		o.fill('0');
		o << (int)*p;
		p++;
	}
	o << " (0x"; o.width(fcsSize*2);
	if(fcsSize == 2)
		o << *(uint16_t*)fcs << ")" <<std::endl;
	else if(fcsSize == 4)
		o << *(uint32_t*)fcs << ")" <<std::endl;

	o << std::dec;
}

bool DataLinkFrame::IsBigEndian()
{
	uint32_t word = 0x1;
	uint8_t * byte = (uint8_t *)&word;
	return *byte != 0x1;

}

} /* namespace radiotransmission */