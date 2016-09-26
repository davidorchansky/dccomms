/*
 * DataLinkFrame.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: diego
 */

#include <DataLinkFrame.h>
#include <RadioException.h>
#include <cstdlib>
#include <cstring>
#include <Checksum.h>
#include <Stream.h>

namespace radiotransmission {

static const unsigned char _manchesterPre[DLNK_PREAMBLE_SIZE] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};

const unsigned char* DataLinkFrame::manchesterPre = _manchesterPre;

static void ThrowDLinkLayerException(std::string msg)
{
	throw RadioException("DLINK EXCEPTION: "+msg, RADIO_DLNKLAYER_ERROR);
}

void DataLinkFrame::Init(DataLinkFrame::fcsType fcst)
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
	buffer = new uint8_t[overheadSize + DLNK_MAX_PAYLOAD_SIZE];
    pre   = buffer;
    ddir  = pre    + DLNK_PREAMBLE_SIZE;
    sdir  = ddir   + DLNK_DIR_SIZE;
    dsize = (uint16_t *) (sdir   + DLNK_DIR_SIZE);
    payload = ((uint8_t *) dsize)  + DLNK_DSIZE_SIZE;

    memcpy(pre, DataLinkFrame::manchesterPre, DLNK_PREAMBLE_SIZE);
    frameSize = overheadSize;
    totalInfoSize = DLNK_DIR_SIZE*2 + DLNK_DSIZE_SIZE;
    dataIn = false;
}

DataLinkFrame::DataLinkFrame(DataLinkFrame::fcsType fcst)
{
	Init(fcst);
}

DataLinkFrame::DataLinkFrame(
		uint8_t desdir,
		uint8_t srcdir,
		uint16_t datasize,
		uint8_t * data,
		fcsType fcst
		)
{
	Init(fcst);

	dataSize = datasize;

    *ddir = desdir;
    *sdir = srcdir;
    if(_BigEndian)
    	*dsize = datasize;
    else
    {
    	*(uint8_t*)dsize = (uint8_t)(datasize >> 8);
    	*(((uint8_t*)dsize)+1) = (uint8_t)(datasize & 0xff);
    }
    if(datasize > DLNK_MAX_PAYLOAD_SIZE)
    {
    	ThrowDLinkLayerException(std::string("El tamano del payload no puede ser mayor que ")+ std::to_string(DLNK_MAX_PAYLOAD_SIZE));
    }
    fcs = ((uint8_t *) payload) + dataSize;
    memcpy(payload, data, datasize);

    frameSize = overheadSize + dataSize;

    _calculateCRC();
    dataIn = true;
}

static uint8_t * getBits(void * data, int length, void * bits)
{

		unsigned char * sptr, * dptr, *maxsptr;

		for(sptr = (unsigned char*)data, dptr = (unsigned char*)bits, maxsptr = sptr + length; sptr < maxsptr; sptr++)
		{
			*dptr++ = *sptr & 1;
			*dptr++ = (*sptr >> 1) & 1;
			*dptr++ = (*sptr >> 2) & 1;
			*dptr++ = (*sptr >> 3) & 1;
			*dptr++ = (*sptr >> 4) & 1;
			*dptr++ = (*sptr >> 5) & 1;
			*dptr++ = (*sptr >> 6) & 1;
			*dptr++ = (*sptr >> 7) & 1;
		}
		return dptr;

}

uint8_t* DataLinkFrame::GetFrameBits(void * dst)
{
	uint8_t * ptr = (uint8_t*)dst;
	ptr = getBits(pre, frameSize, ptr);
	return ptr;
}

DataLinkFrame::~DataLinkFrame() {
	if(buffer != NULL)
		delete buffer;
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

Stream& operator >> (Stream & i, DataLinkFramePtr & dlf)
{
	i.WaitFor((const uint8_t*) dlf->pre, DLNK_PREAMBLE_SIZE);

	i.Read(dlf->ddir, DLNK_DIR_SIZE);
	i.Read(dlf->sdir, DLNK_DIR_SIZE);

	i.Read((uint8_t *)dlf->dsize, DLNK_DSIZE_SIZE);

	if(dlf->_BigEndian)
	{
		dlf->dataSize  = *dlf->dsize;
	}
	else
	{
		dlf->dataSize = ((*dlf->dsize) << 8) | ((*dlf->dsize) >> 8);
	}

    if(dlf->dataSize > DLNK_MAX_PAYLOAD_SIZE)
    {
    	ThrowDLinkLayerException(std::string("El tamano del payload no puede ser mayor que ")+ std::to_string(DLNK_MAX_PAYLOAD_SIZE));
    }

	i.Read(dlf->payload, dlf->dataSize);

	dlf->fcs = ((uint8_t *) dlf->payload) + dlf->dataSize;
	i.Read(dlf->fcs, dlf->fcsSize);

	dlf->frameSize = dlf->overheadSize + dlf->dataSize;

	dlf->dataIn = true;
	return i;
}
Stream& operator << (Stream & i, const DataLinkFramePtr & dlf)
{
	if(dlf->dataIn)
	{
		i.Write(dlf->pre,DLNK_PREAMBLE_SIZE);
		i.Write(dlf->ddir,DLNK_DIR_SIZE);
		i.Write(dlf->sdir,DLNK_DIR_SIZE);
		i.Write(dlf->dsize,DLNK_DSIZE_SIZE);
		i.Write(dlf->payload,dlf->dataSize);
		i.Write(dlf->fcs, dlf->fcsSize);

		i.FlushIO();//Lo ideal seria FlushOutput, pero en algun lado hay algo que hace que se llene el buffer de entrada
						  //y al final llega a bloquearse la comunicación... (TODO: comprobar qué es lo que hace que se llene el buffer de entrada)
	}
	return i;
}
void DataLinkFrame::GetInfoFromBufferWithPreamble(void *o)
{
	GetInfoFromBuffer(o + DLNK_PREAMBLE_SIZE);
}

void DataLinkFrame::GetInfoFromBuffer(void *o)
{
	uint8_t * optr = (uint8_t*)o;

	memcpy(this->ddir, optr, DLNK_DIR_SIZE);
	optr+=DLNK_DIR_SIZE;

	memcpy(this->sdir, optr, DLNK_DIR_SIZE);
	optr+=DLNK_DIR_SIZE;

	memcpy((uint8_t *)this->dsize, optr, DLNK_DSIZE_SIZE);
	optr+=DLNK_DSIZE_SIZE;

	if(this->_BigEndian)
	{
		this->dataSize  = *this->dsize;
	}
	else
	{
		this->dataSize = ((*this->dsize) << 8) | ((*this->dsize) >> 8);
	}

    if(this->dataSize > DLNK_MAX_PAYLOAD_SIZE)
    {
    	ThrowDLinkLayerException(std::string("El tamano del payload no puede ser mayor que ")+ std::to_string(DLNK_MAX_PAYLOAD_SIZE));
    }

	memcpy(this->payload, optr, this->dataSize);
	optr+=this->dataSize;

	fcs = ((uint8_t *) payload) + dataSize;
	memcpy(this->fcs, optr, this->fcsSize);
	optr+=this->fcsSize;


	this->frameSize = this->overheadSize + this->dataSize;
	dataIn = true;
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

	if(dataIn)
	{
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
	else
	{
		o << "No data in frame!" << std::endl;
	}

}

bool DataLinkFrame::IsBigEndian()
{
	uint32_t word = 0x1;
	uint8_t * byte = (uint8_t *)&word;
	return *byte != 0x1;

}

DataLinkFramePtr DataLinkFrame::BuildDataLinkFrame(fcsType fcst)
{
	return DataLinkFramePtr(new DataLinkFrame(fcst));
}

DataLinkFramePtr DataLinkFrame::BuildDataLinkFrame(
		uint8_t desdir,
		uint8_t srcdir,
		uint16_t datasize,
		uint8_t * data,
		fcsType fcst
		)
{
	return DataLinkFramePtr(new DataLinkFrame(
			desdir,
			srcdir,
			datasize,
			data,
			fcst
			));
}

} /* namespace radiotransmission */
