#include <dccomms/TransportPDU.h>
#include <cstring>

namespace dccomms {

int TransportPDU::OverheadSize = 1;

TransportPDU::TransportPDU(int msize, uint8_t * buffer, bool copybuf):
    IPDU(msize, buffer, copybuf)
{
    _Init();
}

TransportPDU::TransportPDU(int msize):
    IPDU(msize)
{
    _Init();
}

void TransportPDU::_Init()
{
    _InitPointers ();
    SetSeqNum(0);
}

void TransportPDU::_InitPointers()
{
    _nseq = _buffer;

    _payload = _buffer + OverheadSize;
}

uint8_t TransportPDU::GetSeqNum ()
{
    return *_nseq;
}

void TransportPDU::SetSeqNum (uint8_t seq)
{
   *_nseq = seq;
}

void TransportPDU::IncSeqNum ()
{
    *_nseq = *_nseq + 1;
}
uint8_t * TransportPDU::GetPayloadBuffer ()
{
   return _payload;
}

void TransportPDU::UpdateBuffer (uint8_t *newBuffer)
{
    _buffer = newBuffer;
    _InitPointers ();
}

uint8_t * TransportPDU::GetPayloadBuffer (int &size, bool copy)
{
    uint8_t * res;
    if(!copy)
        res = _payload;
    else
    {
        res = new uint8_t[_bufferSize];
        memcpy(res, _payload, _payloadSize);
    }
    size = _payloadSize;
    return res;
}

TransportPDUPtr TransportPDU::BuildTransportPDU(int msize,
                                          uint8_t * buffer,
                                          bool copybuf
        )
{
    return TransportPDUPtr(
                new TransportPDU(msize, buffer, copybuf)
                );
}

TransportPDUPtr TransportPDU::BuildTransportPDU(int msize
        )
{
    return TransportPDUPtr(
                new TransportPDU(msize)
                );
}


}
