#include <dccomms/TransportPDU.h>
#include <cstring>

namespace dccomms {

int TransportPDU::OverheadSize = 1;

TransportPDU::TransportPDU(uint32_t msize, uint8_t * buffer, bool copybuf):
    Packet(msize, buffer, copybuf)
{
    _Init();
}

TransportPDU::TransportPDU(uint32_t msize):
    Packet(msize)
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

uint32_t TransportPDU::GetPayloadSize ()
{
   return _payloadSize;
}

void TransportPDU::UpdateBuffer (uint8_t *newBuffer)
{
    _buffer = newBuffer;
    _InitPointers ();
}

TransportPDUPtr TransportPDU::BuildTransportPDU(uint32_t msize,
                                          uint8_t * buffer,
                                          bool copybuf
        )
{
    return TransportPDUPtr(
                new TransportPDU(msize, buffer, copybuf)
                );
}

TransportPDUPtr TransportPDU::BuildTransportPDU(uint32_t msize
        )
{
    return TransportPDUPtr(
                new TransportPDU(msize)
                );
}


}
