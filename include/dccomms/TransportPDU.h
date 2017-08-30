#ifndef TRANSPORTPDU_H
#define TRANSPORTPDU_H
#include <boost/shared_ptr.hpp>

#include <dccomms/Packet.h>

namespace dccomms {

class TransportPDU;

typedef boost::shared_ptr<TransportPDU> TransportPDUPtr;

class TransportPDU : Packet
{
public:
    static int OverheadSize;
    static TransportPDUPtr BuildTransportPDU(uint32_t msize,
                                              uint8_t * buffer,
                                              bool copybuf = false
            );
    static TransportPDUPtr BuildTransportPDU(uint32_t msize
            );

    TransportPDU(uint32_t msize, uint8_t * buffer, bool copybuf = false);
    TransportPDU(uint32_t msize);
    uint8_t GetSeqNum();
    void SetSeqNum(uint8_t seq);
    void IncSeqNum();
    void UpdateBuffer(uint8_t * newBuffer);
    uint8_t * GetPayloadBuffer();
    uint32_t GetPayloadSize();

private:
    uint8_t * _nseq,
            * _payload;

    uint32_t _payloadSize;

    void _Init();
    void _InitPointers();
};

}

#endif // TRANSPORTPDU_H
