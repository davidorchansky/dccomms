#ifndef TRANSPORTPDU_H
#define TRANSPORTPDU_H
#include <boost/shared_ptr.hpp>

#include <dccomms/IPDU.h>

namespace dccomms {

class TransportPDU;

typedef boost::shared_ptr<TransportPDU> TransportPDUPtr;

class TransportPDU : IPDU
{
public:
    static int OverheadSize;
    static TransportPDUPtr BuildTransportPDU(int msize,
                                              uint8_t * buffer,
                                              bool copybuf = false
            );
    static TransportPDUPtr BuildTransportPDU(int msize
            );

    TransportPDU(int msize, uint8_t * buffer, bool copybuf = false);
    TransportPDU(int msize);
    uint8_t GetSeqNum();
    void SetSeqNum(uint8_t seq);
    void IncSeqNum();
    void UpdateBuffer(uint8_t * newBuffer);
    uint8_t * GetPayloadBuffer();
    uint8_t * GetPayloadBuffer (int &size, bool copy = false);

private:
    uint8_t * _nseq,
            * _payload;

    int _payloadSize;

    void _Init();
    void _InitPointers();
};

}

#endif // TRANSPORTPDU_H
