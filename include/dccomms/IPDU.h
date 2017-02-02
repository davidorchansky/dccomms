#ifndef IPDU_H
#define IPDU_H

#include <stdint.h>

namespace dccomms {
class IPDU
{
public:
    IPDU(int msize, uint8_t * buffer, bool copybuf = false);
    IPDU(int msize);
    virtual uint8_t * GetPayloadBuffer() = 0;
    virtual uint8_t * GetPayloadBuffer(int &size, bool copy = false) = 0;

    virtual ~IPDU();

protected:
    uint8_t * _buffer;
    int _bufferSize;
    bool _bufferCopy;

};
}

#endif // IPDU_H
