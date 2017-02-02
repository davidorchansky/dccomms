#include <dccomms/IPDU.h>
#include <stdlib.h>
#include <cstring>

namespace dccomms {

IPDU::IPDU(int msize, uint8_t * buffer, bool copybuf)
{
    _bufferSize = msize;
    _bufferCopy = copybuf;
    if(!_bufferCopy)
    {
        _buffer = buffer;
    }
    else
    {
        memcpy(_buffer, buffer, msize);
    }
}


IPDU::IPDU(int msize)
{
    _bufferSize = msize;
    _buffer = new uint8_t[_bufferSize];
    _bufferCopy = false;
}


IPDU::~IPDU()
{
    if(_bufferCopy)
        delete _buffer;
}

}
