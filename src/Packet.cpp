#include <dccomms/Packet.h>
#include <stdlib.h>
#include <stdint.h>
#include <cstring>

namespace dccomms {

  Packet::Packet(uint32_t msize, uint8_t * buffer, bool copybuf)
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


  Packet::Packet(uint32_t msize)
  {
    _bufferSize = msize;
    _buffer = new uint8_t[_bufferSize];
    _bufferCopy = false;
  }

  Packet::Packet()
  {
  }


  Packet::~Packet()
  {
    if(_bufferCopy)
      delete _buffer;
  }

}


