#include <cstring>
#include <dccomms/Packet.h>
#include <stdint.h>
#include <stdlib.h>

namespace dccomms {

Packet::Packet(uint32_t msize, uint8_t *buffer, bool copybuf) {
  _bufferSize = msize;
  _bufferCopy = copybuf;
  if (!_bufferCopy) {
    _buffer = buffer;
  } else {
    _Alloc();
    memcpy(_buffer, buffer, msize);
  }
}

Packet::Packet(uint32_t msize) {
  _bufferCopy = false;
  _bufferSize = msize;
  _Alloc();
}

void Packet::_Alloc() { _buffer = new uint8_t[_bufferSize]; }

Packet::Packet() {}

Packet::~Packet() {
  if (_bufferCopy)
    delete _buffer;
}
}
