#include <cstring>
#include <dccomms/Packet.h>
#include <stdint.h>
#include <stdlib.h>

namespace dccomms {

Packet::Packet(uint8_t *buffer, int size) { _UpdateBuffer(buffer, size); }

Packet::Packet() {
  _buffer = NULL;
  _ownBuffer = true;
}

Packet::~Packet() { _DeleteBuffer(); }

void Packet::_DeleteBuffer() {
  if (_ownBuffer && _buffer) {
    delete _buffer;
    _buffer = NULL;
  }
}

void Packet::InitBuffer(int size) {
  _DeleteBuffer();
  _buffer = new uint8_t[size];
  _ownBuffer = true;
}

void Packet::_UpdateBuffer(uint8_t *buffer, int size) {
  if (size > 0) {
    _DeleteBuffer();
    _buffer = new uint8_t[size];
    memcpy(_buffer, buffer, size);
    _ownBuffer = true;
  } else {
    _buffer = buffer;
    _ownBuffer = false;
  }
}

void Packet::UpdateBuffer(uint8_t *buffer, int size) {
  _UpdateBuffer(buffer, size);
  BufferUpdated();
}

void Packet::Write(IStream *comms) {
  comms->Write(GetBuffer(), GetPacketSize());
}
}
