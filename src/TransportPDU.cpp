#include <cstring>
#include <dccomms/TransportPDU.h>

namespace dccomms {

int TransportPDU::OverheadSize = 1;

TransportPDU::TransportPDU(uint8_t *buffer, int size) : Packet(buffer, size) {
  _Init();
}

void TransportPDU::_Init() {
  _InitPointers();
  SetSeqNum(0);
}

void TransportPDU::_InitPointers() {
  _nseq = GetBuffer();
  _payload = _nseq + OverheadSize;
}

uint8_t TransportPDU::GetSeqNum() { return *_nseq; }

void TransportPDU::SetSeqNum(uint8_t seq) { *_nseq = seq; }

void TransportPDU::IncSeqNum() { *_nseq = *_nseq + 1; }
uint8_t *TransportPDU::GetPayloadBuffer() { return _payload; }

uint32_t TransportPDU::GetPayloadSize() { return _payloadSize; }

void TransportPDU::BufferUpdated() { _InitPointers(); }

TransportPDUPtr TransportPDU::BuildTransportPDU(uint8_t *buffer, int size) {
  return TransportPDUPtr(new TransportPDU(buffer, size));
}
}
