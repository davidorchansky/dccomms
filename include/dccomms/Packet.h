#ifndef DCCOMMS_PACKET_H
#define DCCOMMS_PACKET_H

#include <dccomms/IStream.h>
#include <iostream>
#include <memory>

namespace dccomms {

class Packet;
typedef std::shared_ptr<Packet> PacketPtr;

class Packet {
public:
  Packet();
  virtual ~Packet();
  inline uint8_t *GetBuffer() const { return _buffer; }

  virtual void CopyFromRawBuffer(void *buffer) = 0;
  virtual uint8_t *GetPayloadBuffer() = 0;
  virtual uint32_t GetPayloadSize() = 0;
  virtual int GetPacketSize() = 0;
  virtual void Read(IStream *comms) = 0;

  virtual bool PacketIsOk();
  virtual void Write(IStream *comms);

protected:
  void _AllocBuffer(int size);
  virtual void _SetBuffer(void *buffer);

private:
  uint8_t *_buffer;
  bool _ownBuffer;
  void _FreeBuffer();
};
}
#endif // DCCOMMS_PACKET_H
