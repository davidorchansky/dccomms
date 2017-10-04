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
  // if size > 0:
  //  - alloc own buffer of 'size' bytes length.
  //  - copy 'size' bytes from buffer to own buffer
  Packet(uint8_t *buffer, int size = 0);
  Packet();

  virtual uint8_t *GetPayloadBuffer() = 0;
  virtual uint32_t GetPayloadSize() = 0;
  inline virtual uint8_t *GetBuffer() const { return _buffer; }
  virtual int GetPacketSize() = 0;
  virtual ~Packet();

  virtual void Read(IStream *comms) = 0;
  virtual void Write(IStream *comms);

  // if size > 0:
  //  - alloc own buffer of 'size' bytes length.
  //  - copy 'size' bytes from buffer to own buffer
  void UpdateBuffer(uint8_t *buffer, int size = 0);
  virtual void BufferUpdated() = 0;

protected:
  void InitBuffer(int size);

private:
  uint8_t *_buffer;
  bool _ownBuffer;
  void _DeleteBuffer();
  void _UpdateBuffer(uint8_t *buffer, int size = 0);
};
}
#endif // DCCOMMS_PACKET_H
