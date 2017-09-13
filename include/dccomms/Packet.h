#ifndef PACKET_H
#define PACKET_H

#include <iostream>
#include <memory>

namespace dccomms {

class Packet;
typedef std::shared_ptr<Packet> PacketPtr;

class Packet {
public:
  static PacketPtr BuildPacket();
  // if size > 0:
  //  - alloc own buffer of 'size' bytes length.
  //  - copy 'size' bytes from buffer to own buffer
  Packet(uint8_t *buffer, int size = 0);
  Packet();

  virtual uint8_t *GetPayloadBuffer() = 0;
  virtual uint32_t GetPayloadSize() = 0;
  inline virtual uint8_t *GetBuffer() const { return _buffer; }
  virtual ~Packet();

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
#endif // PACKET_H
