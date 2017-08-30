#ifndef PACKET_H
#define PACKET_H

#include <iostream>
#include <memory>

namespace dccomms{

  class Packet;
  typedef std::shared_ptr<Packet> PacketPtr;

  class Packet
  {
  public:
    static PacketPtr BuildPacket();

    Packet(uint32_t msize, uint8_t * buffer, bool copybuf = false);
    Packet(uint32_t msize);
    Packet();
    virtual uint8_t * GetPayloadBuffer() = 0;
    virtual uint32_t GetPayloadSize() = 0;
    inline virtual uint32_t GetBufferSize(){
      return _bufferSize;
    }
    inline virtual uint8_t * GetBuffer(){
      return _buffer;
    }
    virtual ~Packet();
  protected:
    uint8_t * _buffer;
    uint32_t _bufferSize;
    bool _bufferCopy;
  };
}
#endif // PACKET_H
