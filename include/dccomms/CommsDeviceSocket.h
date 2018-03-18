/*
 * Radio.h
 *
 *  Created on: Feb 11, 2015
 *      Author: diego
 */

#ifndef DCCOMMS_COMMSDEVICESOCKET_H_
#define DCCOMMS_COMMSDEVICESOCKET_H_

#include <cpplogging/Loggable.h>
#include <dccomms/CommsDevice.h>
#include <dccomms/DataLinkFrame.h>

namespace dccomms {

class CommsDeviceSocket;
typedef std::shared_ptr<CommsDeviceSocket> CommsDeviceSocketPtr;

class CommsDeviceSocket : public cpplogging::Loggable, public Stream {
public:
  ~CommsDeviceSocket();
  CommsDeviceSocket(uint32_t addr, uint32_t maxRxBufferSize = 5000);

  void SetCommsDevice(Ptr<CommsDevice> dev);
  void SetPacketBuilder(PacketBuilderPtr pb);
  void Send(const void *, uint32_t size, uint32_t dirTo = 255,
            uint32_t packetSize = 1000, unsigned long ms = 0);
  void Recv(void *, uint32_t size, unsigned long ms = 10000);
  inline void SetDefaultDestAddr(uint32_t dst) { _defaultDestAddr = dst; }

  // Stream methods to implement
  int Read(void *, uint32_t, unsigned long msTimeout = 0);
  int Write(const void *, uint32_t, uint32_t msTimeout = 0);
  int Available();
  bool IsOpen();
  void FlushInput();
  void FlushOutput();
  void FlushIO();

  int TotalErrors = 0;

private:
  PacketPtr _BuildPacket(uint8_t *buffer, uint32_t dataSize, uint32_t _addr,
                         uint32_t dst);

  Ptr<CommsDevice> _device;
  unsigned char _addr;
  PacketBuilderPtr _packetBuilder;

  uint32_t _maxRxBufferSize;
  uint8_t *_rxBuffer;
  uint32_t _rxBufferFirstPos;
  uint32_t _rxBufferLastPos;
  uint32_t _bytesInBuffer = 0;
  uint32_t _defaultDestAddr;

  void _DecreaseBytesInBuffer();
  void _IncreaseBytesInBuffer();
};
}

#endif /* DCCOMMS_RADIO_H_ */
