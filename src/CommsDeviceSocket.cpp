/*
 * Radio.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: diego
 */

#include <chrono>
#include <dccomms/Arduino.h>
#include <dccomms/CommsDeviceSocket.h>
#include <dccomms/CommsException.h>
#include <dccomms/DataLinkFrame.h>
#include <iostream>
#include <thread> // std::this_thread::sleep_for

namespace dccomms {

CommsDeviceSocket::CommsDeviceSocket(uint32_t d, uint32_t maxRxBufferSize)
    : _addr(d) {
  _maxRxBufferSize = maxRxBufferSize;
  _rxBuffer = new uint8_t[_maxRxBufferSize];
  _bytesInBuffer = 0;
  _rxBufferLastPos = 0;
  _rxBufferFirstPos = 0;
  TotalErrors = 0;

  // FCSType = (DataLinkFrame::fcsType) fcst;
  SetLogName("CommsDeviceSocket");
  SetLogLevel(cpplogging::off);
  SetPayloadSize(1000);
  EnableWaitForDeviceReady(false);
}

CommsDeviceSocket::~CommsDeviceSocket() {
  if (_rxBuffer != NULL)
    delete _rxBuffer;
}

void CommsDeviceSocket::SetCommsDevice(Ptr<CommsDevice> dev) { _device = dev; }

void CommsDeviceSocket::SetPacketBuilder(PacketBuilderPtr pb) {
  _packetBuilder = pb;
}

PacketPtr CommsDeviceSocket::_BuildPacket(uint8_t *buffer, uint32_t dataSize,
                                          uint32_t addr, uint32_t dst) {
  auto packet = _packetBuilder->Create();
  packet->SetPayload(buffer, dataSize);
  packet->SetSrcAddr(addr);
  packet->SetDestAddr(dst);
  return packet;
}

void CommsDeviceSocket::Send(const void *buf, uint32_t size, unsigned long ms) {
  uint8_t *buffer = (uint8_t *)buf;
  uint32_t numPackets = size / _packetSize;
  uint32_t np;
  for (np = 1; np < numPackets; np++) {
    while (_waitForDevice && _device->BusyTransmitting())
      ;

    PacketPtr dlfPtr =
        _BuildPacket(buffer, _packetSize, _addr, _defaultDestAddr);

    Log->debug("Sending packet...");

    _device << dlfPtr;
    buffer += _packetSize;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }
  if (numPackets > 0) {
    while (_waitForDevice && _device->BusyTransmitting())
      ;
    PacketPtr dlfPtr =
        _BuildPacket(buffer, _packetSize, _addr, _defaultDestAddr);

    Log->debug("Sending packet...");

    _device << dlfPtr;
    buffer += _packetSize;
  }

  uint32_t bytesLeft = size % _packetSize;
  if (bytesLeft) {
    while (_waitForDevice && _device->BusyTransmitting())
      ;
    if (numPackets > 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    PacketPtr dlfPtr = _BuildPacket(buffer, bytesLeft, _addr, _defaultDestAddr);

    Log->debug("Sending packet...");

    _device << dlfPtr;
  }
}

void CommsDeviceSocket::_IncreaseBytesInBuffer() {
  if (_bytesInBuffer < _maxRxBufferSize) {
    _rxBufferLastPos = (_rxBufferLastPos + 1) % _maxRxBufferSize;
    _bytesInBuffer++;
  }
}

void CommsDeviceSocket::_DecreaseBytesInBuffer() {
  if (_bytesInBuffer) {
    _rxBufferFirstPos = (_rxBufferFirstPos + 1) % _maxRxBufferSize;
    _bytesInBuffer--;
  }
}

void CommsDeviceSocket::Recv(void *buf, uint32_t size, unsigned long ms) {
  uint8_t *buffer = (uint8_t *)buf;
  uint32_t bytes = 0;
  PacketPtr dlfPtr = _packetBuilder->Create();
  uint16_t i;
  unsigned long currentTimeout = _device->GetTimeout();
  _device->SetTimeout(ms >= 0 ? ms : 0);

  if (_bytesInBuffer) // If bytes in buffer: read them
  {
    while (bytes < size && _bytesInBuffer) {
      *buffer = _rxBuffer[_rxBufferFirstPos];
      _DecreaseBytesInBuffer();
      buffer++;
      bytes++;
    }
  }

  try {
    while (bytes < size) // If more bytes needed, wait for them
    {
      _device >> dlfPtr;
      if (dlfPtr->PacketIsOk()) {
        Log->debug("Frame received without errors!");
        uint32_t bytesToRead = (bytes + dlfPtr->GetPayloadSize()) <= size
                                   ? dlfPtr->GetPayloadSize()
                                   : size - bytes;

        auto payloadBuffer = dlfPtr->GetPayloadBuffer();
        for (i = 0; i < bytesToRead; i++) {
          *buffer = payloadBuffer[i];
          buffer++;
        }
        bytes += dlfPtr->GetPayloadSize();
      } else {
        TotalErrors += 1;
        Log->error("Error in packet (Total Errors: {})", TotalErrors);
      }
    }
    if (bytes > size) // If we have received more bytes save them
    {
      uint32_t bytesLeft = bytes - size;
      uint8_t *ptr = _rxBuffer;
      _bytesInBuffer =
          bytesLeft <= _maxRxBufferSize ? bytesLeft : _maxRxBufferSize;
      uint8_t *maxPtr = _rxBuffer + _bytesInBuffer;
      auto payloadBuffer = dlfPtr->GetPayloadBuffer();
      while (ptr != maxPtr) {
        *ptr = payloadBuffer[i++];
        ptr++;
      }
      _rxBufferLastPos = 0;
      _rxBufferFirstPos = 0;
    }

    _device->SetTimeout(currentTimeout);
  } catch (CommsException &e) {
    _rxBufferLastPos = 0;
    _rxBufferFirstPos = 0;
    _device->SetTimeout(currentTimeout);
    throw;
  } catch (std::exception &e) {
    std::cerr << "Unexpected exception" << std::endl << std::flush;
    _rxBufferLastPos = 0;
    _rxBufferFirstPos = 0;
    _device->SetTimeout(currentTimeout);

  } catch (int &e) {
    std::cerr << "Unexpected exception" << std::endl << std::flush;
    _rxBufferLastPos = 0;
    _rxBufferFirstPos = 0;
    _device->SetTimeout(currentTimeout);
  }
}

int CommsDeviceSocket::Read(void *buff, uint32_t size,
                            unsigned long msTimeout) {
  Recv(buff, size, msTimeout);
  return size;
}
int CommsDeviceSocket::Write(const void *buff, uint32_t size,
                             uint32_t msTimeout) {
  Send(buff, size, msTimeout);
  return size;
}

int CommsDeviceSocket::Available() { return _bytesInBuffer; }
bool CommsDeviceSocket::IsOpen() { return true; }
void CommsDeviceSocket::FlushInput() {}
void CommsDeviceSocket::FlushOutput() {}
void CommsDeviceSocket::FlushIO() {}
}
