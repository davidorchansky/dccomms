/*
 * TCPStream.h
 *
 *  Created on: 24 oct. 2016
 *      Author: diego
 */

#ifndef DCCOMMS_TCPSTREAM_H_
#define DCCOMMS_TCPSTREAM_H_

#include <dccomms/IStreamCommsDevice.h>
#include <string>

// Cstyle
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace dccomms {
class TCPStream : public IStreamCommsDevice {
public:
  TCPStream();
  TCPStream(std::string serveraddr);
  virtual ~TCPStream();
  void CloseConnection();
  void OpenConnection();
  void SetServerAddr(std::string);

  virtual int Read(void *, uint32_t, unsigned long msTimeout = 0);
  int Write(const void *, uint32_t, uint32_t msTimeout = 0);

  virtual ICommsLink &operator>>(const PacketPtr &dlf) {
    return IStreamCommsDevice::operator>>(dlf);
  }
  virtual ICommsLink &operator<<(const PacketPtr &dlf) {
    return IStreamCommsDevice::operator<<(dlf);
  }

  IStream &operator>>(uint8_t &);
  IStream &operator>>(char &);
  IStream &operator>>(uint16_t &);
  IStream &operator>>(uint32_t &);

  int Available();

  bool IsOpen();
  // void TimeoutMode(bool);
  void FlushInput();
  void FlushOutput();
  void FlushIO();

  bool Open();
  void Close();

  //-----

protected:
  bool Connected();
  bool Ready();
  void ThrowExceptionIfErrorOnSocket();
  int Recv(unsigned char *, int bytesLeft, bool block = true);

  int sockfd, portno;
  std::string ip;
  struct sockaddr_in device_addr;
  struct hostent *device;
};
}
#endif /* DCCOMMS_TCPSTREAM_H_ */
