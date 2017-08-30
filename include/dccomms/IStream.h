/*
 * Stream.h
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#ifndef STREAM_H_
#define STREAM_H_

#include <cstdint>
#include <string>

namespace dccomms {

  class IStream {
  public:
    IStream();
    virtual ~IStream();

    virtual int Read(void *, uint32_t, unsigned long msTimeout=0) = 0;
    virtual int Write(const void *, uint32_t, uint32_t msTimeout=0) = 0;

    virtual IStream & operator >> (uint8_t &) = 0;
    virtual IStream & operator >> (char &) = 0;
    virtual IStream & operator >> (uint16_t &) = 0;
    virtual IStream & operator >> (uint32_t &) = 0;
    virtual IStream & operator << (uint8_t);

    virtual int Available() = 0;

    virtual bool IsOpen() = 0;
    //virtual void TimeoutMode(bool) = 0;

    virtual void FlushInput() = 0;
    virtual void FlushOutput() = 0;
    virtual void FlushIO() = 0;

    void WaitFor(const uint8_t * expected, uint32_t size);
    int ReadInt(int & num, char & nextByte);
    int ReadUInt(int & num, char & nextByte);
    IStream & operator << (const char * str);
    IStream & operator << (const std::string & str);

    int ReadUntil(uint8_t * dst, const uint8_t * finalPattern, int finalPatternLength, int maxLength);

  private:
    char buffer[1024];
  };

} /* namespace radiotransmission */

#endif /* STREAM_H_ */
