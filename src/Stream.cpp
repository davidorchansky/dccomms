/*
 * Stream.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#include <cstring>
#include <dccomms/IStream.h>
#include <iostream>

namespace dccomms {

Stream::Stream() {
  // TODO Auto-generated constructor stub
}

Stream::~Stream() {
  // TODO Auto-generated destructor stub
}

void Stream::WaitFor(const uint8_t *expected, uint32_t size) {
  const uint8_t *ptr = expected;
  const uint8_t *max = expected + size;
  uint8_t c;
  while (ptr != max) {
    this->Read(&c, 1);
    if (c == *ptr)
      ptr++;
    else
      ptr = expected;
  }
}

int Stream::ReadInt(int &num, char &nextByte) {
  int n = 0; // number of [0-9] read
  char *ptr = buffer;

  Read(ptr, 1);
  nextByte = *ptr;
  if (*ptr == '-' || *ptr == '+') {
    ptr++;
    Read(ptr, 1);
    nextByte = *ptr;
  }
  while (*ptr >= '0' && *ptr <= '9') {
    ptr++;
    n++;
    Read(ptr, 1);
    nextByte = *ptr;
  }
  if (n) {
    num = atoi(buffer);
    return ptr - buffer; // Number of bytes read
  }
  return -1;
}

int Stream::ReadUInt(int &num, char &nextByte) {
  int n = 0; // number of [0-9] read
  char *ptr = buffer;

  Read(ptr, 1);
  nextByte = *ptr;
  while (*ptr >= '0' && *ptr <= '9') {
    ptr++;
    n++;
    Read(ptr, 1);
    nextByte = *ptr;
  }
  if (n) {
    num = atoi(buffer);
    return ptr - buffer; // Number of bytes read
  }
  return -1;
}

int Stream::ReadUntil(uint8_t *dst, const uint8_t *finalPattern,
                       int finalPatternLength, int maxLength) {
  uint8_t *cdptr = dst, *edptr = dst + maxLength;

  const uint8_t *cfpptr = finalPattern,
                *efpptr = finalPattern + finalPatternLength;

  while (cfpptr < efpptr) {
    if (cdptr < edptr) {
      Read(cdptr, 1);
      if (*cdptr != *cfpptr) {
        cfpptr = finalPattern;
      } else {
        cfpptr++;
      }
      cdptr++;
    } else
      return cdptr - dst;
  }
  return cdptr - dst;
}

/*
int Stream::Read(uint8_t * buf, uint32_t size, uint32_t to)
{


}

int Stream::Write(const uint8_t * buf, uint32_t size, uint32_t to)
{


}
int Stream::Available()
{

}


Stream & Stream::operator >> (uint8_t &byte)
{

}
*/
Stream &Stream::operator<<(const std::string &str) {
  *this << str.c_str();
  return *this;
}

Stream &Stream::operator<<(uint8_t byte) {
  Write(&byte, sizeof(uint8_t));
  return *this;
}

Stream &Stream::operator<<(const char *str) {
  int n = strlen(str);
  Write(str, n);
  return *this;
}

} /* namespace radiotransmission */