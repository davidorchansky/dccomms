/*
 * Stream.h
 *
 *  Created on: Feb 15, 2015
 *      Author: diego
 */

#ifndef STREAM_H_
#define STREAM_H_

#include <cstdint>
namespace radiotransmission {

class Stream {
public:
	Stream();
	virtual ~Stream();

	virtual int Read(void *, uint32_t, unsigned long msTimeout=0) = 0;
	virtual int Write(const void *, uint32_t, uint32_t msTimeout=0) = 0;

	virtual Stream & operator >> (uint8_t &) = 0;
	virtual Stream & operator >> (char &) = 0;
	virtual Stream & operator >> (uint16_t &) = 0;
	virtual Stream & operator >> (uint32_t &) = 0;
	virtual Stream & operator << (uint8_t) = 0;
	virtual Stream & operator << (const char * str) = 0;
	virtual int Available() = 0;

	virtual bool IsOpen() = 0;
	//virtual void TimeoutMode(bool) = 0;
	virtual void SetTimeout(unsigned long);
	unsigned long GetTimeout();
	void WaitFor(const uint8_t * expected, uint32_t size);

	virtual void FlushInput() = 0;
	virtual void FlushOutput() = 0;
	virtual void FlushIO() = 0;

protected:
	unsigned long _timeout = 0;
};

} /* namespace radiotransmission */

#endif /* STREAM_H_ */
