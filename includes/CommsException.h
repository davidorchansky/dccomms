/*
 * RadioException.h
 *
 *  Created on: 06/03/2015
 *      Author: diego
 */

#ifndef RADIOEXCEPTION_H_
#define RADIOEXCEPTION_H_

#include <exception>
#include <cstdio>
#include <string>


using namespace std;
namespace dccomms {

#define COMMS_EXCEPTION_LINEDOWN 	 	0
#define COMMS_EXCEPTION_TIMEOUT      	2
#define COMMS_EXCEPTION_CORRUPTFRAME 	3
#define COMMS_EXCEPTION_CORRUPTBLOCK 	4
#define COMMS_EXCEPTION_UNKNOWN_ERROR 	5
#define COMMS_EXCEPTION_PHYLAYER_ERROR 	6
#define COMMS_EXCEPTION_DLNKLAYER_ERROR 7
#define COMMS_EXCEPTION_NOTIMPLEMENTED 	8

class CommsException: public exception{
public:
	CommsException(std::string msg, int cod);
	virtual ~CommsException();
	virtual const char* what() const throw()
	{
		return message.c_str();
	}
	int code;
private:
	std::string message;


};

} /* namespace radiotransmission */

#endif /* RADIOEXCEPTION_H_ */
