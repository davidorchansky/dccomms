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

#define RXLINEDOWN   0
#define TXLINEDOWN   1
#define TIMEOUT      2
#define CORRUPTFRAME 3
#define CORRUPTBLOCK 4
#define UNKNOWN_ERROR 5
#define PHYLAYER_ERROR 6
#define DLNKLAYER_ERROR 7

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
