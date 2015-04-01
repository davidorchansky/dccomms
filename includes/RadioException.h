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



using namespace std;
namespace radiotransmission {

#define RADIO_RXLINEDOWN   0
#define RADIO_TXLINEDOWN   1
#define RADIO_TIMEOUT      2
#define RADIO_CORRUPTFRAME 3
#define RADIO_CORRUPTBLOCK 4

class RadioException: public exception{
public:
	RadioException(const char* msg, int cod);
	virtual ~RadioException();
	virtual const char* what() const throw()
	{
		return message;
	}
	int code;
private:
	char * message = NULL;


};

} /* namespace radiotransmission */

#endif /* RADIOEXCEPTION_H_ */
