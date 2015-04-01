/*
 * RadioException.cpp
 *
 *  Created on: 06/03/2015
 *      Author: diego
 */

#include <RadioException.h>
#include <cstring>

namespace radiotransmission {

RadioException::RadioException(const char * msg, int cod) {
	int size = strlen(msg);
	message = new char[size+1];
	strcpy(message, msg);
	code = cod;
}

RadioException::~RadioException() {
	// TODO Auto-generated destructor stub
//	if(message != NULL)
//		delete message;
//	message = NULL;
}

} /* namespace radiotransmission */
