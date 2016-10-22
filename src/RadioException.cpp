/*
 * RadioException.cpp
 *
 *  Created on: 06/03/2015
 *      Author: diego
 */

#include <RadioException.h>
#include <cstring>
#include <string>

namespace dccomms {

RadioException::RadioException(std::string msg, int cod) {
	message = msg;
	code = cod;
}

RadioException::~RadioException() {
}

} /* namespace radiotransmission */
