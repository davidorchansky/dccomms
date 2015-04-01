/*
 * Utils.h
 *
 *  Created on: 05/03/2015
 *      Author: diego
 */

#ifndef UTILS_H_
#define UTILS_H_
#include <cstdint>
namespace radiotransmission {

class Utils {
public:
	Utils();
	virtual ~Utils();
	static bool IsBigEndian();
	static void IntSwitchEndian(void * b, uint32_t entero);
};

} /* namespace radiotransmission */

#endif /* UTILS_H_ */
