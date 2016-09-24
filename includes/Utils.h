/*
 * Utils.h
 *
 *  Created on: 05/03/2015
 *      Author: diego
 */

#ifndef UTILS_H_
#define UTILS_H_
#include <cstdint>
#include <initializer_list>
#include <string>
#include <iostream>

namespace radiotransmission {


#define LOG_DEBUG(msg) std::cerr << "DEBUG("+std::string(typeid(*this).name())+":"+std::string(__func__)+"): " + std::string(msg) << std::endl; std::cerr << std::flush;

class Utils {
public:
	Utils();
	virtual ~Utils();
	static bool IsBigEndian();
	static void IntSwitchEndian(void * b, uint32_t entero);
	static std::string BuildString(std::initializer_list<std::string> list );
	static void Debug(std::ostream &, std::string &);
};

} /* namespace radiotransmission */

#endif /* UTILS_H_ */
