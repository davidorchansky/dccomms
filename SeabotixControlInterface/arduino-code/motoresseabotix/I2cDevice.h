// ===========================================================================
/**
 * Force controller class.
 *
 * file			ForceController.h
 *				\$RCSfile: ThrusterController.h,v $
 * @date		\$Date: 2010/03/26 18:28:53 $
 * @version		\$Revision: 1.3 $
 * 
 */

#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H
// ================================================================== includes

#include<Arduino.h>
// ========================================================== external defines

// ===================================================================== class
class I2cDevice
{
public:
	I2cDevice();
	bool readNoReg(uint8_t add, uint8_t nbytes, uint8_t * data);
	bool read(int deviceAdd, uint8_t regAddress, int num, uint8_t _buff[]);
	bool writeNoReg(uint8_t add, uint8_t nbytes, uint8_t * data);
	bool write(uint8_t deviceAdd, uint8_t regAddress, uint8_t val);

};
#endif 
