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

#ifndef _THRUSTER_CONTROLLER_H_
#define _THRUSTER_CONTROLLER_H_


// ================================================================== includes

#include <I2cDevice.h>

// ========================================================== external defines

//========= STATUS ========
//...
//===========================
//========= FAULT =========
#define OVER_TEMP 			0x01
#define STALLED_MOTOR 		0x02
#define HALL_SENSOR_ERROR 	0x04
#define GROUND_FAULT 		0x08
#define WATER_DETECT 		0x10
//=========================
// ===================================================================== class
class ControladorMotores
{
public:
	ControladorMotores();
	bool changeSeabotixAddr(uint8_t oldAddr, uint8_t newAddr);
	bool checkSeabotixStatus(uint8_t seabotixAddr);
	bool forward(uint8_t add, int speed);
	bool stop(uint8_t add);
	bool reverse(uint8_t add, int speed);
	bool overTemp(uint8_t add,bool getStatus=true);
	bool stalledMotor(uint8_t add,bool getStatus=true);
	bool hallSensorError(uint8_t add,bool getStatus=true);
	bool groundFault(uint8_t add,bool getStatus=true);
	bool waterDetect(uint8_t add,bool getStatus=true);
	float getTemperature(uint8_t add,bool getStatus=true);
	float getSpeed(uint8_t add,bool getStatus=true);
	float getCurrent(uint8_t add,bool getStatus=true);
	//float getCurrent(uint8_t add);


	uint8_t speed;
private:

	void seabotixSpeedTest(uint8_t seabotixAddr);
	bool sendSeabotixSpeed(uint8_t addr, uint8_t speed);


	I2cDevice * i2cDevice;
	uint8_t status;
	uint8_t fault;
	uint8_t current;
	uint8_t temperature;
};

#endif /* _THRUSTER_CONTROLLER_H_ */
