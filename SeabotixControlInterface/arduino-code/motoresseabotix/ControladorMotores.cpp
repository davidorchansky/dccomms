// ==========================================================================
/**
 * Force controller class.
 *
 *
 * file			ForceController.cpp
 *				\$RCSfile: ThrusterController.cpp,v $
 * @date		\$Date: 2010/03/26 18:28:53 $
 * @version		\$Revision: 1.3 $
 *
 */

// Uncomment DEBUG define to enable TRACE output.
//#define DEBUG

// ================================================================== includes

//#include <iostream>
//#include <iomanip>
//#include <string>

//#include <boost/make_shared.hpp>



#include <float.h>
#include <math.h>

#include <ControladorMotores.h>


// =================================================================== defines

#define SEABOTIX_BASE_I2C_ADDRESS	0x56
#define FW_SP_M 1.02
#define FW_SP_B 0x80

#define RV_SP_M -1.02
#define RV_SP_B 0x7f

// =================================================================== methods

ControladorMotores::ControladorMotores()
{
	i2cDevice=new I2cDevice();
}

// ---------------------------------------------------------------------------
bool ControladorMotores::changeSeabotixAddr(uint8_t oldAddr, uint8_t newAddr)
{
	uint8_t data[7];
	uint8_t checksum;

	if (!i2cDevice->readNoReg(oldAddr>>1, 7, data))
	{
	/*	cerr << "changeSeabotixAddr: ERROR reading status from old addr: 0x"
			<< hex << (int)oldAddr << dec << endl;*/
		return false;
	}

/*	cout << "changeSeabotixAddr: Read old status hex: ";
	printHex(data, 7);
	cout << endl;
*/
	// First part of change address command
	data[0] = 0x06;
	data[1] = 0x72;
	// Checksum
	checksum = oldAddr;
	for (int i=0; i<2; ++i) checksum += data[i];
	data[2] = checksum;

	if (!i2cDevice->writeNoReg(oldAddr>>1, 3, data))
	{
/*		cerr << "changeSeabotixAddr: ERROR writing first command to addr: 0x"
			<< hex << (int)oldAddr << dec << endl;*/
		return false;
	}

	// Second part of change address command
	data[0] = newAddr;
	// Checksum
	checksum = oldAddr;
	for (int i=0; i<1; ++i) checksum += data[i];
	data[1] = checksum;

	if (!i2cDevice->writeNoReg(oldAddr>>1, 2, data))
	{
/*		cerr << "changeSeabotixAddr: ERROR writing second command to addr: 0x"
			<< hex << (int)oldAddr << dec << endl;*/
		return false;
	}

/*	cout << "changeSeabotixAddr: thruster address change cmd issued: " << hex << (int)oldAddr << " -> " << hex << (int)newAddr << dec << endl;
	cout << "changeSeabotixAddr: POWER CYCLE IS NEEDED FOR ADDRESS CHANGE TO TAKE EFFECT." << endl;
	cout << "changeSeabotixAddr: SLEEPING FOREVER NOW - PRESS CTRL-C TO KILL MODULE." << endl;
*/
	while (true) delay(100);

/*
	cout << "changeSeabotixAddr: sleeping for 15 sec to allow user to cycle power" << endl;

	sleep(15000);

	if (!i2cDevice->readNoReg(newAddr, 7, data))
	{
		cerr << "changeSeabotixAddr: ERROR reading status from new addr: "
			<< hex << (int)newAddr << dec << endl;
		return false;
	}

	cout << "changeSeabotixAddr: Read new status hex: ";
	printHex(data, 7);
	cout << endl;

	cout << "changeSeabotixAddr: Address changed from 0x"
		<< hex << (int)oldAddr << dec
		<< " to " << hex << (int)newAddr << dec << endl;
*/

	return true;
}

// ---------------------------------------------------------------------------
bool ControladorMotores::checkSeabotixStatus(uint8_t seabotixAddr)
{
	// This code reads the "SeaBotix Motor Controller Thruster Motor Controller To LBV
	// Motherboard Message" - see Seabotix "Interface Design Specification Between
	// LBV Motherboard and Thruster Motor Controller". Here's a summary:
	// Byte 0: Address of thruster
	// Byte 1: Status, bits:
	//          0: 1 = non-current limiting version of firmware, 0 = current limiting
	//          1: 1 = brushed motor, 0 = brushless
	//          2: future growth
	//          3: future growth
	//          4-7: revision number of firmware
	// Byte 2: Fault, bits:
	//          0: 1 = overtemp
	//          1: 1 = stalled motor
	//          2: 2 = hall sensor error
	//          3: 3 = ground fault
	//          4: 4 = water detect
	//          5-6: future growth
	// Byte 3: Current: for brushless DC software, indicates drawn current in 1/10 Amps.
	// Byte 4: Speed: actual speed of the thruster (see document for definition).
	// Byte 5: Temperature: motor temp in deg C, or motor controller temp for brushed motors.
	// Byte 6: Checksum: eight bit unsigned addition without carry of above fields.

	uint8_t data[7];
	bool success;

	for (int tries=1; tries<=3; ++tries) {
		success = true;

		//Serial.println("prueba1");
		if (!i2cDevice->readNoReg(seabotixAddr>>1, 7, data))
		{
			//Serial.println("prueba2");
	//		cout << "Error reading seabotix thruster 0x" << hex << (int)seabotixAddr
	//			<< dec << " status." << endl;
			// Not fatal, so continue to try again.
			success = false;
			continue;
		}
		//Serial.println("prueba3");
	/*	cout << "Seabotix thruster 0x" << hex << (int)seabotixAddr << dec
			<< " status read try " << tries << " hex: ";*/
//		printHex(data, 7);
//		cout << endl;

		uint8_t checksum = 0;
		for (int i=0; i<6; ++i) checksum += data[i];
		if (checksum != data[6]) {
		//	Serial.println("prueba4");
		/*	cout << "Seabotix thruster 0x" << hex << (int)seabotixAddr << dec
				<< " status checksum mismatch; expected 0x" << hex << (int)checksum
				<< " read 0x" << hex << (int)data[6] << dec << endl;*/
			// Not fatal, so continue to try again.
			success = false;
			continue;
		}
	//	Serial.println("prueba5");
		status = data[1];
		fault = data[2];
		current = data[3];
		speed = data[4];
		temperature = data[5];

	/*	cout << "Seabotix thruster 0x" << hex << (int)seabotixAddr << dec << "   Status:";
		if (status & (1<<0)) cout << " non-current-limiting";
		else cout << " current-limiting";
		if (status & (1<<1)) cout << " brushless-motor";
		else cout << " brushed-motor";
		cout << " firmware-0x" << hex << (int)(status >> 4) << dec;
		cout << endl;

                cout << "Seabotix thruster 0x" << hex << (int)seabotixAddr << dec << "   Temperature: "
                                << (int)temperature << " deg C" << endl;
*/
		if (fault != 0)
		{
			/*cerr << "Seabotix thruster 0x" << hex << (int)seabotixAddr << dec << "   FAULTS:";
			if (fault & (1<<0)) cout << " overtemp";
			if (fault & (1<<1)) cout << " stalled-motor";
			if (fault & (1<<2)) cout << " hall-sensor-error";
			if (fault & (1<<3)) cout << " ground-fault";
			if (fault & (1<<4)) cout << " water-detect";
			cout << endl;*/
			success = false;
		}
		/*
		else {
			cout << "Seabotix thruster 0x" << hex << (int)seabotixAddr << dec << "   Faults: none" << endl;
		}


		// If retrying, continue would have been called already.
		*/
		break;
	}
	return success;
}

// ---------------------------------------------------------------------------
void ControladorMotores::seabotixSpeedTest(uint8_t seabotixAddr)
{
	uint8_t data[7];

	for (int speed=127; speed<=230; speed+=1)
	{
		// Set speed
		data[0] = speed;
		// Set additional info byte
		data[1] = 100;
		// Set checksum - note I2C address is included.
		data[2] = seabotixAddr + data[0] + data[1];

		if (i2cDevice->writeNoReg(seabotixAddr>>1, 3, data))
		{
	/*		cout << "Speed " << (speed -128) << ": ";
			printHex(data, 3);
			cout << endl;
			*/
		}

		if (i2cDevice->readNoReg(seabotixAddr>>1, 7, data))
		{
			/*
			cout << "Read status hex: ";
			printHex(data, 7);
			cout << endl;
			*/
		}
		delay(50);
	}

	// Set speed
	data[0] = 127;
	// Set additional info byte
	data[1] = 100;
	// Set checksum - note I2C address is included.
	data[2] = seabotixAddr + data[0] + data[1];

	i2cDevice->writeNoReg(seabotixAddr, 3, data);
}

// ---------------------------------------------------------------------------
bool ControladorMotores::sendSeabotixSpeed(uint8_t addr, uint8_t speed)
{
	uint8_t data[3];

	// Set speed
	data[0] = speed;
	// Set additional info byte
	data[1] = 100;
	// Set checksum - note I2C address is included.
	data[2] = addr + data[0] + data[1];

	//cout << "sendSeabotixSpeed: ";
	//printHex(data, 3);
	//cout << endl;

	if (!i2cDevice->writeNoReg(addr>>1, 3, data))
	{
		//cerr << "sendSeabotixSpeed: error setting controller 0x" << hex << (int)addr
		//	<< " to speed " << dec << (int)speed << endl;
		return false;
	}

	return true;
}
/*
Serial.println((int)(0x81 + (float)speed/100*(0xE6 - 0x81)),HEX);
Serial.println((int)(0x7E - (float)speed/100*(0x7E - 0x19)),HEX);
 *
 */
bool ControladorMotores::forward(uint8_t add, int speed)
{
	if(speed>100)speed=100;
	else if(speed<0)speed=0;
	return sendSeabotixSpeed(add, (int)(FW_SP_M*speed+FW_SP_B));
}
bool ControladorMotores::stop(uint8_t add)
{

	return sendSeabotixSpeed(add,0x80);
}

bool ControladorMotores::reverse(uint8_t add,int speed)
{
	if(speed>100)speed=100;
	else if(speed<0)speed=0;
	return sendSeabotixSpeed(add,(int)(RV_SP_M*speed+RV_SP_B));
}

bool ControladorMotores::overTemp(uint8_t add,bool getStatus)
{
	if(getStatus)checkSeabotixStatus(add);
	return (fault & OVER_TEMP);
}

bool ControladorMotores::stalledMotor(uint8_t add,bool getStatus)
{
	if(getStatus)checkSeabotixStatus(add);
	return (fault & STALLED_MOTOR);
}

bool ControladorMotores::hallSensorError(uint8_t add,bool getStatus)
{
	if(getStatus)checkSeabotixStatus(add);
	return (fault & HALL_SENSOR_ERROR);
}

bool ControladorMotores::groundFault(uint8_t add,bool getStatus)
{
	if(getStatus)checkSeabotixStatus(add);
	return (fault & GROUND_FAULT);
}

bool ControladorMotores::waterDetect(uint8_t add,bool getStatus)
{
	if(getStatus)checkSeabotixStatus(add);
	return (fault & WATER_DETECT);
}

float ControladorMotores::getTemperature(uint8_t add,bool getStatus)
{
	if(getStatus)checkSeabotixStatus(add);
	return temperature;
}

float ControladorMotores::getSpeed(uint8_t add,bool getStatus)
{
	/*
	if(getStatus)checkSeabotixStatus(add);
	if(speed <= 0xe6 && speed >= 0x80)
		return ((float)(speed-FW_SP_B))/FW_SP_M;
	else if(speed <= 0x7f && speed >= 0x19)
		return ((float)(speed-RV_SP_B))/RV_SP_M;
	else return -1;
	*/
	if(getStatus)checkSeabotixStatus(add);
	return (speed+0.4)/1.96;
}

float ControladorMotores::getCurrent(uint8_t add,bool getStatus)
{
	if(getStatus)checkSeabotixStatus(add);
	return ((float)current)/10;
}

