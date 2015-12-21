
#include <I2cDevice.h>
#include <Wire.h>

I2cDevice::I2cDevice()
{
	Wire.begin();
}

bool I2cDevice::readNoReg(uint8_t add, uint8_t nbytes, uint8_t * data)
{
	Wire.beginTransmission(add);
	Wire.write(0x57);
	Wire.endTransmission();
	Wire.requestFrom(add, nbytes);

	while(Wire.available()) // device may send less than requested (abnormal)
	{
			*data = Wire.read(); // receive a byte
			data++;
	}
	Wire.endTransmission(); // end transmission

	return true;
}

bool I2cDevice::read(int deviceAdd, uint8_t regAddress, int num, uint8_t _buff[])
{
	Wire.beginTransmission(deviceAdd); // start transmission to device
	Wire.write(regAddress); // sends address to read from
	Wire.endTransmission(); // end transmission

	Wire.beginTransmission(deviceAdd); // start transmission to device
	Wire.requestFrom(deviceAdd, num); // request 6 bytes from device

	int i = 0;
	while(Wire.available()) // device may send less than requested (abnormal)
	{
		_buff[i] = Wire.read(); // receive a byte
		i++;
	}
	Wire.endTransmission(); // end transmission
	return true;
}

bool I2cDevice::writeNoReg(uint8_t add, uint8_t nbytes, uint8_t * data)
{
	Wire.beginTransmission(add);
	for(int i=0;i<nbytes;i++)
	{
		Wire.write(*data);
		data++;
	}
	Wire.endTransmission();
	return true;
}

bool I2cDevice::write(uint8_t deviceAdd, uint8_t regAddress, uint8_t val)
{
	Wire.beginTransmission(deviceAdd); // start transmission to device
	Wire.write(regAddress); // send register address
	Wire.write(val); // send value to write
	Wire.endTransmission(); // end transmission

	return true;
}
