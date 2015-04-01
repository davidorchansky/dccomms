//============================================================================
// Name        : SerialTransmissionChannel.cpp
// Author      : Diego
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <Radio.h>
#include <Checksum.h>
#include <cstring>
#include <SerialPortInterface.h>
#include <DataLinkFrame.h>
#include <cstdint>
#include <tests.h>
#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>			// std::chrono::seconds

#include <cstring>
#include <Stream.h>
#include <Arduino.h>
#include <BlockRadioTransmitter.h>

using namespace radiotransmission;

void tx()
{
	Arduino arduino = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you TX?\n",
			"Yes, I'm TX");
	if(arduino.IsOpen())
		std::cout <<"TX listo\n";
}

void rx()
{
	Arduino arduino = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you RX?\n",
			"Yes, I'm RX");
	if(arduino.IsOpen())
		std::cout <<"RX listo\n";
}

void testrxtx()
{
	Arduino arduTx = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you TX?\n",
			"Yes, I'm TX");
	//SerialPortInterface spi("/dev/ttyACM1", SerialPortInterface::BAUD_115200);
	//spi.Open();
	//Arduino arduTx(spi);
	if(!arduTx.IsOpen())
		return;
	std::cout <<"TX listo\n";
	Arduino arduRx = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you RX?\n",
			"Yes, I'm RX");
	//SerialPortInterface spi2("/dev/ttyACM0", SerialPortInterface::BAUD_115200);
	//spi2.Open();
	//Arduino arduRx(spi2);
	if(!arduRx.IsOpen())
		return;
	std::cout <<"RX listo\n";

	uint8_t buffer[1000];
	for(int i=0; i< 1000; i++)
		buffer[i]='f';
	DataLinkFrame myframe = DataLinkFrame(0x1,0x2, 500,buffer,DataLinkFrame::crc32);

	DataLinkFrame rxframe = DataLinkFrame(DataLinkFrame::crc32);
	//std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	while(1)
	{
//		std::cout << "Enviando frame:" <<std::endl;
//		myframe.printFrame(std::cout);
		arduTx << myframe;
		arduRx >> rxframe;
		std::cout << "Recibido frame:" <<std::endl;
		rxframe.printFrame(std::cout);
		if(rxframe.checkFrame())
			std::cout << "El frame es correcto (sin errores)" << std::endl;
	}


}

void testrxtx2()
{
	Arduino arduTx = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you TX?\n",
			"Yes, I'm TX");
	if(!arduTx.IsOpen())
		return;
	std::cout <<"TX listo\n";
	Arduino arduRx = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you RX?\n",
			"Yes, I'm RX");
	if(!arduRx.IsOpen())
		return;
	std::cout <<"RX listo\n";

	uint8_t buffer[1000];
	uint8_t rxbuffer[1000];
	for(int i=0; i< 1000; i++)
		buffer[i]='f';

	Radio radioTx(0, arduTx, Radio::crc32);

	Radio radioRx(1,arduRx); //default crc is crc32

	radioTx.SendBytes(buffer, 1000, 255, 400);
	radioRx.ReceiveBytes(rxbuffer, 1000);
	std::cout << "Ya esta" << std::endl;


}

void testrxtx3()
{
	Arduino arduTx = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you TX?\n",
			"Yes, I'm TX");
	if(!arduTx.IsOpen())
		return;

	std::cout <<"TX listo\n";
	Arduino arduRx = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you RX?\n",
			"Yes, I'm RX");
	if(!arduRx.IsOpen())
		return;

	std::cout <<"RX listo\n";

	uint32_t size = 2048;
	uint8_t buffer[size];
	uint8_t rxbuffer[size];

	for(int i=0; i< size; i++)
		buffer[i]='f';

	Radio radioTx(0,arduTx);
	Radio radioRx(1,arduRx);

	BlockRadioTransmitter fileTx(radioTx);
	BlockRadioTransmitter fileRx(radioRx);
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	while(1)
	{
		std::cout << "Enviando..." <<std::endl;
		fileTx.Send("Imagen", buffer, size, size);
		fileRx.Receive("Imagen", rxbuffer, size);
		std::cout << "Recibido!" << std::endl;
	}





}

int main() {

	testrxtx3();
	return 0;
}



void test7()
{
	SerialPortInterface serial("/dev/ttyACM9", SerialPortInterface::BAUD_115200);
	bool opened = serial.Open();
	if (! opened)
		return;
	Arduino arduino(serial);
	DataLinkFrame dlf(DataLinkFrame::crc16);
	if(DataLinkFrame::IsBigEndian())
	{
		std::cout << "Big Endian";
	}
	while(1)
	{
		arduino >> dlf;
		std::cout << "Frame recibido!" <<std::endl;
		dlf.printFrame(std::cout);
		if(dlf.checkFrame())
		{
			std::cout << "Frame correcto" << std::endl;
		}
		else
			std::cout << "Frame incorrecto" << std::endl;

		std::cout << "###############" << std::endl;
	}
}

void test9()
{
	Arduino arduino = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you TX?\n",
			"Yes, I'm TX");
	if(!arduino.IsOpen())
		return;

	unsigned char buf[5000];


	for(int i = 0; i < 5000 ; i++)
	{
		buf[i] = '1';
	}

	while(1)
	{
		arduino << "juanito";
		arduino.Write(buf, 1008, 0);

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}


void test8()
{

	Arduino arduino = Arduino::FindArduino(Arduino::BAUD_115200,
			"Hello, are you TX?\n",
			"Yes, I'm TX");

	arduino << "juanito";
	unsigned char buf[5000];


	for(int i = 0; i < 5000 ; i++)
	{
		buf[i] = '1';
	}

	arduino.Write(buf, 1500, 0);
}




void test1()
{
	uint16_t info = 0x2fce;
	std::cout << sizeof(uint16_t)<< std::endl;
	std::cout << std::hex << info<< std::endl;

	std::cout << std::hex << "0x"<<
			radiotransmission::Checksum::crc16(&info,sizeof(uint16_t))<< std::endl;

	std::cout << std::hex << "0x"<<
			radiotransmission::Checksum::crc32(&info,sizeof(uint16_t))<< std::endl;


	char cad[100] = "pepito el del pueblo 1234567890";

	std::cout << std::hex << "0x"<<
			radiotransmission::Checksum::crc16(cad,strlen(cad))<< std::endl;

	std::cout << std::hex << "0x"<<
			radiotransmission::Checksum::crc32(cad,strlen(cad))<< std::endl;

}

void test2()
{


	char str[1500] = "cagontota";

	DataLinkFrame myframe = DataLinkFrame(0x1,0x2, strlen(str),(uint8_t*)str,DataLinkFrame::crc16);

	myframe.printFrame(std::cout);

	std::cout << std::endl ;

	DataLinkFrame myframe2 = DataLinkFrame(0x1,0x2, strlen(str),(uint8_t*)str,DataLinkFrame::crc32);

	myframe2.printFrame(std::cout);

	std::cout << sizeof(unsigned long) << std::endl;
	std:: cout << std::endl;


	char cad[20] = "123456789";

	std::cout << std::hex << "0x"<<
			radiotransmission::Checksum::crc16(cad,strlen(cad))<< std::endl;

	std::cout << std::hex << "0x"<<
			radiotransmission::Checksum::crc32_cbf43926(cad,strlen(cad))<< std::endl;

	std::cout << std::hex << "0x"<<
			radiotransmission::Checksum::crc32_2dfd2d88(cad,strlen(cad))<< std::endl;

	///
	uint32_t a = 0;
	uint32_t b = ~0U;
	uint32_t c = b^a ;
	std::cout << std::hex << "0x"<< c << std::endl;
	///

	bool ok = myframe.checkFrame();
	if(ok)
		std::cout << "El frame con crc16 no contiene errores"<< std::endl;
	else std::cout << "Error en el frame con crc16"<< std::endl;

	ok = myframe2.checkFrame();
	if(ok)
		std::cout << "El frame con crc32 no contiene errores"<< std::endl;
	else std::cout << "Error en el frame con crc32"<< std::endl;
	//std::cout << myFrame.checkFrame() << std::endl;
}
