//============================================================================
// Name        : TX.cpp
// Author      : Diego
// Version     :
// Copyright   :
// Description : Sends an image per bucle iteration
//============================================================================

#include <iostream>
#include <Arduino.h>
#include <Radio.h>
#include <BlockRadioTransmitter.h>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>			// std::chrono::seconds
#include <stdint.h>
#include <RadioException.h>
#include <stdlib.h>
#include <cstdio>
#include <cstring>
#include <ros/ros.h>
#include <seabotix_thrusters_interface/ROVThrustersOrder.h>

#include <pthread.h>
#include <list>
#include <string>

using namespace radiotransmission;

pthread_mutex_t lock;
seabotix_thrusters_interface::ROVThrustersOrder motores_rov;
Arduino arduTx;

void rovOrderCallback(const seabotix_thrusters_interface::ROVThrustersOrder &msg)
{
	ROS_INFO("Nueva orden para los motores del ROV");
	pthread_mutex_lock(&lock);
	motores_rov = msg;
	pthread_mutex_unlock(&lock);
}

void getMotorOrderByNumber(int n, uint8_t * speed, bool * s)
{
	switch(n)
	{
		case 1:
			*speed = motores_rov.motor1.speed;
			*s = motores_rov.motor1.forward;
			break;
		case 2:
			*speed = motores_rov.motor2.speed;
			*s = motores_rov.motor2.forward;
			break;
		case 3:
			*speed = motores_rov.motor3.speed;
			*s = motores_rov.motor3.forward;
			break;
		case 4:
			*speed = motores_rov.motor4.speed;
			*s = motores_rov.motor4.forward;
			break;
		case 5:
			*speed = motores_rov.motor5.speed;
			*s = motores_rov.motor5.forward;
			break;
		case 6:
			*speed = motores_rov.motor6.speed;
			*s = motores_rov.motor6.forward;
			break;
	}

}

std::list<std::string> buildCommands()
{
	pthread_mutex_lock(&lock);
	std::list<std::string> orders;

	bool m1=false, m2=false, m3=false, m4=false,
	m5=false, m6=false;

	for(int i = 0; i < 6; i++)
	{
		std::string order;
		uint8_t speed;
		bool s;

		getMotorOrderByNumber(i,&speed,&s);

		if(s) order = "fw ";
		else  order = "rv ";

		for(int c = i+1; c < 6; c++)
		{
			uint8_t speed2;
			bool s2;

			getMotorOrderByNumber(c,&speed, &s);

			bool comp = false;
			switch(c)
			{
				case 1:
					if(!m1) {comp=true; m1=true;}
					break;
				case 2:
					if(!m1) {comp=true; m1=true;}
					break;
				case 3: 
					if(!m1) {comp=true; m1=true;}
					break;
				case 4:
					if(!m1) {comp=true; m1=true;}
					break;
				case 5:
					if(!m1) {comp=true; m1=true;}
					break;
				case 6:
					if(!m1) {comp=true; m1=true;}
					break;
			}

		}

	}
	pthread_mutex_unlock(&lock);
}

void* transmitterInterface(void * r)
{
	ROS_INFO("Interfaz con el transmisor creada.");
	Radio * radioTx = (Radio*) r;
	int frameSize = 100;//atoi(argv[2]);
	int milis = 5000;//atoi(argv[3]);

	while(1)
	{
		try
		{
			ROS_INFO("Enviando ordenes a los motores...");
		//	radioTx->SendBytes(stop, strlen(stop), 255, frameSize, milis);
			std::this_thread::sleep_for(std::chrono::milliseconds(milis));
		}
		catch(RadioException& e) //Control de excepciones
		{

			switch(e.code)
			{
				case RADIO_TXLINEDOWN: //Se ha perdido la comunicación con la arduino transmisora
					std::cout << "Intentando reconectar con TX..." << std::endl << std::flush;
					while(!arduTx.TryReconnect()){};
					std::cout << "Éxito!" << std::endl << std::flush;
					break;
				case RADIO_TIMEOUT:
					std::cout << "TIMEOUT!" << std::endl << std::flush;
					break;
				default:
					std::cout << "CUIDAO!" << std::endl << std::flush;
					break;
			}
		}
		catch(std::exception & e)
		{
			std::cout <<"CUIDADO!" <<std::endl  << std::flush;

		}
		catch(int e)
		{
			std::cout <<"CUIDADO!" <<std::endl  << std::flush;

		}
	}

}

int main(int argc, char ** argv) {

	ros::init(argc, argv, "ROVThrustersControlInterface");

	ros::NodeHandle n;	

	ros::Subscriber sub = n.subscribe("ROVThrusters", 1000, rovOrderCallback);
	try
	{
		arduTx = Arduino::FindArduino(Arduino::BAUD_115200,
				"Hello, are you TX?\n",
				"Yes, I'm TX");

		if(!arduTx.IsOpen())
		{
			std::cerr << "No ha sido posible encontrar la arduino" << std::endl;
			exit(4);
		}

		std::cout <<"TX listo\n";

		Radio radioTx(0,arduTx, Radio::fcsType::crc16);
		
		if (pthread_mutex_init(&lock, NULL) != 0)
		{
			printf("\n mutex init failed\n");
			return 1;
		}

		pthread_t thread_id;
		ROS_INFO("Creando interfaz con el transmisor RF...");	
		if( pthread_create( &thread_id , NULL ,  transmitterInterface , (void*) &radioTx) < 0)
		{
			perror("could not create thread");
			ROS_ERROR("No se pudo crear la interfaz con el transmisor RF");
			return 1;
		}


	}catch(RadioException& e)
	{
		switch(e.code)
		{
			case RADIO_TXLINEDOWN:
				std::cout << "CUIDAO TX!" << std::endl << std::flush;
				break;
			default:
				std::cout << "CUIDADO!" << std::endl << std::flush;
				break;
		}
		std::cout << "Radio Exception: " << e.what() << std::endl << std::flush;
		exit(1);
	}

	ros::spin();


	return 0;
}




