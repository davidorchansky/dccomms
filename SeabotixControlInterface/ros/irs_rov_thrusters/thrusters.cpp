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
#include <irs_rov_thrusters/ThrustersOrder.h>

#include <pthread.h>
#include <list>
#include <string>

using namespace radiotransmission;
using namespace irs_rov_thrusters;
pthread_mutex_t lock;
ThrustersOrder thrustersOrder;
Arduino arduTx;

void showOrder(std::ostream & out, const ThrustersOrder & msg)
{
	int size = msg.speeds.size();
	for(int s = 0; s < size; s++)
	{
		out << "Thruster " << s << ": " <<  (int)msg.speeds[s] << "\n";	
	}

}
void rovOrderCallback(const ThrustersOrder &msg)
{
	ROS_INFO("Nueva orden para los motores del ROV");
	pthread_mutex_lock(&lock);
	thrustersOrder = msg;
	showOrder(std::cout, msg);
	pthread_mutex_unlock(&lock);
}


ThrustersOrder getOrder()
{
	pthread_mutex_lock(&lock);
	ThrustersOrder order = thrustersOrder;
	pthread_mutex_unlock(&lock);

	return order;
}

void* transmitterInterface(void * r)
{
	ROS_INFO("Interfaz con el transmisor creada.");
	Radio * radioTx = (Radio*) r;
	int frameSize = 100;//atoi(argv[2]);
	int milis = 90;//atoi(argv[3]);

	ThrustersOrder order;
	while(1)
	{
		try
		{
			order = getOrder();
			radioTx->SendBytes(&order.speeds[0], order.speeds.size() , 255, frameSize, milis);
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
		if( pthread_create( &thread_id , NULL ,  transmitterInterface ,  &radioTx) < 0)
		{
			perror("could not create thread");
			ROS_ERROR("No se pudo crear la interfaz con el transmisor RF");
			return 1;
		}

		ros::init(argc, argv, "ROVThrustersControlInterface");

		ros::NodeHandle n;	

		ros::Subscriber sub = n.subscribe("ROVThrusters", 1, rovOrderCallback);

		ros::spin();


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

	return 0;
}




