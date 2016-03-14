/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/thrusters_controller_ui/qnode.hpp"
#include <irs_rov_thrusters/ThrustersOrder.h>
#include <QDebug>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thrusters_controller_ui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

irs_rov_thrusters::ThrustersOrder QNode::getThrustersState()
{
    irs_rov_thrusters::ThrustersOrder msg;
    msg = thrusters_msg;
    return msg;
}

void QNode::rovOrderCallback(const irs_rov_thrusters::ThrustersOrder &msg)
{
    qDebug("rovOrderCallback!");
    std::stringstream logging_msg;
/*
    logging_msg << "\nMotor 1:\n\tspeed = " << QString::number(msg.motor1.speed).toStdString() <<"\tD. = " << (msg.motor1.forward ? "Forward" : "Reverse")
                << "\nMotor 2:\n\tspeed = " << QString::number(msg.motor2.speed).toStdString() <<"\tD. = " << (msg.motor2.forward ? "Forward" : "Reverse")
                << "\nMotor 3:\n\tspeed = " << QString::number(msg.motor3.speed).toStdString() <<"\tD. = " << (msg.motor3.forward ? "Forward" : "Reverse")
                << "\nMotor 4:\n\tspeed = " << QString::number(msg.motor4.speed).toStdString() <<"\tD. = " << (msg.motor4.forward ? "Forward" : "Reverse")
                << "\nMotor 5:\n\tspeed = " << QString::number(msg.motor5.speed).toStdString() <<"\tD. = " << (msg.motor5.forward ? "Forward" : "Reverse")
                << "\nMotor 6:\n\tspeed = " << QString::number(msg.motor6.speed).toStdString() <<"\tD. = " << (msg.motor6.forward ? "Forward" : "Reverse");
*/

    if(msg.speeds.size() == 6)
    {
        logging_msg << "\nMotor 1:\n\tspeed = " << (int)msg.speeds[0]
                << "\nMotor 2:\n\tspeed = " <<(int) msg.speeds[1]
                << "\nMotor 3:\n\tspeed = " << (int)msg.speeds[2]
                << "\nMotor 4:\n\tspeed = " <<(int)msg.speeds[3]
                << "\nMotor 5:\n\tspeed = " <<(int) msg.speeds[4]
                << "\nMotor 6:\n\tspeed = " <<(int) msg.speeds[5] <<"\n";
    }
    else
    {
        logging_msg << "QNode::rovOrderCallback: tamano incorrecto: n. motores = " << msg.speeds.size() << "\n";
    }
    mutex.lock();
    log(Info,logging_msg.str());
    thrusters_msg = msg;
    Q_EMIT newThrustersState();
    mutex.unlock();
}


bool QNode::init() {
	ros::init(init_argc,init_argv,"thrusters_controller_ui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
        ROVThrusters_publisher = n.advertise<irs_rov_thrusters::ThrustersOrder>("ROVThrusters", 1);
        ROVThrusters_subscriber = n.subscribe("ROVThrusters", 1, &QNode::rovOrderCallback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"thrusters_controller_ui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
        ROVThrusters_publisher = n.advertise<irs_rov_thrusters::ThrustersOrder>("ROVThrusters", 1);
        ROVThrusters_subscriber = n.subscribe("ROVThrusters", 1, &QNode::rovOrderCallback, this);
	start();
	return true;
}

void QNode::run() {
        ros::Rate loop_rate(10);
	int count = 0;
        while ( ros::ok() ) {


                mutex.lock();
               // ROVThrusters_publisher.publish(thrusters_msg);
                mutex.unlock();


		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &_msg) {
        int rleft = logging_model.rowCount() - 9;
        if(rleft > 0)
            logging_model.removeRows(0,rleft);

        logging_model.insertRows(logging_model.rowCount(),1);

        if ( ! ros::master::check() ) {
                std::string msg = "[ERROR] no conectado a ROS master" + _msg;
                QVariant new_row2(QString(msg.c_str()));;
                logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row2);
                Q_EMIT loggingUpdated(); // used to readjust the scrollbar
                return;
        }
        std::stringstream logging_model_msg;
        std::string msg = _msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
                                ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
        }
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}


void QNode::thrustersUiUpdated(irs_rov_thrusters::ThrustersOrder msg)
{
    std::stringstream logging_msg;
/*
    logging_msg << "\nMotor 1:\n\tspeed = " << QString::number(msg.motor1.speed).toStdString() <<"\tD. = " << (msg.motor1.forward ? "Forward" : "Reverse")
                << "\nMotor 2:\n\tspeed = " << QString::number(msg.motor2.speed).toStdString() <<"\tD. = " << (msg.motor2.forward ? "Forward" : "Reverse")
                << "\nMotor 3:\n\tspeed = " << QString::number(msg.motor3.speed).toStdString() <<"\tD. = " << (msg.motor3.forward ? "Forward" : "Reverse")
                << "\nMotor 4:\n\tspeed = " << QString::number(msg.motor4.speed).toStdString() <<"\tD. = " << (msg.motor4.forward ? "Forward" : "Reverse")
                << "\nMotor 5:\n\tspeed = " << QString::number(msg.motor5.speed).toStdString() <<"\tD. = " << (msg.motor5.forward ? "Forward" : "Reverse")
                << "\nMotor 6:\n\tspeed = " << QString::number(msg.motor6.speed).toStdString() <<"\tD. = " << (msg.motor6.forward ? "Forward" : "Reverse");
*/
    if(msg.speeds.size() == 6)
    {
        logging_msg << "\nMotor 1:\n\tspeed = "  << (int)msg.speeds[0]
                << "\nMotor 2:\n\tspeed = " <<(int) msg.speeds[1]
                << "\nMotor 3:\n\tspeed = " <<(int) msg.speeds[2]
                << "\nMotor 4:\n\tspeed = " <<(int) msg.speeds[3]
                << "\nMotor 5:\n\tspeed = " <<(int) msg.speeds[4]
                << "\nMotor 6:\n\tspeed = " <<(int)msg.speeds[5] <<"\n";
    }
    else
    {
        logging_msg << "QNode::thrustersUiUpdated: tamano incorrecto: n. motores = " << msg.speeds.size() << "\n";
    }
    mutex.lock();
    log(Info,logging_msg.str());
    thrusters_msg = msg;
    ROVThrusters_publisher.publish(thrusters_msg);
    mutex.unlock();
}

}  // namespace thrusters_controller_ui
