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
#include <seabotix_thrusters_interface/ROVThrustersOrder.h>
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

bool QNode::init() {
	ros::init(init_argc,init_argv,"thrusters_controller_ui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
        ROVThrusters_publisher = n.advertise<seabotix_thrusters_interface::ROVThrustersOrder>("ROVThrusters", 1000);
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
        ROVThrusters_publisher = n.advertise<seabotix_thrusters_interface::ROVThrustersOrder>("ROVThrusters", 1000);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

                /*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);

                */
                mutex.lock();
                ROVThrusters_publisher.publish(thrusters_msg);
                mutex.unlock();
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
        if ( ! ros::master::check() ) {
                logging_model_msg << "[ERROR] no conectado a ROS master";
                QVariant new_row(QString(logging_model_msg.str().c_str()));
                logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
                logging_model.insertRows(logging_model.rowCount(),1);
                QVariant new_row2(QString(msg.c_str()));
                logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row2);
                Q_EMIT loggingUpdated(); // used to readjust the scrollbar
                return;
        }
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


void QNode::thrustersUiUpdated(seabotix_thrusters_interface::ROVThrustersOrder msg)
{
    std::stringstream logging_msg;

    logging_msg << "\nMotor 1:\n\tspeed = " << QString::number(msg.motor1.speed).toStdString() <<"\tD. = " << (msg.motor1.forward ? "Forward" : "Reverse")
                << "\nMotor 2:\n\tspeed = " << QString::number(msg.motor2.speed).toStdString() <<"\tD. = " << (msg.motor2.forward ? "Forward" : "Reverse")
                << "\nMotor 3:\n\tspeed = " << QString::number(msg.motor3.speed).toStdString() <<"\tD. = " << (msg.motor3.forward ? "Forward" : "Reverse")
                << "\nMotor 4:\n\tspeed = " << QString::number(msg.motor4.speed).toStdString() <<"\tD. = " << (msg.motor4.forward ? "Forward" : "Reverse")
                << "\nMotor 5:\n\tspeed = " << QString::number(msg.motor5.speed).toStdString() <<"\tD. = " << (msg.motor5.forward ? "Forward" : "Reverse")
                << "\nMotor 6:\n\tspeed = " << QString::number(msg.motor6.speed).toStdString() <<"\tD. = " << (msg.motor6.forward ? "Forward" : "Reverse");

    log(Info,logging_msg.str());
    mutex.lock();
    thrusters_msg = msg;
    ROVThrusters_publisher.publish(thrusters_msg);
    mutex.unlock();
}

}  // namespace thrusters_controller_ui
