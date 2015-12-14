/**
 * @file /include/thrusters_controller_ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef thrusters_controller_ui_QNODE_HPP_
#define thrusters_controller_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <seabotix_thrusters_interface/ROVThrustersOrder.h>
#include <QMutex>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thrusters_controller_ui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
        void rovOrderCallback(const seabotix_thrusters_interface::ROVThrustersOrder &msg);

        seabotix_thrusters_interface::ROVThrustersOrder getThrustersState();
public Q_SLOTS:
        void thrustersUiUpdated(seabotix_thrusters_interface::ROVThrustersOrder msg);

Q_SIGNALS:
	void loggingUpdated();
        void rosShutdown();
        void newThrustersState();

private:
	int init_argc;
	char** init_argv;
        ros::Publisher ROVThrusters_publisher;
        ros::Subscriber ROVThrusters_subscriber;
        QStringListModel logging_model;
        seabotix_thrusters_interface::ROVThrustersOrder thrusters_msg;
        QMutex mutex;
};

}  // namespace thrusters_controller_ui

#endif /* thrusters_controller_ui_QNODE_HPP_ */
