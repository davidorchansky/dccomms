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
#include <irs_rov_thrusters/ThrustersOrder.h>
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
        void rovOrderCallback(const irs_rov_thrusters::ThrustersOrder &msg);
        QMutex mutex;
        irs_rov_thrusters::ThrustersOrder thrusters_msg;
        irs_rov_thrusters::ThrustersOrder getThrustersState();
public Q_SLOTS:
        void thrustersUiUpdated(irs_rov_thrusters::ThrustersOrder msg);

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


};

}  // namespace thrusters_controller_ui

#endif /* thrusters_controller_ui_QNODE_HPP_ */
