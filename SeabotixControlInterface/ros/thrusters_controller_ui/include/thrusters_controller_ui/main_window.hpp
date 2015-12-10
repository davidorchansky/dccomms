/**
 * @file /include/thrusters_controller_ui/main_window.hpp
 *
 * @brief Qt based gui for thrusters_controller_ui.
 *
 * @date November 2010
 **/
#ifndef thrusters_controller_ui_MAIN_WINDOW_H
#define thrusters_controller_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <seabotix_thrusters_interface/ROVThrustersOrder.h>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace thrusters_controller_ui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function

	void showNoMasterMessage();

Q_SIGNALS:
        void thrustersControlsUpdated(seabotix_thrusters_interface::ROVThrustersOrder);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);


        void thrusters_controls_valueChanged();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace thrusters_controller_ui

#endif // thrusters_controller_ui_MAIN_WINDOW_H
