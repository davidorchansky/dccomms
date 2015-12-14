/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/thrusters_controller_ui/main_window.hpp"
#include <seabotix_thrusters_interface/ROVThrustersOrder.h>
#include <QtGlobal>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace thrusters_controller_ui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    ui.th1Sp->setRange(0,100);
    ui.th2Sp->setRange(0,100);
    ui.th3Sp->setRange(0,100);
    ui.th4Sp->setRange(0,100);
    ui.th5Sp->setRange(0,100);
    ui.th6Sp->setRange(0,100);

    ui.th1Sp->setSingleStep(1);
    ui.th2Sp->setSingleStep(1);
    ui.th3Sp->setSingleStep(1);
    ui.th4Sp->setSingleStep(1);
    ui.th5Sp->setSingleStep(1);
    ui.th6Sp->setSingleStep(1);

    QObject::connect(this, SIGNAL(thrustersControlsUpdated(seabotix_thrusters_interface::ROVThrustersOrder)), &qnode, SLOT(thrustersUiUpdated(seabotix_thrusters_interface::ROVThrustersOrder)));
    QObject::connect(&qnode, SIGNAL(newThrustersState()), this, SLOT(thrusters_ROSState_changed()));

    QObject::connect(ui.th1Sp, SIGNAL(valueChanged(int)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th2Sp, SIGNAL(valueChanged(int)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th3Sp, SIGNAL(valueChanged(int)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th4Sp, SIGNAL(valueChanged(int)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th5Sp, SIGNAL(valueChanged(int)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th6Sp, SIGNAL(valueChanged(int)), this, SLOT(thrusters_controls_valueChanged()));

    QObject::connect(ui.th1Fw, SIGNAL(toggled(bool)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th2Fw, SIGNAL(toggled(bool)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th3Fw, SIGNAL(toggled(bool)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th4Fw, SIGNAL(toggled(bool)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th5Fw, SIGNAL(toggled(bool)), this, SLOT(thrusters_controls_valueChanged()));
    QObject::connect(ui.th6Fw, SIGNAL(toggled(bool)), this, SLOT(thrusters_controls_valueChanged()));

    QObject::connect(ui.th1Stop, SIGNAL(clicked()), this, SLOT(stopThruster1()));
    QObject::connect(ui.th2Stop, SIGNAL(clicked()), this, SLOT(stopThruster2()));
    QObject::connect(ui.th3Stop, SIGNAL(clicked()), this, SLOT(stopThruster3()));
    QObject::connect(ui.th4Stop, SIGNAL(clicked()), this, SLOT(stopThruster4()));
    QObject::connect(ui.th5Stop, SIGNAL(clicked()), this, SLOT(stopThruster5()));
    QObject::connect(ui.th6Stop, SIGNAL(clicked()), this, SLOT(stopThruster6()));

    QObject::connect(ui.th1Max, SIGNAL(clicked()), this, SLOT(maxThruster1()));
    QObject::connect(ui.th2Max, SIGNAL(clicked()), this, SLOT(maxThruster2()));
    QObject::connect(ui.th3Max, SIGNAL(clicked()), this, SLOT(maxThruster3()));
    QObject::connect(ui.th4Max, SIGNAL(clicked()), this, SLOT(maxThruster4()));
    QObject::connect(ui.th5Max, SIGNAL(clicked()), this, SLOT(maxThruster5()));
    QObject::connect(ui.th6Max, SIGNAL(clicked()), this, SLOT(maxThruster6()));

    QObject::connect(ui.stopAllButton, SIGNAL(clicked()), this, SLOT(stopAllThrusters()));

    publish = true;

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::stopAllThrusters(){
    stopThruster1();
    stopThruster2();
    stopThruster3();
    stopThruster4();
    stopThruster5();
    stopThruster6();

    thrusters_controls_valueChanged();
}
void MainWindow::stopThruster1() {
    ui.th1Sp->setSliderPosition(0);
}
void MainWindow::stopThruster2() {
    ui.th2Sp->setSliderPosition(0);
}
void MainWindow::stopThruster3() {
    ui.th3Sp->setSliderPosition(0);
}
void MainWindow::stopThruster4() {
    ui.th4Sp->setSliderPosition(0);
}
void MainWindow::stopThruster5() {
    ui.th5Sp->setSliderPosition(0);
}
void MainWindow::stopThruster6() {
    ui.th6Sp->setSliderPosition(0);
}

void MainWindow::maxThruster1() {
    ui.th1Sp->setSliderPosition(100);
}
void MainWindow::maxThruster2() {
    ui.th2Sp->setSliderPosition(100);
}
void MainWindow::maxThruster3() {
    ui.th3Sp->setSliderPosition(100);
}
void MainWindow::maxThruster4() {
    ui.th4Sp->setSliderPosition(100);
}
void MainWindow::maxThruster5() {
    ui.th5Sp->setSliderPosition(100);
}
void MainWindow::maxThruster6() {
    ui.th6Sp->setSliderPosition(100);
}

void MainWindow::thrusters_ROSState_changed()
{
    publish = false;
    seabotix_thrusters_interface::ROVThrustersOrder thrusters_msg = qnode.getThrustersState();

    qDebug("Nuevo estado de los motores recibido de ROS!");
    ui.th1Sp->setSliderPosition(thrusters_msg.motor1.speed);
    ui.th2Sp->setSliderPosition(thrusters_msg.motor2.speed);
    ui.th3Sp->setSliderPosition(thrusters_msg.motor3.speed);
    ui.th4Sp->setSliderPosition(thrusters_msg.motor4.speed);
    ui.th5Sp->setSliderPosition(thrusters_msg.motor5.speed);
    ui.th6Sp->setSliderPosition(thrusters_msg.motor6.speed);

    thrusters_msg.motor1.forward ? ui.th1Fw->click() : ui.th1Rv->click();
    thrusters_msg.motor2.forward ? ui.th2Fw->click() : ui.th2Rv->click();
    thrusters_msg.motor3.forward ? ui.th3Fw->click() : ui.th3Rv->click();
    thrusters_msg.motor4.forward ? ui.th4Fw->click() : ui.th4Rv->click();
    thrusters_msg.motor5.forward ? ui.th5Fw->click() : ui.th5Rv->click();
    thrusters_msg.motor6.forward ? ui.th6Fw->click() : ui.th6Rv->click();

    publish = true;
}

void MainWindow::thrusters_controls_valueChanged()
{
    if(publish)
    {
        seabotix_thrusters_interface::ROVThrustersOrder thrusters_msg;

        thrusters_msg.motor1.speed = ui.th1Sp->value();
        thrusters_msg.motor2.speed = ui.th2Sp->value();
        thrusters_msg.motor3.speed = ui.th3Sp->value();
        thrusters_msg.motor4.speed = ui.th4Sp->value();
        thrusters_msg.motor5.speed = ui.th5Sp->value();
        thrusters_msg.motor6.speed = ui.th6Sp->value();

        thrusters_msg.motor1.forward = ui.th1Fw->isChecked();
        thrusters_msg.motor2.forward = ui.th2Fw->isChecked();
        thrusters_msg.motor3.forward = ui.th3Fw->isChecked();
        thrusters_msg.motor4.forward = ui.th4Fw->isChecked();
        thrusters_msg.motor5.forward = ui.th5Fw->isChecked();
        thrusters_msg.motor6.forward = ui.th6Fw->isChecked();

        Q_EMIT thrustersControlsUpdated(thrusters_msg);
    }
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "thrusters_controller_ui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "thrusters_controller_ui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace thrusters_controller_ui

