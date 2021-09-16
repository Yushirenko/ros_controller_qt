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
#include <QDebug>
#include <QMessageBox>
#include <iostream>
#include "../include/insrobo/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace insrobo {

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

    /*********************
    ** Keyboard Connect To Slot
    **********************/
    // Robot control
    connect(ui.pushButton_I,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_U,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_O,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_J,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_K,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_L,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_M,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_dot,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_point,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));

    // Camera pan control
    connect(ui.pushButton_W,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_A,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_S,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_D,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_Q,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
    connect(ui.pushButton_E,SIGNAL(clicked()),this,SLOT(keyboard_control_click()));
}

/*****************************************************************************
** LCDnumbers Display
*****************************************************************************/
// Show the speed setting on LCD
// Linear speed
void insrobo::MainWindow::on_horizontalSlider_valueChanged(int value)
{
    int linear_speed_set = ui.horizontalSlider->value();
    QString linear_speed_set_str = QString::number(linear_speed_set, 10);
    ui.lcdNumber->display(linear_speed_set_str);
}
// Angular speed
void insrobo::MainWindow::on_horizontalSlider_2_valueChanged(int value)
{
    int angular_speed_set = ui.horizontalSlider_2->value();
    QString angular_speed_set_str = QString::number(angular_speed_set, 10);
    ui.lcdNumber_2->display(angular_speed_set_str);
}

// Show the angular setting on LCD
// Camera pan yaw
void insrobo::MainWindow::on_horizontalSlider_3_valueChanged(int value)
{
    int yaw_set = ui.horizontalSlider_3->value();
    QString yaw_set_str = QString::number(yaw_set, 10);
    ui.lcdNumber_5->display(yaw_set_str);
}
// Camera pan pitch
void insrobo::MainWindow::on_horizontalSlider_4_valueChanged(int value)
{
    int pitch_set = ui.horizontalSlider_4->value();
    QString pitch_set_str = QString::number(pitch_set, 10);
    ui.lcdNumber_4->display(pitch_set_str);
}
// Camera pan raw
void insrobo::MainWindow::on_horizontalSlider_5_valueChanged(int value)
{
    int raw_set = ui.horizontalSlider_5->value();
    QString raw_set_str = QString::number(raw_set, 10);
    ui.lcdNumber_3->display(raw_set_str);
}

/*****************************************************************************
** Keyboard Control
*****************************************************************************/
// Receive keyboard signal
void MainWindow::keyboard_control_click()
{
    QPushButton* btn = qobject_cast<QPushButton*>(sender());
    char keyboard_value = btn->text().toStdString()[0];
    bool omni_mode = ui.checkBox_omni_mode->isChecked();

    int linear_speed_set = ui.horizontalSlider->value();
    int angular_speed_set = ui.horizontalSlider_2->value();
    double linear_speed = linear_speed_set * 0.01;
    double angular_speed = angular_speed_set * 0.01;

    switch(keyboard_value)
    {
        case 'u':
            qnode.set_cmd_vel_keyboard(omni_mode?'U':'u', linear_speed, angular_speed);
            break;
        case 'i':
            qnode.set_cmd_vel_keyboard(omni_mode?'I':'i', linear_speed, angular_speed);
            break;
        case 'o':
            qnode.set_cmd_vel_keyboard(omni_mode?'O':'o', linear_speed, angular_speed);
            break;
        case 'j':
            qnode.set_cmd_vel_keyboard(omni_mode?'J':'j', linear_speed, angular_speed);
            break;
        case 'k':
            qnode.set_cmd_vel_keyboard(omni_mode?'K':'k', linear_speed, angular_speed);
            break;
        case 'l':
            qnode.set_cmd_vel_keyboard(omni_mode?'L':'l', linear_speed, angular_speed);
            break;
        case 'm':
            qnode.set_cmd_vel_keyboard(omni_mode?'M':'m', linear_speed, angular_speed);
            break;
        case ',':
            qnode.set_cmd_vel_keyboard(omni_mode?'<':',', linear_speed, angular_speed);
            break;
        case '.':
            qnode.set_cmd_vel_keyboard(omni_mode?'>':'.', linear_speed, angular_speed);
            break;
    }
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
    QSettings settings("Qt-Ros Package", "insrobo");
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
    QSettings settings("Qt-Ros Package", "insrobo");
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

}  // namespace insrobo
