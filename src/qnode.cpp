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
#include <map>
#include "../include/insrobo/qnode.hpp"
#include "../include/insrobo/main_window.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace insrobo {

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
	ros::init(init_argc,init_argv,"insrobo");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    chatter_subscriber = n.subscribe("/chatter", 1000, &QNode::chatter_callback, this);
    robot_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    //camera_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("camera_cmd_vel", 1000);
    start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"insrobo");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    chatter_subscriber = n.subscribe("/chatter", 1000, &QNode::chatter_callback, this);
	start();
	return true;
}


void QNode::chatter_callback(const std_msgs::String &msgs) {
    log(Info, "NIMALIGEBI" + msgs.data);
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
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

void QNode::set_cmd_vel_keyboard(char keyboard_value, double linear_speed, double angular_speed)
{
    // Map for robot control
    std::map<char, std::vector<int>> robotControl
    {
        // Nomal mode
        {'i', {1, 0, 0, 0}},   {'o', {1, 0, 0, -1}},  {'j', {0, 0, 0, 1}},
        {'l', {0, 0, 0, -1}},  {'u', {1, 0, 0, 1}},   {',', {-1, 0, 0, 0}},
        {'.', {-1, 0, 0, 1}},  {'m', {-1, 0, 0, -1}}, {'k', {0, 0, 0, 0}},
        // Omni mode
        {'I', {1, 0, 0, 0}},   {'O', {1, -1, 0, 0}},  {'J', {0, 1, 0, 0}},
        {'L', {0, -1, 0, 0}},  {'U', {1, 1, 0, 0}},   {'<', {-1, 0, 0, 0}},
        {'>', {-1, -1, 0, 0}}, {'M', {-1, 1, 0, 0}},  {'K', {0, 0, 0, 0}}
    };
/*
    // Map for camera control
    std::map<char, std::vector<int>> cameraPanControl
    {
        {'Q', {-1, 0, 0}},   {'W', {0, 1, 0}},    {'E', {1, 0, 0}},
        {'A', {0, 0, -1}},   {'S', {0, -1, 0}},   {'D', {0, 0, 1}}
    };
*/
    char key = keyboard_value;

    double robot_x  = robotControl[key][0];
    double robot_y  = robotControl[key][1];
    double robot_z  = robotControl[key][2];
    double robot_th = robotControl[key][3];
/*
    double camera_yaw    = cameraPanControl[key][0];
    double camera_pitch  = cameraPanControl[key][1];
    double camera_raw    = cameraPanControl[key][2];
*/
    geometry_msgs::Twist robot_twist;
    robot_twist.linear.x = robot_x * linear_speed;
    robot_twist.linear.y = robot_y * linear_speed;
    robot_twist.linear.z = robot_z * linear_speed;
    robot_twist.angular.x = 0;
    robot_twist.angular.y = 0;
    robot_twist.angular.z = robot_th * angular_speed;
/*
    geometry_msgs::Twist camera_twist;
    camera_twist.linear.x = 0;
    camera_twist.linear.y = 0;
    camera_twist.linear.z = 0;
    camera_twist.angular.x = camera_yaw;
    camera_twist.angular.y = camera_pitch;
    camera_twist.angular.z = camera_raw;
*/
    robot_cmd_vel_pub.publish(robot_twist);
    //camera_cmd_vel_pub.publish(camera_twist);
}
}  // namespace insrobo
