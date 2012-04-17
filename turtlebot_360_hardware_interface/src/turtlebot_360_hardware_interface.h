/*******************************************************************************
 * @file turtlebot_360_hardware_interface.h
 * @author James Anderson <jra798>
 * @date 1/14/12
 * @version 1.0
 * @computes and sends velocity comands to motors based off x y and rotation will also send back any sensor data pulled from the base
 ******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include <geometry_msgs/Twist.h>
#include <turtlebot_360_hardware_interface/BaseData.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <turtlebot_360_hardware_interface/turtlebot_360_hardware_interface_paramsConfig.h>
#include "serial_port.h"

/***********************************************************
* defines
***********************************************************/
//rate for comunicating with hardware hertz
#define SENSOR_RATE 30
#define ODOM_STATIC_COVARIANCE 100.0

/***********************************************************
* Global variables
***********************************************************/
bool watchdog_tripped_ = false;
ros::Time watchdog_timer_;
bool motors_enabled_ = false;
serial_port motor_port;

//variables for max accel
ros::Time vel_last_time_;
double last_wheel_speed_[3];

//odometry stuff
ros::Time g_odom_last_time;
double g_odom_x;
double g_odom_y;
double g_odom_th;

double g_vx;
double g_vy;
double g_v_theta;

geometry_msgs::TransformStamped odom_trans;

//create message
nav_msgs::Odometry odom_msg;

/***********************************************************
* Parameters
***********************************************************/

turtlebot_360_hardware_interface::turtlebot_360_hardware_interface_paramsConfig params;

/***********************************************************
* Subscribers
***********************************************************/

ros::Subscriber     twist_sub;


/***********************************************************
* Publishers
***********************************************************/

ros::Publisher      sensor_pub;
ros::Publisher      odom_pub;


/***********************************************************
* Function prototypes
***********************************************************/
static bool setVelocity(double x_base, double y_base, double theta_base);
static bool setMotor(int &motor_num, double &ticks_per_sec);
static bool killMotors();
static bool initMotors();
static void publishSensorData();
static void updateOdomMsg(double, double, double);





