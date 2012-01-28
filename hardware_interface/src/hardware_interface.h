/*******************************************************************************
 * @file hardware_interface.h
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
#include <hardware_interface/BaseData.h>

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/hardware_interface_paramsConfig.h>

/***********************************************************
* defines
***********************************************************/
//rate for comunicating with hardware hertz
#define SENSOR_RATE 30

/***********************************************************
* Global variables
***********************************************************/
bool watchdog_tripped_ = false;
ros::Time watchdog_timer_;
bool motors_enabled_ = false;

/***********************************************************
* Parameters
***********************************************************/

hardware_interface::hardware_interface_paramsConfig params;

/***********************************************************
* Subscribers
***********************************************************/

ros::Subscriber     twist_sub;


/***********************************************************
* Publishers
***********************************************************/

ros::Publisher      sensor_pub;


/***********************************************************
* Function prototypes
***********************************************************/
static bool setVelocity(double x_base, double y_base, double theta_base);
static bool setMotor(int &motor_num, double &ticks_per_sec);
static bool killMotors();
static bool initMotors();
static void publishSensorData();





