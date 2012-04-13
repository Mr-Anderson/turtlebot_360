/*******************************************************************************
 * @file mesh_bot_control.cpp
 * @author James Anderson <jra798>
 * @version 1.1
 * @date 1/21/12
 * @brief controlls which output goes to motors 
 ******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include <geometry_msgs/Twist.h>
#include <wiimote/State.h>
#include <wiimote/LEDControl.h>
#include <wiimote/RumbleControl.h>
#include <wiimote/TimedSwitch.h>
#include <joy/Joy.h>
#include <sound_play/SoundRequest.h>


/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <mesh_bot_control/mesh_bot_control_ParamsConfig.h>

/***********************************************************
* Defines
***********************************************************/
#define MSG_BTN_1      0
#define MSG_BTN_2      1
#define MSG_BTN_A      4
#define MSG_BTN_B      5
#define MSG_BTN_PLUS   2
#define MSG_BTN_MINUS  3
#define MSG_BTN_LEFT   6
#define MSG_BTN_RIGHT  7
#define MSG_BTN_UP     8
#define MSG_BTN_DOWN   9
#define MSG_BTN_HOME   10

//start at +11 in togg
#define MSG_BTN_Z      0
#define MSG_BTN_C      1

//start at +13 in togg
#define TOGG_CLASSIC             13
#define MSG_CLASSIC_BTN_X        0
#define MSG_CLASSIC_BTN_Y        1
#define MSG_CLASSIC_BTN_A        2
#define MSG_CLASSIC_BTN_B        3
#define MSG_CLASSIC_BTN_PLUS     4
#define MSG_CLASSIC_BTN_MINUS    5
#define MSG_CLASSIC_BTN_LEFT     6
#define MSG_CLASSIC_BTN_RIGHT    7
#define MSG_CLASSIC_BTN_UP       8
#define MSG_CLASSIC_BTN_DOWN     9
#define MSG_CLASSIC_BTN_HOME     10
#define MSG_CLASSIC_BTN_L        11
#define MSG_CLASSIC_BTN_R        12
#define MSG_CLASSIC_BTN_ZL       13
#define MSG_CLASSIC_BTN_ZR       14

#define TIMEOUT_TIME             1


/***********************************************************
* Subscribers
***********************************************************/

ros::Subscriber                 nav_sub;
ros::Subscriber                 classic_sub;
ros::Subscriber                 wiimote_sub;



/***********************************************************
* Publishers
***********************************************************/

ros::Publisher                  wiimote_led_pub;
ros::Publisher                  wiimote_rum_pub;
ros::Publisher                  motor_pub;
ros::Publisher                  sound_pub;




/***********************************************************
* Global variables
***********************************************************/

//enumorator for robot mode
enum 
Mode
{
    standby,
    remote_wiimote,
    autonomous_mapping,
};

Mode mode_;



//storage for wiimote toggle bools 
//last one is for the disconect and starts empty
bool wii_togg[30] ;





                


ros::Time                       home_press_begin;


geometry_msgs::Twist            wii_twist;
geometry_msgs::Twist            nav_twist;
bool                            wii_updated;
bool                            nav_updated;


bool                            wii_dis = true;
bool                            estop_togg = false;
bool                            done_togg = false;
bool                            classic_connected = false;
bool                            wiimote_init = true;
bool                            robot_init = true;

ros::Time                       wii_timeout ;
ros::Time                       classic_timeout ;


mesh_bot_control::mesh_bot_control_ParamsConfig params;



/***********************************************************
* Namespace Changes
***********************************************************/


/***********************************************************
* Function prototypes
***********************************************************/
static void change_mode(Mode new_mode);
static void say(std::string );
static void play(std::string );
static void stop_robot();
static bool check_togg(bool, int);


