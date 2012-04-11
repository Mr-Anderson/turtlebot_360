/*******************************************************************************
 * @file kinect_mapper.h
 * @author James Anderson <jra798>
 * @version 1.0
 * @brief main file to exstract a mesh from RGBD camera
 ******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <MST_Edge_Detection/Edge_Detection_ParamsConfig.h>


/***********************************************************
* Global variables
***********************************************************/

sensor_msgs::Image              image;

unsigned char*                  g_output_image;

image_transport::Subscriber     image_sub;
image_transport::Publisher      image_pub;

std::string                     topic;


MST_Edge_Detection::Edge_Detection_ParamsConfig params;

/***********************************************************
* Function prototypes
***********************************************************/

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;


/***********************************************************
* Message Callbacks
***********************************************************/
