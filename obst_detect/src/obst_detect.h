/*******************************************************************************
 * @file obst_detect.h
 * @author James Anderson <jra798>
 * @date 1/15/12
 * @version 1.0
 * @brief pre vision dose simple vision processing needed by multipule uper 
 level modules.
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
#include <queue>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>
#include <dynamic_reconfigure/server.h>
#include <obst_detect/obst_detect_paramsConfig.h>


/***********************************************************
* Global variables
***********************************************************/

std::queue<cv::Mat>             depth_q;


/***********************************************************
* Parameters
***********************************************************/

obst_detect::obst_detect_paramsConfig params;

/***********************************************************
* Subscribers
***********************************************************/

image_transport::Subscriber     y_sobel_sub;
image_transport::Subscriber     depth_sub;


/***********************************************************
* Publishers
***********************************************************/

image_transport::Publisher      local_map_pub;


/***********************************************************
* Function prototypes
***********************************************************/

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;



