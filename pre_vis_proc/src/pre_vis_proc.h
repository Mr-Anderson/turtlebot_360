/*******************************************************************************
 * @file pre_vis_proc.h
 * @author James Anderson <jra798>
 * @date 1/14/12
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
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>
#include <dynamic_reconfigure/server.h>
#include <pre_vis_proc/pre_vis_proc_paramsConfig.h>


/***********************************************************
* Global variables
***********************************************************/



/***********************************************************
* Parameters
***********************************************************/

pre_vis_proc::pre_vis_proc_paramsConfig params;

/***********************************************************
* Subscribers
***********************************************************/

image_transport::Subscriber     depth_sub;


/***********************************************************
* Publishers
***********************************************************/

image_transport::Publisher      x_sobel_pub;
image_transport::Publisher      y_sobel_pub;


/***********************************************************
* Function prototypes
***********************************************************/

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;



