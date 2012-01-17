/*******************************************************************************
* @file obst_detect.h
* @author James Anderson <jra798>
* @date 1/15/12
* @version 1.0
* @brief pre vision dose simple vision processing needed by multipule uper 
level modules.
******************************************************************************/
#include "obst_detect.h"

/***********************************************************
* Message Callbacks
***********************************************************/
/***********************************************************
* @fn sobelCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief creates a local map where as slope rises obsical
* value goes down exponetialy
* @pre takes in a ros message of a raw or cv image
* @post publishes a CV_32FC1 image using cv_bridge
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void sobelCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr_src;

    //creates an opencv copy of the image
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(msg, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    
    //create internal images
    cv_bridge::CvImage obst_map;
    
    
    //create local map with robot in center
    
    
    //move through sobel image
    //use (base_wall_value)/(abbs(y_sobel)+1)^ wall_dropoff_rate
    //this gives an inverse exponetial curve where certainty drops off
    //exponetialy as slope goes up
    //add theis values on to the map using the depth image for location
    //ignore if it is zero in the depth image
    
    
    
    
    //setup and publish message for sobel_x
    out_msg.header = cv_ptr_src->header;
    out_msg.encoding = "32FC1";
    out_msg.image = sobel_x ;
    
    x_sobel_pub.publish(out_msg.toImageMsg());
    
    //setup and publish message for sobel_y
    out_msg.header = cv_ptr_src->header;
    out_msg.encoding = "32FC1";
    out_msg.image = sobel_y ;
    
    y_sobel_pub.publish(out_msg.toImageMsg());
    
}

/***********************************************************
* @fn depthCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief pushes depth image onto queue to use in obst map
* @pre takes in a ros message of a raw or cv image
* @post publishes a CV_32FC1 image using cv_bridge
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr_src;

    //creates an opencv copy of the image
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(msg, "16UC1");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    
    //create internal images
    cv_bridge::CvImage obst_map;
    
    
    //create local map with robot in center
    
    
    //move through sobel image
    //use (base_wall_value)/(abbs(y_sobel)+1)^ wall_dropoff_rate
    //this gives an inverse exponetial curve where certainty drops off
    //exponetialy as slope goes up
    //add theis values on to the map using the depth image for location
    //ignore if it is zero in the depth image
    
    
    
    
    //setup and publish message for sobel_x
    out_msg.header = cv_ptr_src->header;
    out_msg.encoding = "32FC1";
    out_msg.image = sobel_x ;
    
    x_sobel_pub.publish(out_msg.toImageMsg());
    
    //setup and publish message for sobel_y
    out_msg.header = cv_ptr_src->header;
    out_msg.encoding = "32FC1";
    out_msg.image = sobel_y ;
    
    y_sobel_pub.publish(out_msg.toImageMsg());
    
}



/***********************************************************
* Parameter Callbacks
***********************************************************/
/***********************************************************
* @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief callback for the reconfigure gui
* @pre has to have the setup for the reconfigure gui
* @post changes the parameters
***********************************************************/
void setparamsCallback(obst_detect::obst_detect_paramsConfig &config, uint32_t level)
{
    
    // set params
    params = config;
}


/***********************************************************
* Main
***********************************************************/
/***********************************************************
* @fn main(int argc, char **argv)
* @brief starts the pre processing node
***********************************************************/
int main(int argc, char **argv)
{
    //setup topic names
    std::string y_sobel_topic;
    std::string depth_topic;
    
    //setup node and image transport
    ros::init(argc, argv, "obst_detect");
    ros::NodeHandle node;
    image_transport::ImageTransport it(node);
    
    //setup dynamic reconfigure gui
    dynamic_reconfigure::Server<obst_detect::obst_detect_paramsConfig> srv;
    dynamic_reconfigure::Server<obst_detect::obst_detect_paramsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
    srv.setCallback(f);
    
    
    //get slope topic name
    y_sobel_topic = node.resolveName("y_sobel");

    //check to see if user has defined an image to subscribe to 
    if (y_sobel_topic == "/y_sobel") 
    {
        ROS_WARN("obst_detect: image has not been remapped! Typical command-line usage:\n"
                 "\t$ ./obst_detect y_sobel:=<image topic> [transport]");
    }
    
    //get depth topic name
    depth_topic = node.resolveName("depth");

    //check to see if user has defined an image to subscribe to 
    if (depth_topic == "/depth") 
    {
        ROS_WARN("obst_detect: image has not been remapped! Typical command-line usage:\n"
                 "\t$ ./obst_detect depth:=<image topic> [transport]");
    }
    
    
    //create image subscriptions
    y_sobel_sub = it.subscribe( y_sobel_topic , 1, sobelCallback  );
    depth_sub = it.subscribe( depth_topic , 1, depthCallback  );

    //create image publishers
    local_map_pub = it.advertise( "obst_detect/local_map_pub" , 5 );

    //start node
    ros::spin();
    
    return 0;
}

