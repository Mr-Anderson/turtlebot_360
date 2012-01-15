/*******************************************************************************
* @file pre_vis_proc.h
* @author James Anderson <jra798>
* @date 1/14/12
* @version 1.0
* @brief pre vision dose simple vision processing needed by multipule uper 
level modules.
******************************************************************************/
#include "pre_vis_proc.h"

/***********************************************************
* Message Callbacks
***********************************************************/
/***********************************************************
* @fn depthCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief preforms x and y sobel on images
* @pre takes in a ros message of a raw or cv image
* @post publishes a CV_32FC1 image using cv_bridge
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr_src;

    //takes in the image
    //the dept image type is unint16 so mono should work
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(msg, "mono16");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //create internal images
    cv::Mat sobel_x(cv_ptr_src->image.size(), CV_16U);
    cv::Mat sobel_y(cv_ptr_src->image.size(), CV_16U);
    cv_bridge::CvImage out_msg;
    
    
    //preforms x and y sobels
    cv::Sobel(cv_ptr_src->image, sobel_x, 1, 1, 0, params.sobel_size);
    cv::Sobel(cv_ptr_src->image, sobel_y, 1, 0, 1, params.sobel_size);

    

    //setup and publish message for sobel_x
    out_msg.header = cv_ptr_src->header;
    out_msg.encoding = "CV_16U1";
    out_msg.image = sobel_x ;
    
    x_sobel_pub.publish(out_msg.toImageMsg());
    
    //setup and publish message for sobel_y
    out_msg.header = cv_ptr_src->header;
    out_msg.encoding = "CV_16U1";
    out_msg.image = sobel_y ;
    
    y_sobel_pub.publish(out_msg.toImageMsg());
    
    //feed through the depth image if not turned off
    if(params.output_depth)
    {
        //setup and publish message for depth image
        out_msg.header = cv_ptr_src->header;
        out_msg.encoding = "CV_16U1";
        out_msg.image = cv_ptr_src->image ;
        
        depth_pub.publish(out_msg.toImageMsg());
    }
}

/***********************************************************
* @fn colorCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief preforms x and y sobel on images
* @pre takes in a ros message of a raw or cv image
* @post publishes a CV_32FC1 image using cv_bridge
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void colorCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr_src;

    //takes in the image
    //the dept image type is unint16 so mono should work
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(msg, "mono16");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv_bridge::CvImage out_msg;
    
    //feed through the color image if not turned off
    if(params.output_color)
    {
        //setup and publish message for depth image
        out_msg.header = cv_ptr_src->header;
        out_msg.encoding = "CV_16U1";
        out_msg.image = cv_ptr_src->image ;
        
        color_pub.publish(out_msg.toImageMsg());
    }
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
void setparamsCallback(pre_vis_proc::pre_vis_proc_paramsConfig &config, uint32_t level)
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
    std::string depth_topic;
    std::string color_topic;
    
    ros::init(argc, argv, "pre_vis_proc");
    ros::NodeHandle node;
    image_transport::ImageTransport it(node);
    
    //setup dynamic reconfigure gui
    dynamic_reconfigure::Server<pre_vis_proc::pre_vis_proc_paramsConfig> srv;
    dynamic_reconfigure::Server<pre_vis_proc::pre_vis_proc_paramsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
    srv.setCallback(f);
    
    
    //get depth topic name
    depth_topic = node.resolveName("depth");

    //check to see if user has defined an image to subscribe to 
    if (depth_topic == "/depth") 
    {
        ROS_WARN("pre_vis_proc: image has not been remapped! Typical command-line usage:\n"
                 "\t$ ./pre_vis_proc depth:=<image topic> [transport]");
    }
    
    //get color topic name
    color_topic = node.resolveName("color");

    //check to see if user has defined an image to subscribe to 
    if (color_topic == "/color") 
    {
        ROS_WARN("pre_vis_proc: image has not been remapped! Typical command-line usage:\n"
                 "\t$ ./pre_vis_proc color:=<image topic> [transport]");
    }
    
    //create image subscriptions
    depth_sub = it.subscribe( depth_topic , 1, depthCallback  );
    color_sub = it.subscribe( depth_topic , 1, colorCallback  );

    //create image publishers
    x_sobel_pub = it.advertise( "pre_vis_proc/x_sobel" , 5 );
    y_sobel_pub = it.advertise( "pre_vis_proc/y_sobel" , 5 );
    depth_pub = it.advertise( "pre_vis_proc/depth" , 5 );
    color_pub = it.advertise( "pre_vis_proc/color" , 5 );

    //start node
    ros::spin();
    
    return 0;
}

