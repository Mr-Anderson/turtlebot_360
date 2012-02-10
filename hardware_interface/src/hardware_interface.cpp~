/*******************************************************************************
* @file hardware_interface.h
* @author James Anderson <jra798>
* @date 1/28/12
* @version 1.0
* @computes and sends velocity comands to motors based off x y and rotation will
also send back any sensor data pulled from the base
******************************************************************************/
#include "hardware_interface.h"

/***********************************************************
* Message Callbacks
***********************************************************/

//pulls in the twist from control
void motionCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //reset watchdog
    watchdog_timer_ = ros::Time::now() + ros::Duration(params.watchdog_timeout/60);
    
    //initialize motors if needed
    if( motors_enabled_ == false )
    {
        motors_enabled_ = initMotors();
    }
    
    //calculate and set motor speeds
    setVelocity(msg->linear.x, msg->linear.x, msg->angular.z );
}



/***********************************************************
* Parameter Callbacks
***********************************************************/
//copy parameters from server
void setparamsCallback(hardware_interface::hardware_interface_paramsConfig &config, uint32_t level)
{
    // set params
    params = config;
}


/***********************************************************
* Main
***********************************************************/
//main ros code
int main(int argc, char **argv)
{
    //setup topic names
    std::string twist_topic;
    
    //setup node and image transport
    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle node;

    
    //setup dynamic reconfigure gui
    dynamic_reconfigure::Server<hardware_interface::hardware_interface_paramsConfig> srv;
    dynamic_reconfigure::Server<hardware_interface::hardware_interface_paramsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
    srv.setCallback(f);
    
    
    //get twist topic name
    twist_topic = node.resolveName("twist");

    //check to see if user has defined an image to subscribe to 
    if (twist_topic == "/twist") 
    {
        ROS_WARN("hardware_interface: twist has not been remapped! Typical command-line usage:\n"
                 "\t$ ./hardware_interface twist:=<image topic> [transport]");
    }
    

    
    //create twist subscription
    twist_sub = node.subscribe( twist_topic , 1, motionCallback  );
    
    //advertise publishers
    sensor_pub = node.advertise<hardware_interface::BaseData>("hardware_interface/sensor_data", 5);
    
    //collect motor base sensor data at this rate
    ros::Rate loop_rate(SENSOR_RATE);
    
    while (ros::ok())
    {
        //check callbacks
        ros::spinOnce();
        
        //handel timeout
        if( watchdog_timer_ < ros::Time::now() )
        {
            //have not receved a new message so stop robot
            ROS_WARN("Watchdog timed out!");
            killMotors();
            motors_enabled_ = false;
        }
        
        //collect and publish sensor data
        publishSensorData();
        
        //wait for next spin
        loop_rate.sleep();
    }
    
    return 0;
}


/***********************************************************
* Functions
***********************************************************/
bool setVelocity(double x_base, double y_base, double theta_base)
{
        //wheel speed 
        double wheel_speed[3];
        double speed_scaler = 1;
        bool success = true;
        
        //x is forward y is left z is counter clockwise
        //calculate velocity at each wheel
        wheel_speed[0] = (theta_base * params.base_radius) 
                          + y_base;
        wheel_speed[1] = (theta_base * params.base_radius) 
                          - (.5 * x_base) 
                          - (.5 * y_base);
        wheel_speed[2] = (theta_base * params.base_radius) 
                          + (.5 * x_base) 
                          - (.5 * y_base);
        
        //display wheel velocitys
        ROS_INFO("Driving Motors 0:%f 1:%f 2%f m/s", 
                  wheel_speed[0], 
                  wheel_speed[1], 
                  wheel_speed[2]);
        
        for(int i = 0; i < 3; i++)
        {
            //calculate rotation speed of each wheel
            wheel_speed[i] /= params.wheel_radius;
            
            //calculate ticks per second of each wheel
            wheel_speed[i] *= params.motor_res;
            
            //see if we need to scale back motors
            if(wheel_speed[i] > params.max_speed)
            {
                double speed_scaler_temp = params.max_speed / wheel_speed[i];
                
                if (speed_scaler_temp < speed_scaler ) 
                {
                    speed_scaler = speed_scaler_temp;
                }
            }
        }
        
        //scale and set values
        for(int i = 0; i < 3; i++)
        {
                //scale back motors
                wheel_speed[i] *= speed_scaler;
                
                //set motor ticks_per_sec
                success &= setMotor(i, wheel_speed[i]);
        }
        
        return success;
}

//sets the specified motors speed in tics/sec 
//returns true if success
bool setMotor(int &motor_num, double& ticks_per_sec)
{
    //@TODO Adam this is for your code 
    // the motors are numbered counter clockwise with 0 at the front 
    
    return true;
}

//disables motors during ideling returns true on success
bool killMotors()
{
    //@TODO Adam put motor shut downs here for ideling
    //you can remove the following motor stop if not needed
    setVelocity(0, 0, 0 );
    
    return true;
}

//initializes motors returns true on success
bool initMotors()
{
    //@TODO Adam put motor initialization code here
    
    return true;
}

//gets sensor data from base and publishes to topic
void publishSensorData()
{
    //@TODO create sensor message and topic then fill with data
    hardware_interface::BaseData msg;
    
    
    sensor_pub.publish(msg);
    
}

