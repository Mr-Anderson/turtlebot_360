/*******************************************************************************
* @file hardware_interface.h
* @author James Anderson <jra798>, Adam Honse <amhb59, calcprogrammer1@gmail.com>
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
    setVelocity(msg->linear.x, msg->linear.y, msg->angular.z );
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

    //Initialize serial port
    motor_port.serial_open("/dev/ttyUSB1", 19200); //params.serial_port.c_str(), 19200);
    
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
        if( watchdog_timer_ < ros::Time::now() && motors_enabled_)
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
        //sqrt of 3/3 is .866025
        wheel_speed[0] = (theta_base * params.base_radius) 
                          + y_base;
        wheel_speed[1] = (theta_base * params.base_radius) 
                          - (.866025 * x_base) 
                          - (.5 * y_base);
        wheel_speed[2] = (theta_base * params.base_radius) 
                          + (.866025 * x_base) 
                          - (.5 * y_base);
        
        //display wheel velocitys
//        ROS_INFO("Driving Motors 0:%f 1:%f 2:%f m/s", 
//                  wheel_speed[0], 
//                  wheel_speed[1], 
//                  wheel_speed[2]);
        
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
                
                //if(wheel_speed[i] < params.min_speed)
                //{
                    //set motor ticks_per_sec
                    success &= setMotor(i, wheel_speed[i]);
                //}
        }
        
        return success;
}

//sets the specified motors speed in tics/sec 
//returns true if success
bool setMotor(int &motor_num, double& ticks_per_sec)
{
    //@TODO Adam this is for your code 
    // the motors are numbered counter clockwise with 0 at the front 
    //params.min_speed acceses min speed param in ticks
    char serial_cmd[6] = {0x24,0x03,(char)motor_num+1};

    int step_length = (312500.0 / abs(ticks_per_sec)); //timer frequency

    ROS_INFO("Motor %u: %f ticks per sec, output %x\n", motor_num, ticks_per_sec, step_length);
    //Step Length (16 bits)
    serial_cmd[3] = 0x01;
    serial_cmd[4] = step_length >> 8;
    serial_cmd[5] = step_length;
    motor_port.serial_write(serial_cmd, 6);

    //Step Count (16 bits)
    serial_cmd[3] = 0x02;
    serial_cmd[4] = 0x00;
    serial_cmd[5] = 0x38;
    motor_port.serial_write(serial_cmd, 6);

    //Step Direction
    serial_cmd[3] = 0x03;
    if(ticks_per_sec > 0)
    {
        serial_cmd[4] = 0x01;
    }
    else
    {
        serial_cmd[4] = 0x00;
    }
    serial_cmd[5] = 0x00;
    motor_port.serial_write(serial_cmd, 6);

    //Enable Motor if speed is greater than minimum
    serial_cmd[3] = 0x04;
    if(abs(ticks_per_sec) > 25)
    {
        serial_cmd[4] = 0x01;
    }
    else
    {
        serial_cmd[4] = 0x00;
    }
    serial_cmd[5] = 0x00;
    motor_port.serial_write(serial_cmd, 6);

    return true;
}

//disables motors during ideling returns true on success
bool killMotors()
{
    //@TODO Adam put motor shut downs here for ideling
    //you can remove the following motor stop if not needed
    setVelocity(0, 0, 0 );
    
    char serial_cmd[6] = {0x21,0x01,0x00,0x04,0x00,0x00};
    motor_port.serial_write(serial_cmd, 6);

    return true;
}

//initializes motors returns true on success
bool initMotors()
{
    //@TODO Adam put motor initialization code here
    
    //Put serial-connected board into I2C Master mode
    char serial_cmd[6] = {0x21,0x01,0x00};
    motor_port.serial_write(serial_cmd, 3);

    //Test motor
    serial_cmd[0] = 0x24;
    serial_cmd[1] = 0x03;
    serial_cmd[2] = 0x01;
    serial_cmd[3] = 0x04;
    serial_cmd[4] = 0x01;
    serial_cmd[5] = 0x00;
    motor_port.serial_write(serial_cmd, 6);

    return true;
}

//gets sensor data from base and publishes to topic
void publishSensorData()
{
    //@TODO create sensor message and topic then fill with data
    hardware_interface::BaseData msg;
    
    
    sensor_pub.publish(msg);
    
}

