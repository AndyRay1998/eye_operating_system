#include "ros/ros.h"
#include "serial/serial.h"  //ROS installed package, need to revise cmakelists.txt
#include "std_msgs/Float32MultiArray.h"
#include <stdlib.h>
#include <string>

using namespace std;

// Reference: https://www.cnblogs.com/TooyLee/p/6104863.html

serial::Serial ser; // serial object declaration

void callback1(const std_msgs::Float32MultiArray::ConstPtr& msg1)
{
    ROS_INFO("YAMAHA hear Hyperion");
    // ser.write(msg->data);   // send out serial data
}

void callback2(const std_msgs::Float32MultiArray::ConstPtr& msg2)
{
    ROS_INFO("YAMAHA hear Omni");
    // ser.write(msg->data);   // send out serial data
}


void serial_connect(string port, int freq, double timeout)
{
    try
    {
    // serial setup and connect
        ser.setPort(port);
        ser.setBaudrate(freq);
        serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR("Connection to YAMAHA failed!");
        exit(0); // do clearance before exit
    }

    // Judge whether serial is open
    if(ser.isOpen())
    {
        ROS_INFO("YAMAHA serial Port initialized");
    }
    else
    {
        ROS_ERROR("YAMAHA serial not open");
        exit(0); // do clearance before exit
    }
}


int main (int argc, char** argv)
{
    // initialize ROS node
    ros::init(argc, argv, "Yamaha_listener");
    ros::NodeHandle nh("~"); // to get private params from .launch file
    ros::NodeHandle n;

    ros::Subscriber write_sub1 = n.subscribe("Hyperion_data", 1000, callback1);
    ros::Subscriber write_sub2 = n.subscribe("Omni_data", 1000, callback2);

    // retrieve params from .launch file
    string _address;
    int _freq;
    double _timeout;
    nh.getParam("Yamaha_address", _address);
    nh.getParam("Yamaha_freq", _freq);
    nh.getParam("Yamaha_timeout", _timeout);
    ROS_INFO("get Yamaha_freq from .launch file:  %d", _freq);

    // connect to YAMAHA
    // serial_connect(_address, _freq, _timeout);

    //指定循环的频率
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
