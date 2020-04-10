#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "../src/Hyperion_CPP_API/hLibrary.h"
#include "../src/hyperion_get_peaks.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <ctype.h>
#include <vector>


void hyperion_talker(ros::NodeHandle n)
{


  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("Hyperion_data", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Float32MultiArray msg;

    std::vector<float> msg_data={0,0,0,1,2,3};
    msg.data = msg_data;

    ROS_INFO("%s", "hyperion data sent");

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
}


void hyperion_commu(ros::NodeHandle nh, std::vector<double> allPeaks, std::vector<double> wavelength)
{
  // retrieve param from .launch file
  std::string current_address;
  nh.getParam("Hyperion_address", current_address);

  // use class to comunicate with hyperion
  // hyperion_operation test;
  try
  {
    ;
  }
  catch(...)
  {
    std::cout << "\033[31m [WARN] Hyperion connection failed \033[0m""" << std::endl;
    exit(0);
  }
  // test.test_print();
  // test.networkconfiguration();
  // test.networkconfiguration(current_address, "hyperion")
  // test.get_peaks(allPeaks, wavelength)

}


int main(int argc, char **argv)
{
  // ROS node initialize
  ros::init(argc, argv, "Hyperion_talker");
  ros::NodeHandle n;
  ros::NodeHandle nh("~"); // for private param in .launch

  // connect to hyperion and get data
  std::vector<double> allPeaks, wavelength;
  hyperion_commu(nh, allPeaks, wavelength);

  // ROS talker that publish data
  hyperion_talker(n);

  return 0;
}
