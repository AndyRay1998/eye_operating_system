#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "gclib_cpp/x_examples.h"


void chatterCallback1(const std_msgs::Float32MultiArray::ConstPtr& msg1)
{
  ROS_INFO("Galil hear Hyperion");
  ROS_INFO("I heard data of size: [%ld]", msg1->data.size());
  for (int i=0; i < msg1->data.size(); i++)
  {
    ROS_INFO("data: [%f]", msg1->data[i]);
  }

}

void chatterCallback2(const std_msgs::Float32MultiArray::ConstPtr& msg2)
{
  // ROS_INFO("I heard: [%f]", msg2->n);
  ROS_INFO("Galil hear Omni");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Galil_overall_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("Hyperion_data", 1000, chatterCallback1);
  ros::Subscriber sub2 = n.subscribe("Omni_data", 1000, chatterCallback2);

  ros::spin();

  return 0;
}
