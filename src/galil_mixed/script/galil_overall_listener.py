#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from std_msgs.msg import Float32MultiArray

import sys
sys.path.append(sys.path[0] + '/../src')

import gclib_python.example # galil api



def gclib_command():
    pass
    # to be continued

def callback1(data1):
    rospy.loginfo(rospy.get_caller_id() + " python: Galil heard Hyperion")
    rospy.loginfo(f"data length is: {len(data1.data)}")
    rospy.loginfo(data1.data)

def callback2(data2):
    rospy.loginfo(rospy.get_caller_id() + " python: Galil heard Omni")

def galil_listener():
    rospy.init_node('Galil_overall_listener', anonymous=False)
    rospy.Subscriber("Hyperion_data", Float32MultiArray, callback1)
    rospy.Subscriber("Omni_data", Float32MultiArray, callback2)

    # gclib_command()

    rospy.spin()

if __name__ == '__main__':
    global data1, data2
    galil_listener()
