#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from std_msgs.msg import Float32MultiArray
import yamaha_serial


def callback1(data1):
    rospy.loginfo(rospy.get_caller_id() + " python: YAMAHA hear hyperion")


def callback2(data2):
    rospy.loginfo(rospy.get_caller_id() + " python: YAMAHA hear omni")


def yamaha_listener():

    # init ROS node
    rospy.init_node("Yamaha_listener", anonymous=False)
    rospy.Subscriber("Hyperion_data", Float32MultiArray, callback1)
    rospy.Subscriber("Omni_data", Float32MultiArray, callback2)

    # get params from ros.launch file
    _address = rospy.get_param("~Yamaha_address")
    _freq = rospy.get_param("~Yamaha_freq")
    _timeout = rospy.get_param("~Yamaha_timeout")

    # yamaha connect and data transmission
    # yamaha_serial.yamaha_connect(_address, _freq, _timeout)
    # yamaha_serial.yamaha_transmit()

    rospy.spin()


if __name__ == '__main__':

    yamaha_listener()
