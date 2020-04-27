#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from yamaha_serial import command # command sending class
import numpy as np
from omni_msgs.msg import OmniState
import eye_op_jacobian as jacob


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " python: YAMAHA hear hyperion")
    # retreive velocities
    v_x = data.velocity.x
    v_y = data.velocity.y
    v_z = data.velocity.z
    # we will implement increment control
    # TODO: velocity transform to joint vel; joint vel to pulses
    ## incre = getVal() -> array-like
    # movei(incre)

def yamaha_listener():

    # init ROS node
    rospy.init_node("Yamaha_listener", anonymous=False)
    rospy.Subscriber("joint_states", OmniState, callback)

    # get params from ros.launch file
    address = rospy.get_param("~Yamaha_address")
    freq = rospy.get_param("~Yamaha_freq")
    timeout = rospy.get_param("~Yamaha_timeout")

    # TODO: yamaha connect and data transmission
    yamaha = command(address, freq, timeout)
    # yamaha.connect()

    rospy.spin()


if __name__ == '__main__':

    yamaha_listener()
