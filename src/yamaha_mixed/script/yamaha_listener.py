#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from yamaha_serial import command # command sending class
import numpy as np
from omni_msgs.msg import OmniState
import eye_op_jacobian as jacob
from std_msgs.msg import Float32MultiArray


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " python: YAMAHA hear hyperion")
    # retreive velocities
    v_x = data.velocity.x
    v_y = data.velocity.y
    v_z = data.velocity.z
    ## we will implement increment control
    # TODO: control mode judgement (RCM point && RCM mechanism)
    # TODO: test velocity transform to joint vel;
    # yamaha.where()
    # joint_pos = yamaha.ser_read()
    ## extract joint position from joint_pos
    joint_1 = 30
    joint_2 = -30
    jac_matrix = jacob.cal_jacobian_O30(joint_1, joint_2)
    ctrl_val = jacob.cal_speed_control(v_x, v_y, v_z, jac_matrix)
    ## here 100 is frequency of publishment indicated in omni_state.cpp
    incre = [ctrl_val[0][0]*180/3.141593/100, ctrl_val[1][0]*180/3.141593/100, ctrl_val[2][0]/100/1000]
    # movei(incre)
    yamaha_talker(incre * 100)

def yamaha_listener():
    # init ROS node
    rospy.init_node("Yamaha_listener", anonymous=False)
    rospy.Subscriber("phantom/state", OmniState, callback)

    # get params from ros.launch file
    address = rospy.get_param("~Yamaha_address")
    freq = rospy.get_param("~Yamaha_freq")
    timeout = rospy.get_param("~Yamaha_timeout")

    # TODO: yamaha connect and data transmission
    # init an instance of yamaha class
    yamaha = command(address, freq, timeout)
    # yamaha.connect()

    rospy.spin()

    return yamaha


def yamaha_talker(data):
    '''
    return speed of first 3 joints
    '''
    pub_yamaha = rospy.Publisher('yamaha/speed', Float32MultiArray, queue_size=10)

    position = Float32MultiArray(data=data)
    pub_yamaha.publish(position)


if __name__ == '__main__':

    yamaha = yamaha_listener()

    # close serial object
    yamaha.ser_close()
