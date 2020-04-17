#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py
import numpy as np
import rospy

# use omni custom message type; revised CmakeLists.txt
from omni_msgs.msg import OmniState
from omni_msgs.msg import OmniFeedback

# calculate jacobian matrix
import eye_op_jacobian as jacob


def speed_control_O30(v_x, v_y, v_z):

    '''
    This function utilize velocities of omni to output velocities of first three
    joints of eye_op_robot
    '''
    # TODO: retrieve real joint values of robots
    # persai1, persai2 = getValue()
    persai1 = persai2 = 30
    jacob_O30 = jacob.cal_jacobian_O30(persai1, persai2)
    value_O30 = jacob.cal_speed_control(v_x, v_y, v_z, jacob_O30)

    return value_O30


def callback1(state_msg):
    rospy.loginfo(rospy.get_caller_id() + " python: Common heard Omni")
    vel_x = state_msg.velocity.x
    vel_y = state_msg.velocity.y
    vel_z = state_msg.velocity.z
    print(vel_x, vel_y, vel_z)

    result = speed_control_O30(vel_x, vel_y, vel_z)

    print(result)


def callback2(force_msg):
    rospy.loginfo(rospy.get_caller_id() + " python: Common heard force Omni")
    f_x = force_msg.force.x
    f_y = force_msg.force.y
    f_z = force_msg.force.z
    print(f_x, f_y, f_z)



def common_listener():
    print('common listener started')
    rospy.init_node('common_listener', anonymous=False)
    rospy.Subscriber("joint_states", OmniState, callback1)
    rospy.Subscriber("phantom/force_feedback", OmniFeedback, callback2)

    rospy.spin()


if __name__ == '__main__':
    common_listener()
