#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py
import numpy as np
import rospy
from omni_msgs.msg import OmniState
import eye_op_jacobian

def callback(state_msg):
    rospy.loginfo(rospy.get_caller_id() + " python: Common heard Omni")
    vel_x = state_msg.velocity.x
    vel_y = state_msg.velocity.y
    vel_z = state_msg.velocity.z




def common_listener():
    print('common listener started')
    rospy.init_node('common_listener', anonymous=False)
    rospy.Subscriber("joint_states", OmniState, callback)

    rospy.spin()


if __name__ == '__main__':
    print(eye_op_jacobian.calculate_jacobian_30(0, 0))
    common_listener()
