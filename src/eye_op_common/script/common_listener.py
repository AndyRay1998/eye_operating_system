#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py
import numpy as np
import rospy

# use omni custom message type; revised CmakeLists.txt
from omni_msgs.msg import OmniFeedback


def callback(force_msg):
    rospy.loginfo(rospy.get_caller_id() + " python: Common heard force Omni")
    f_x = force_msg.force.x
    f_y = force_msg.force.y
    f_z = force_msg.force.z
    print(f_x, f_y, f_z)



def common_listener():
    print('common listener started')
    rospy.init_node('common_listener', anonymous=False)
    rospy.Subscriber("phantom/force_feedback", OmniFeedback, callback)

    rospy.spin()


if __name__ == '__main__':
    common_listener()
