#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from galil_command import g_control
# use omni customed message type
from omni_msgs.msg import OmniState


def callback(data):

    rospy.loginfo(rospy.get_caller_id() + " python: Galil heard Omni")

    rospy.loginfo(data)

    v1 = data.velocity.x
    v2 = data.velocity.y
    v3 = data.velocity.z

    # TODO: uncomment and test
    # Galil velocity control in joint space
    # v1 -> theta3, v2 -> theta2, v3 -> d6
    # g.g_jog(v1, v2, v3)

    # detect button change
    if data.close_gripper == True and g.grip_state == False:
        grip_count += 1
    # record previous button state
    g.grip_state = data.close_gripper
    # TODO: determine hardware I/O number and revise galil_command.py
    # g.grip(grip_count%2)


def galil_listener():
    rospy.Subscriber("joint_states", OmniState, callback)
    rospy.spin()


if __name__ == '__main__':
    # ROS init
    rospy.init_node('Galil_overall_listener', anonymous=False)

    # retrieve parameter from .launch file
    galil_address = rospy.get_param("~Galil_address")

    # galil card connection
    g = g_control(galil_address)
    # TODO: uncomment for real test
    # g.connect()

    # data listening and galil command
    try:
        galil_listener()
    except:
        pass
    finally:
        # Close galil connection
        # TODO: uncomment for real test
        # g.disconnect()
        pass
