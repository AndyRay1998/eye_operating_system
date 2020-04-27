#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from galil_command import g_control
# galil API
import sys
sys.path.append(sys.path[0] + '/../src')
import gclib_python.gclib as gclib
# use omni customed message type
from omni_msgs.msg import OmniState


def callback(data):

    rospy.loginfo(rospy.get_caller_id() + " python: Galil heard Omni")

    rospy.loginfo(data)

    v1 = data.velocity.x
    v2 = data.velocity.y
    v3 = data.velocity.z

    # TODO: velocity transform using jacobian
    # Galil velocity control
    # g.g_jog(v1, v2, v3)


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
    # g.connect()

    # data listening and galil command
    try:
        galil_listener()
    except:
        pass
    finally:
        # Close galil connection
        # g.disconnect()
        pass
