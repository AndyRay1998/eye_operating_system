#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy

import galil_command as g_command

# galil API
import sys
sys.path.append(sys.path[0] + '/../src')
import gclib_python.gclib as gclib

# use omni customed message type
from omni_msgs.msg import OmniState


def callback(data, args):

    rospy.loginfo(rospy.get_caller_id() + " python: Galil heard Omni")

    rospy.loginfo(data)

    v1 = data.velocity.x
    v2 = data.velocity.y
    v3 = data.velocity.z

    # TODO: velocity transform using jacobian
    # Galil velocity control
    # g_command.g_jog(v1, v2, v3, g)


def galil_listener(g):
    rospy.Subscriber("joint_states", OmniState, callback, g)
    rospy.spin()


if __name__ == '__main__':
    # ROS init
    rospy.init_node('Galil_overall_listener', anonymous=False)

    # retrieve parameter from .launch file
    galil_address = rospy.get_param("~Galil_address")

    # galil card connection
    # g = g_command.g_init(galil_address)
    # TODO: the following g is for test only
    g = gclib.py()
    # data listening and galil command
    try:
        galil_listener(g)
    except:
        pass
    finally:
        # Close galil connection
        # g.GClose()
        pass
