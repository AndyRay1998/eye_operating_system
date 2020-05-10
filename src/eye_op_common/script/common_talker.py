#!/usr/bin/env python3

import rospy
import numpy as np
from omni_msgs.msg import OmniState
from sensor_msgs.msg import JointState


def talker(pub, pub1):
    common_data = OmniState()
    common_data.velocity.x = 1.0
    common_data.velocity.y = 2.0
    common_data.velocity.z = 3.0

    joint_data = JointState()
    joint_data.position = [2.555, 2.555, 2.555, 2.555, 2.555, 2.555]

    while not rospy.is_shutdown():
        rospy.loginfo("python: common_data sent")
        # rospy.loginfo(_hyperion_data) # for test only
        pub.publish(common_data)
        pub1.publish(joint_data)
        rate.sleep()


if __name__ == '__main__':
    pub = rospy.Publisher('phantom/state', OmniState, queue_size=10)
    pub1 = rospy.Publisher('phantom/joint_state', JointState, queue_size=10)
    rospy.init_node('common_talker', anonymous=False)
    rate = rospy.Rate(2) # 2hz

    try:
        talker(pub, pub1)
    except rospy.ROSInterruptException:
        rospy.logerr("common_data error")
