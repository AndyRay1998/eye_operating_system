#!/usr/bin/env python3
# note: Hyperion is programmed under python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import sys

# add API located in another folder
sys.path.append(sys.path[0] + '/../src')

import Hyperion_PY_API.hyperion
import Hyperion_PY_API.networkconfiguration
import Hyperion_PY_API.getspectrumandpeaksplot


def talker(pub):

    while not rospy.is_shutdown():

        # random data for debug and test
        _hyperion_data =  Float32MultiArray()
        _hyperion_data.data = np.random.random((2,3)).astype(np.float32).reshape([6])
        _hyperion_data.layout.data_offset = 0
        # create two dimensions in the dim array
        _hyperion_data.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        '''
        The stride in these dimensions refers to how many samples along the next
        higher dimension is. So in the case of the samples, there is a stride of
        4096 to the next sample. In the case of dimension zero there is no higher
        dimension so the stride is the overall size of the message data.
        '''
        # dim[0] is the vertical dimension of your matrix
        _hyperion_data.layout.dim[0].label = "vertical"
        _hyperion_data.layout.dim[0].size = 2
        _hyperion_data.layout.dim[0].stride = 3
        # dim[1] is the horizontal dimension of your matrix
        _hyperion_data.layout.dim[1].label = "horizontal"
        _hyperion_data.layout.dim[1].size = 3
        _hyperion_data.layout.dim[1].stride = 3

        rospy.loginfo("python: hyperion_data sent")
        # rospy.loginfo(_hyperion_data) # for test only
        pub.publish(_hyperion_data)
        rate.sleep()

if __name__ == '__main__':

    pub = rospy.Publisher('Hyperion_data', Float32MultiArray, queue_size=10)
    rospy.init_node('Hyperion_talker', anonymous=False)
    rate = rospy.Rate(11) # 11hz

    current_address = rospy.get_param("~Hyperion_address")

    # networkconfiguration.hyperion_connect_config(current_address)
    try:
        # get peaks and wavelengths from hyperion as numpy.array float32
        # hyperion_data = getspectrumandpeaksplot.getpeaks(current_address)
        pass
    except:
        # print out warning message
        print("\033[1;31;45m[WARN] Hyperion get peaks failed\033[0m")
        print("\033[1;31;45m[WARN] Stopping Hyperion script\033[0m")
        sys.exit()

    try:
        talker(pub)
    except rospy.ROSInterruptException:
        rospy.logerr("hyperion_data error")
        pass
