#!/usr/bin/env python3
# note: Hyperion is programmed under python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension



def omni_talker():
    pub = rospy.Publisher('Omni_data', Float32MultiArray, queue_size=10)
    rospy.init_node('Omni_talker', anonymous=False)
    rate = rospy.Rate(19) # 11hz

    while not rospy.is_shutdown():

        # random data for debug and test
        _Omni_data =  Float32MultiArray()
        _Omni_data.data = np.random.random((3,3)).astype(np.float32).reshape([9])
        _Omni_data.layout.data_offset = 0
        # create two dimensions in the dim array
        _Omni_data.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        '''
        The stride in these dimensions refers to how many samples along the next
        higher dimension is. So in the case of the samples, there is a stride of
        4096 to the next sample. In the case of dimension zero there is no higher
        dimension so the stride is the overall size of the message data.
        '''
        # dim[0] is the vertical dimension of your matrix
        _Omni_data.layout.dim[0].label = "vertical"
        _Omni_data.layout.dim[0].size = 3
        _Omni_data.layout.dim[0].stride = 3
        # dim[1] is the horizontal dimension of your matrix
        _Omni_data.layout.dim[1].label = "horizontal"
        _Omni_data.layout.dim[1].size = 3
        _Omni_data.layout.dim[1].stride = 3

        rospy.loginfo("python: Omni_data sent")
        # rospy.loginfo(_Omni_data) # for test only
        pub.publish(_Omni_data)
        rate.sleep()



if __name__ == '__main__':
    try:
        omni_talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Omni_data error")
        pass
