#!/usr/bin/env python3
# note: Hyperion is programmed under python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from omni_msgs.msg import OmniFeedback
import sys

# add API located in another folder
sys.path.append(sys.path[0] + '/../src')

import Hyperion_PY_API.hyperion
import Hyperion_PY_API.networkconfiguration
import Hyperion_PY_API.getspectrumandpeaksplot


def hyperion_force(address, ref_wavelen):
    '''
    return force feedback np.array([[f_x], [f_y]])
    Calculation detail is from a Chinese essay located in "/hyperion_mixed"
    '''
    try:
        # Connection of hyperion device (we may not need this)
        # networkconfiguration.hyperion_connect_config(current_address)

        # TODO: real test
        # retrieve hyperion data
        # get peaks and wavelengths from hyperion in 1*3 array
        '''
        wave_diff = getspectrumandpeaksplot.getpeaks(address, ref_wavelen)
        # Calculation referenced from essay
        lbd_m = np.sum(wave_diff) / 3
        lbd_1 = wave_diff[0] - lbd_m
        lbd_2 = wave_diff[1] - lbd_m
        lbd_3 = wave_diff[2] - lbd_m

        # matrix calculation
        k_ = np.array([[ 37.47996088,   8.99352363, -46.47348451],
                        [ 27.42708426, -43.05784321,  15.63075895]])
        # force: 2*1 matrix
        f = np.dot(k_, np.array([[lbd_1], [lbd_2], [lbd_3]]))
        return f
        '''
        pass
    except:
        # print out warning message
        print("\033[1;31;45m[WARN] Hyperion get peaks failed\033[0m")
        print("\033[1;31;45m[WARN] Stopping Hyperion script\033[0m")
        sys.exit()


# calculate reference wavelength during initialization
def wavelength_calibrate(address):
    ref_wavelen = 0
    for i in range(20):
        test = getspectrumandpeaksplot.getpeaks(address)
        ref_wavelen += np.sum(test)
    return(ref_wavelen / 20 + 1550)


def talker(pub, pub_omni, f_x=0, f_y=0, f_z=0):

    while not rospy.is_shutdown():

        # random data for debug and test
        hyperion_data =  Float32MultiArray()
        hyperion_data.data = np.random.random((2,3)).astype(np.float32).reshape([6])
        hyperion_data.layout.data_offset = 0
        # create two dimensions in the dim array
        hyperion_data.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        '''
        The stride in these dimensions refers to how many samples along the next
        higher dimension is. So in the case of the samples, there is a stride of
        4096 to the next sample. In the case of dimension zero there is no higher
        dimension so the stride is the overall size of the message data.
        '''
        # dim[0] is the vertical dimension of your matrix
        hyperion_data.layout.dim[0].label = "vertical"
        hyperion_data.layout.dim[0].size = 2
        hyperion_data.layout.dim[0].stride = 3
        # dim[1] is the horizontal dimension of your matrix
        hyperion_data.layout.dim[1].label = "horizontal"
        hyperion_data.layout.dim[1].size = 3
        hyperion_data.layout.dim[1].stride = 3

        rospy.loginfo("python: hyperion_data sent")
        pub.publish(hyperion_data)

        # f_x = hyperion_force()[0][0]
        # f_y = hyperion_force()[1][0]
        force_data = OmniFeedback()
        force_data.force.x = f_x
        force_data.force.y = f_y
        force_data.force.z = f_z

        rospy.loginfo("python: force_feedback_data sent")
        # rospy.loginfo(_hyperion_data) # for test only
        pub_omni.publish(force_data)


        rate.sleep()




if __name__ == '__main__':
    pub = rospy.Publisher('Hyperion_data', Float32MultiArray, queue_size=10)
    # TODO: test publishing force to omni phantom; OmniFeedback.msg
    pub_omni = rospy.Publisher('phantom/force_feedback', OmniFeedback, queue_size=10)
    rospy.init_node('Hyperion_talker', anonymous=False)
    rate = rospy.Rate(11) # 11hz

    # retrieve parameter from .launch file
    current_address = rospy.get_param("~Hyperion_address")

    # reference wavelength calibration
    # ref_wavelen = wavelength_calibrate(current_address)
    # TODO: test retrieve and process hyperion data
    # f_x, f_y = hyperion_force(current_address, ref_wavelen)

    try:
        talker(pub, pub_omni)
    except rospy.ROSInterruptException:
        rospy.logerr("hyperion_data error")
        pass
