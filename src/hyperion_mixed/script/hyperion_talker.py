#!/usr/bin/env python3
# note: Hyperion is programmed under python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from omni_msgs.msg import OmniFeedback


from hyperion_command import Hcommand


def hyperion_force(address, ref_wavelen):
    '''
    return force feedback np.array([[f_x], [f_y]])
    Calculation detail is from a Chinese essay located in "/hyperion_mixed"
    '''
    try:
        # network configuration of hyperion device (we may not need this)
        # h.connect_config(current_address)

        # TODO: real test
        # retrieve hyperion data
        # get peaks and wavelengths from hyperion in 1*3 array
        '''
        wave_diff = h.getpeaks(address, ref_wavelen)
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
        test = h.getpeaks()
        ref_wavelen += np.sum(test)
    return(ref_wavelen / 20 + 1550)


def talker(pub_omni):

    while not rospy.is_shutdown():
        # TODO: uncomment for real test
        # f = hyperion_force()
        # f_x = f[0][0]
        # f_y = f[1][0]
        force_data = OmniFeedback()
        # force_data.force.x = f_x
        # force_data.force.y = f_y
        force_data.force.z = 0

        rospy.loginfo("python: force_feedback_data sent")
        # rospy.loginfo(_hyperion_data) # for test only
        pub_omni.publish(force_data)

        rate.sleep()


if __name__ == '__main__':
    # TODO: test publishing force to omni phantom; OmniFeedback.msg
    pub_omni = rospy.Publisher('phantom/force_feedback', OmniFeedback, queue_size=10)
    rospy.init_node('Hyperion_talker', anonymous=False)
    rate = rospy.Rate(11) # 11hz

    # TODO: cofirm hyperion address and revise eye_op_robot.launch
    # retrieve parameter from .launch file
    current_address = rospy.get_param("~Hyperion_address")

    # hyperion connection
    h = Hcommand(current_address)
    # reference wavelength calibration
    # TODO: uncomment for real test
    # ref_wavelen = wavelength_calibrate(current_address)
    # TODO: test retrieve and process hyperion data
    # f_x, f_y = hyperion_force(current_address, ref_wavelen)

    try:
        talker(pub_omni)
    except rospy.ROSInterruptException:
        rospy.logerr("hyperion_data error")
        pass
