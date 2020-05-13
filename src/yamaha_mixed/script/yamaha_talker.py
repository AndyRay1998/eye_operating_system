#!/usr/bin/env python3
# REMEMBER: chmod +x XX.py

import rospy
from std_msgs.msg import Float32MultiArray
from yamaha_serial import command # command sending class


def yamaha_talker():
    '''
    return joint positions
    '''
    pub_yamaha = rospy.Publisher('yamaha/position', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(11) # 11hz
    while not rospy.is_shutdown():
        # TODO: test and retrieve value of joints in degree/mm unit
        # yamaha.where()
        # data = yamaha.ser_read()
        data = [55, 66, 77]
        position = Float32MultiArray(data=data)
        pub_yamaha.publish(position)
        rate.sleep()


if __name__ == '__main__':
    # ROS init
    rospy.init_node('yamaha_talker', anonymous=False)

    # get params from ros.launch file
    address = rospy.get_param("~Yamaha_address")
    freq = rospy.get_param("~Yamaha_freq")
    timeout = rospy.get_param("~Yamaha_timeout")

    # TODO: yamaha connect and data transmission
    # init an instance of yamaha class
    yamaha = command(address, freq, timeout)
    # yamaha.connect()

    yamaha_talker()

    # Close connection
    # TODO: uncomment for real test
    # yamaha.ser_close()
