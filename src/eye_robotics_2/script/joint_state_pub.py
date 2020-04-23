#!/usr/bin/env python3
'''
This script is for jacobian verification through rviz simulation.
Check the picture in eye_robotics_2 to see coordinate systems of this robot.
Launch display.launchto see configuration of each joint.
Revice v_x, v_y, v_z and roslaunch simulation.launch file.
No software position limit is inplemented, so when approaching sigularity, /
simulation becomes strange; we do not have that affliction in real life though.
'''
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

import eye_op_jacobian as jacob

# v_x, v_y, v_z are velocities in coordinate system3
def talker(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint4_2, j5, j6, v_x=5, v_y=0, v_z=0):
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz


    '''
    During this 10s time delay, you should do the following:
    roslaunch simulation.launch
    reference frame -> base_link
    add -> RobotModel
    Adjust to a suitable view to observe motion of end effector
    '''
    start = time.time()
    while (time.time()-start < 20):
        state = JointState()
        state.header = Header()
        state.header.stamp = rospy.Time.now()
        state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint4_2']
        state.position = [0, 0, 0, 0, 0, 0, 0]
        state.velocity = []
        state.effort = []
        state.header.stamp = rospy.Time.now()
        pub.publish(state)
        rate.sleep()

    while not rospy.is_shutdown():
        start = time.time()
        state = JointState()
        state.header = Header()
        state.header.stamp = rospy.Time.now()
        # joint4_2 mimic joint_5
        state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint4_2']
        state.position = [joint_1*3.141593/180, joint_2*3.141593/180, joint_3, joint_4*3.141593/180, joint_5*3.141593/180, joint_6, joint_5*3.141593/180]
        state.velocity = []
        state.effort = []

        if 1:
            # control RCM module given velocity in system6
            # map velocity from system6 to system3
            v6 = np.array([[v_x], [v_y], [v_z]])
            v3 = np.dot(jacob.cal_r_36(joint_4, j5, j6), v6)
            print(f'speed in 3: {v3}')
            v_x_3 = v3[0][0]
            v_y_3 = v3[1][0]
            v_z_3 = v3[2][0]

            joint_4, joint_5, joint_6, j5, j6 = speed_control63(v_x_3, v_y_3, v_z_3, j5, j6, joint_4, joint_5, joint_6)

        if 0:
            # control RCM point given velocity in base system
            joint_1, joint_2, joint_3 = speed_control30(v_x, v_y, v_z, joint_1, joint_2, joint_3)

        if 0:
            # control RCM module given velocity in system3
            joint_4, joint_5, joint_6, j5, j6 = speed_control63(v_x, v_y, v_z, j5, j6, joint_4, joint_5, joint_6)

        state.header.stamp = rospy.Time.now()
        pub.publish(state)
        print(time.time()-start)
        rate.sleep()


def speed_control63(v_x, v_y, v_z, j5, j6, joint_4, joint_5, joint_6):
    # initial state of joint_5 is 90 degrees
    jac_matrix = jacob.cal_jacobian_63(joint_4, j5, j6)
    print(joint_4, j5, j6)
    print(f'jacobian: {jac_matrix}')
    ctrl_val = jacob.cal_speed_control(v_x, v_y, v_z, jac_matrix)

    print(f'speed: {ctrl_val}')
    joint_4 += ctrl_val[0][0]*180/3.141593 / 10 # 10hz
    # when theta2 grows, the publishment should decrease, /
    # because z axis points out of the paper
    joint_5 -= ctrl_val[1][0]*180/3.141593 / 10
    joint_6 += ctrl_val[2][0] / 10 / 1000 # mm -> m
    j5 += ctrl_val[1][0]*180/3.141593 / 10
    j6 += ctrl_val[2][0] / 10

    return joint_4, joint_5, joint_6, j5, j6


def speed_control30(v_x, v_y, v_z, joint_1, joint_2, joint_3):
    jac_matrix = jacob.cal_jacobian_O30(joint_1, joint_2)
    print(joint_1, joint_2, joint_3)
    # print(f'jacobian: {jac_matrix}')
    ctrl_val = jacob.cal_speed_control(v_x, v_y, v_z, jac_matrix)

    print(f'speed: {ctrl_val}')
    joint_1 += ctrl_val[0][0]*180/3.141593 / 10
    joint_2 += ctrl_val[1][0]*180/3.141593 / 10
    joint_3 += ctrl_val[2][0] / 10 / 1000 # mm -> m
    return joint_1, joint_2, joint_3



if __name__ == '__main__':
    try:
        joint_1 = -25
        joint_2 = 15
        joint_3 = 0
        joint_4 = 30
        joint_5 = 0
        joint_6 = 0
        joint4_2 = 0
        # initial state of exported model
        j5 = 90 - joint_5
        j6 = 66.000004 - 19.921387 + joint_6 * 1000
        talker(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint4_2, j5, j6)
    except rospy.ROSInterruptException:
        pass
