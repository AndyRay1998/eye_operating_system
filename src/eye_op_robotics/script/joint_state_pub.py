#!/usr/bin/env python3
'''
This script is for jacobian verification through rviz simulation.
Revice v_x, v_y, v_z and roslaunch simulation.launch file.
No software position limit is inplemented, so when approaching sigularity, /
simulation becomes strange; we do not have that affliction in real life though.
'''

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

import eye_op_jacobian as jacob


def talker(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, j5, j6, v_x=0, v_y=5, v_z=0):
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
    while (time.time()-start < 10):
        state = JointState()
        state.header = Header()
        state.header.stamp = rospy.Time.now()
        state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        state.position = [0, 0, 0, 0, 0, 0]
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
        state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        state.position = [joint_1*3.141593/180, joint_2*3.141593/180, joint_3, joint_4*3.141593/180, joint_5*3.141593/180, joint_6]
        state.velocity = []
        state.effort = []

        # map the velocity in system0 to that in system3 and input
        joint_4, joint_5, joint_6, j5, j6 = speed_control63(-v_y, v_x, v_z, j5, j6, joint_4, joint_5, joint_6)
        # joint_1, joint_2, joint_3 = speed_control30(v_x, v_y, v_z, joint_1, joint_2, joint_3)

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
    print(f'jacobian: {jac_matrix}')
    ctrl_val = jacob.cal_speed_control(v_x, v_y, v_z, jac_matrix)

    print(f'speed: {ctrl_val}')
    joint_1 += ctrl_val[0][0]*180/3.141593 / 10
    joint_2 += ctrl_val[1][0]*180/3.141593 / 10
    joint_3 += ctrl_val[2][0] / 10 / 1000 # mm -> m
    return joint_1, joint_2, joint_3



if __name__ == '__main__':
    try:
        joint_1 = 5
        joint_2 = 5
        joint_3 = 0
        joint_4 = 0
        joint_5 = 0
        joint_6 = 0
        # initial state of exported model
        j5 = 90
        j6 = 66.000004 - 19.921387
        talker(joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, j5, j6)
    except rospy.ROSInterruptException:
        pass
