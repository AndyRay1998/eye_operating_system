#!/usr/bin/env python3
# coding: utf-8

import numpy as np

''' parameters note'''
# Six params of eye_op_robot: persai1, persai2, theta3, d3, theta2, d6
# Among them, theta2 and d6 are slave params, whose master params are l2 and d5

''' unit note'''
# mm, degree, mm/s, degree/s


def cal_jacobian_60(persai1,
                       persai2,
                       theta3,
                       d3,
                       theta2,
                       d6,
                       a1=200.,
                       a2=86.5,
                       L=406.0) -> np.array: # -> means suggested return type

    '''
    This function calculate jacobian matrix of Coordinate System 6 to 0
    input: values of six parameters
    output: jacobian matrix of current state
    unit: mm, degree
    '''
    persai1 = persai1 / 180 * np.pi
    persai2 = persai2 / 180 * np.pi
    theta3 = theta3 / 180 * np.pi
    theta2 = theta2 / 180 * np.pi

    s3 = np.sin(theta3)
    s2 = np.sin(theta2)
    c2 = np.cos(theta2)
    c3 = np.cos(theta3)

    jacobian_60 = np.array([[-d6*s3*s2*np.cos(persai1+persai2)+(d6*c2-L-a2)*np.sin(persai1+persai2)-a1*np.sin(persai1), -d6*s3*s2*np.cos(persai1+persai2)+(d6*c2-L-a2)*np.sin(persai1+persai2), 0, -d6*c3*s2*np.sin(persai1+persai2), -d6*s3*c2*np.cos(persai1+persai2)+d6*s2*np.cos(persai1+persai2), -s3*s2*np.sin(persai1+persai2)-c2*np.cos(persai1+persai2)],
                         [-d6*s3*s2*np.sin(persai1+persai2)+(d6*c2+L+a2)*np.cos(persai1+persai2)+a1*np.cos(persai1), d6*s3*s2*np.sin(persai1+persai2)+(d6*c2+L+a2)*np.cos(persai1+persai2), 0, d6*c3*s2*np.cos(persai1+persai2), d6*s3*c2*np.cos(persai1+persai2)+d6*s2*np.sin(persai1+persai2), s3*s2*np.cos(persai1+persai2)-c2*np.sin(persai1+persai2)],
                         [0,0,1,-d6*s3*s2,-d6*c3*c2,-c3*s2],
                         [0,0,0,np.cos(persai1+persai2),c3*np.sin(persai1+persai2),0],
                         [0,0,0,np.sin(persai1+persai2),-c3*np.cos(persai1+persai2),0],
                         [1,1,0,0,0,0]])

    return jacobian_60


def cal_jacobian_O30(persai1,
                       persai2,
                       a1=200.,
                       a2=86.5,
                       L=406.0) -> np.array:
    '''
    Jacob matrix of RCM points xyz values in Coordinate System 3 relative to 0
    input: first two joint values
    unit: degree, mm
    '''

    persai1 = persai1 * np.pi / 180
    persai2 = persai2 * np.pi / 180

    jacobian_O30 = np.array([[-a1*np.sin(persai1)-(a2+L)*np.sin(persai1+persai2), -(a2+L)*np.sin(persai1+persai2), 0],
                            [a1*np.cos(persai1)+(a2+L)*np.cos(persai1+persai2), (a2+L)*np.cos(persai1+persai2), 0],
                            [0, 0, 1]])

    return jacobian_O30


def cal_jacobian_63(theta3, theta2, d6) -> np.array:
    '''
    This function calculate jacobian matrix of Coordinate System 6 to 3
    unit: degree, mm
    '''
    theta3 = theta3 * np.pi / 180
    theta2 = theta2 * np.pi / 180

    s3 = np.sin(theta3)
    s2 = np.sin(theta2)
    c2 = np.cos(theta2)
    c3 = np.cos(theta3)

    jacobian_63 = np.array([[-d6*c3*s2, -d6*s3*c2, -s3*s2],
                            [0,         d6*s2,     -c2],
                            [d6*s3*s2,  -d6*c3*c2, -c3*s2]])

    return jacobian_63


def cal_r_63(theta3, theta2, d6) -> np.array:
    '''
    This function calculates rotation matrix of Coordinate System 6 to 3.
    This is for velocity transform from end effector to system3.
    unit: degree, mm
    '''
    theta3 = theta3 * np.pi / 180
    theta2 = theta2 * np.pi / 180

    s3 = np.sin(theta3)
    s2 = np.sin(theta2)
    c2 = np.cos(theta2)
    c3 = np.cos(theta3)

    r_63 = np.array([[c2*s3, -c3, -s3*s2],
                     [-s2,    0,   -c2],
                     [c3*c2, s3, -c3*s2]])

    return r_63


def cal_end_vel(persai1_dot,
                persai2_dot,
                theta3_dot,
                d3_dot,
                theta2_dot,
                d6_dot,
                jacobian_matrix) -> np.array:
    '''
    Map joint velocity to that of the end effector
    Input parameters: velocity of each joint
    jacobian_matrix: jacobian matrix of current status
    return: velocity of end effector
    unit: mm/s, degree/s
    '''
    velocity_matrix = np.array([
        [persai1_dot],
        [persai2_dot],
        [theta3_dot],
        [d3_dot],
        [theta2_dot],
        [d6_dot]
    ])

    return(np.dot(jacobian_matrix, velocity_matrix))


def cal_speed_control(x_dot, y_dot, z_dot, jacobian_matrix) -> np.array:
    '''
    Input parameters: velocity of omni phantom
    jacobian_matrix: jacobian matrix of current status
    return: velocity of corresponding joints
    unit: mm/s
    '''
    velocity_matrix = np.array([[x_dot],[y_dot],[z_dot]])
    inv_jacobian = np.linalg.inv(jacobian_matrix)
    print(inv_jacobian)

    return(np.dot(inv_jacobian, velocity_matrix))
