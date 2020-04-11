#!/usr/bin/env python3
# coding: utf-8

import numpy as np

# calculate jacobian matrix of robot arm
def calculate_jacobian_60(persai1,
                       persai2,
                       theta3,
                       d3,
                       theta2,
                       d6,
                       a1=200.,
                       a2=86.5,
                       L=406.0):

    '''
    input: values of six parameters
    output: jacobian matrix of current state
    '''
    persai1 = persai1 / 180 * np.pi
    persai2 = persai2 / 180 * np.pi
    theta3 = theta3 / 180 * np.pi
    theta2 = theta2 / 180 * np.pi

    _jacobian_60 = np.array([[-d6*np.sin(theta3)*np.sin(theta2)*np.cos(persai1+persai2)+(d6*np.cos(theta2)-L-a2)*np.sin(persai1+persai2)-a1*np.sin(persai1), -d6*np.sin(theta3)*np.sin(theta2)*np.cos(persai1+persai2)+(d6*np.cos(theta2)-L-a2)*np.sin(persai1+persai2), 0, -d6*np.cos(theta3)*np.sin(theta2)*np.sin(persai1+persai2), -d6*np.sin(theta3)*np.cos(theta2)*np.cos(persai1+persai2)+d6*np.sin(theta2)*np.cos(persai1+persai2), -np.sin(theta3)*np.sin(theta2)*np.sin(persai1+persai2)-np.cos(theta2)*np.cos(persai1+persai2)],
                         [-d6*np.sin(theta3)*np.sin(theta2)*np.sin(persai1+persai2)+(d6*np.cos(theta2)+L+a2)*np.cos(persai1+persai2)+a1*np.cos(persai1), d6*np.sin(theta3)*np.sin(theta2)*np.sin(persai1+persai2)+(d6*np.cos(theta2)+L+a2)*np.cos(persai1+persai2), 0, d6*np.cos(theta3)*np.sin(theta2)*np.cos(persai1+persai2), d6*np.sin(theta3)*np.cos(theta2)*np.cos(persai1+persai2)+d6*np.sin(theta2)*np.sin(persai1+persai2), np.sin(theta3)*np.sin(theta2)*np.cos(persai1+persai2)-np.cos(theta2)*np.sin(persai1+persai2)],
                         [0,0,1,-d6*np.sin(theta3)*np.sin(theta2),-d6*np.cos(theta3)*np.cos(theta2),-np.cos(theta3)*np.sin(theta2)],
                         [0,0,0,np.cos(persai1+persai2),np.cos(theta3)*np.sin(persai1+persai2),0],
                         [0,0,0,np.sin(persai1+persai2),-np.cos(theta3)*np.cos(persai1+persai2),0],
                         [1,1,0,0,0,0]])

    return _jacobian_60


def calculate_jacobian_30(persai1,
                       persai2,
                       a1=200.,
                       a2=86.5,
                       L=406.0):
    '''
    input standard: degree(s)
    '''

    persai1 = persai1 * np.pi / 180
    persai2 = persai2 * np.pi / 180

    _jacobian_30 = np.array([[-a1*np.sin(persai1)-(a2+L)*np.sin(persai1+persai2), -(a2+L)*np.sin(persai1+persai2), 0],
                            [a1*np.cos(persai1)+(a2+L)*np.cos(persai1+persai2), (a2+L)*np.cos(persai1+persai2), 0],
                            [0, 0, 1]])

    return _jacobian_30


def calculate_jacobian_63(theta3,
                       theta2,
                       d6,
                       a1=200.,
                       a2=86.5,
                       L=406.0):
    '''
    input standard: degree(s)
    '''

    theta3 = theta3 * np.pi / 180
    theta2 = theta2 * np.pi / 180

    _jacobian_63 = np.array([[-np.sin(theta3)*np.sin(theta2), -d6*np.sin(theta3)*np.cos(theta2), -d6*np.cos(theta3)*np.sin(theta2)],
                            [-np.cos(theta2),                 d6*np.sin(theta2),                  0],
                            [-np.cos(theta3)*np.sin(theta2), -d6*np.cos(theta3)*np.cos(theta2), d6*np.sin(theta3)*np.sin(theta2)]])

    return _jacobian_63


def calculate_jacob_result(persai1_dot, persai2_dot, theta3_dot, d3_dot, theta2_dot, d6_dot, jacobian_matrix):
    '''
    Input parameters: velocity of each joint
    jacobian_matrix: jacobian matrix of current status
    return: velocity of end effector
    '''
    _velocity_matrix = np.array([[persai1_dot],[persai2_dot],[theta3_dot],[d3_dot],[theta2_dot],[d6_dot]])

    return(np.dot(jacobian_matrix, _velocity_matrix))


def calculate_speed_control(x_dot, y_dot, z_dot, jacobian_matrix):
    '''
    Input parameters: velocity of omni phantom
    jacobian_matrix: jacobian matrix of current status
    return: velocity of corresponding joints
    '''
    _velocity_matrix = np.array([[x_dot],[y_dot],[z_dot]])
    _inv_jacobian = np.linalg.inv(jacobian_matrix)

    return(np.dot(_inv_jacobian, _velocity_matrix))


if __name__ == '__main__':
    calculate_jacobian_63(0, 0, 0)
