#!/usr/bin/env python3
# coding: utf-8

import numpy as np


def calculate_jacobian(persai1,
                       persai2,
                       theta3,
                       d3,
                       theta2,
                       d6,
                       a1=200.,
                       a2=86.5,
                       L=406.0):

    persai1 = persai1 / 180 * np.pi
    persai2 = persai2 / 180 * np.pi
    theta3 = theta3 / 180 * np.pi
    theta2 = theta2 / 180 * np.pi

    _jacobian = np.array([[-d6*np.sin(theta3)*np.sin(theta2)*np.cos(persai1+persai2)+(d6*np.cos(theta2)-L-a2)*np.sin(persai1+persai2)-a1*np.sin(persai1), -d6*np.sin(theta3)*np.sin(theta2)*np.cos(persai1+persai2)+(d6*np.cos(theta2)-L-a2)*np.sin(persai1+persai2), 0, -d6*np.cos(theta3)*np.sin(theta2)*np.sin(persai1+persai2), -d6*np.sin(theta3)*np.cos(theta2)*np.cos(persai1+persai2)+d6*np.sin(theta2)*np.cos(persai1+persai2), -np.sin(theta3)*np.sin(theta2)*np.sin(persai1+persai2)-np.cos(theta2)*np.cos(persai1+persai2)],
                         [-d6*np.sin(theta3)*np.sin(theta2)*np.sin(persai1+persai2)+(d6*np.cos(theta2)+L+a2)*np.cos(persai1+persai2)+a1*np.cos(persai1), d6*np.sin(theta3)*np.sin(theta2)*np.sin(persai1+persai2)+(d6*np.cos(theta2)+L+a2)*np.cos(persai1+persai2), 0, d6*np.cos(theta3)*np.sin(theta2)*np.cos(persai1+persai2), d6*np.sin(theta3)*np.cos(theta2)*np.cos(persai1+persai2)+d6*np.sin(theta2)*np.sin(persai1+persai2), np.sin(theta3)*np.sin(theta2)*np.cos(persai1+persai2)-np.cos(theta2)*np.sin(persai1+persai2)],
                         [0,0,1,-d6*np.sin(theta3)*np.sin(theta2),-d6*np.cos(theta3)*np.cos(theta2),-np.cos(theta3)*np.sin(theta2)],
                         [0,0,0,np.cos(persai1+persai2),np.cos(theta3)*np.sin(persai1+persai2),0],
                         [0,0,0,np.sin(persai1+persai2),-np.cos(theta3)*np.cos(persai1+persai2),0],
                         [1,1,0,0,0,0]])

    return _jacobian



def calculate_jacob_result(persai1_dot, persai2_dot, theta3_dot, d3_dot, theta2_dot, d6_dot, jacobian_matrix):
    '''
    Input parameters: velocity of each joint
    jacobian_matrix: jacobian matrix of current status
    '''
    _velocity_matrix = np.array([[persai1_dot],[persai2_dot],[theta3_dot],[d3_dot],[theta2_dot],[d6_dot]])

    return(np.dot(jacobian_matrix, _velocity_matrix))
