#!/usr/bin/env python3
# coding: utf-8

import numpy as np



def fw_kine(persai1, persai2, theta3, d3, d5, l2, d6,
            a1=200., a2=86.5, L=406.0, d1=242.8, d2=30.3, l1=70.):
    '''
    forward kinematics
    '''
    persai1 = persai1 * np.pi / 180
    persai2 = persai2 * np.pi / 180

    # theta2 = (0, 180)
    cos = -(l2**2 + d5*d5 - l1*l1 ) /2 /l2 /d5
    sin = np.sqrt(1-cos**2)
    # theta2 = np.arctan( sin/cos )

    theta3 = theta3 * np.pi / 180


    T30 = np.array([[np.sin(persai1 + persai2), np.cos(persai1 + persai2), 0, a1 * np.cos(persai1) + a2 * np.cos(persai1 + persai2)],
                    [-np.cos(persai1 + persai2), np.sin(persai1 + persai2),   0, a1 * np.sin(persai1) + a2 * np.sin(persai1 + persai2)],
                    [0, 0, 1, d1 + d2 + d3],
                    [0, 0, 0, 1]])

    T63 = np.array([[-cos*np.cos(theta3), -np.cos(theta3), -np.sin(theta3)*sin, -d6*np.sin(theta3)*sin],
                    [-sin, 0, -cos, L-d6*cos],
                    [np.cos(theta3)*cos, np.sin(theta3), -np.cos(theta3)*sin, -d6*np.cos(theta3)*sin],
                    [0, 0, 0, 1]])

    T60 = np.dot(T30, T63)

    return T30, T63, T60




fw_data = np.array([
    [0, 0, 0, 0, 78.081811, 37.873248, 0],
    [-68., 20., 55.180247, 47.987605, 76.971144, 25.141174, 40.858823],
    [21.4456, -82.4418, 104.253556, 64.634362, 73.664050, 27.210943, 2.152513],
    [21.4456, 23.5408, -91.458979, 79.860918, 73.023483, 61.883088, 4.116904],
    [55.8793, -31.9768, 48.140870, 37.673202, 83.830159, 49.477857, 16.522138],
    [10.438745, -14.336781, 52.626030, 48.152901, 84.652676, 62.017707, 3.982292],
    [-10.438745, -14.336781, -12.968561, 72.120706, 68.039481, 46.160261, 19.839739],
])

# T60
fw_result = np.array([
    [692.5, 0, 273.1],
    [438.699092, -544.147921, 300.002652],
    [427.011242, -357.246006, 338.237184],
    [538.424515, 420.096537, 353.053352],
    [566.62165, 378.199937, 301.576746],
    [690.538249, 5.165818, 319.287381],
    [647.410728, -248.94932, 326.75419],
])


for i in range(fw_data.shape[0]):
    T30, T63, T60 = fw_kine(fw_data[i][0], fw_data[i][1], fw_data[i][2],
                            fw_data[i][3], fw_data[i][4], fw_data[i][5],
                            fw_data[i][6])
    result1 = T60[0][3]
    result2 = T60[1][3]
    result3 = T60[2][3]

    print(f"{result1 - fw_result[i][0]}, \t{result2 - fw_result[i][1]}, \t{result3 - fw_result[i][2]}")
