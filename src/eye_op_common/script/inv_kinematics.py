#!/usr/bin/env python3
# coding: utf-8

import numpy as np



def inv_kine_36(Px, Py, Pz, L=406, l1=70, L_=66.000004):
    '''
    Given position of RCM point in coordinate system3 and control joint4, 5, 6
    '''
    # theta2 = (0, 180)
    theta2 = np.arctan(-np.sqrt(Px**2 + Pz**2) / (Py - L))
    if (theta2 < 0):
        theta2 += 3.1415926
    # print(f"theta2: {theta2* 180 / 3.1415927}")

    # need not adjust arctan of theta3, theta3 = [-90,90]
    theta3 = np.arctan(Px / Pz)

    sin_theta3 = Px / (np.tan(theta2) * (Py - L_))
    tan_theta3 = Px / Pz
    # consider four conditions to ajust value of theta3
    if (sin_theta3 > 0 and tan_theta3 < 0):
        theta3 += np.pi
    if (sin_theta3 > 0 and tan_theta3 > 0):
        pass
    if (sin_theta3 < 0 and tan_theta3 < 0):
        pass
    if (sin_theta3 < 0 and tan_theta3 > 0):
        theta3 -= np.pi

    print(f"theta3: {theta3* 180 / 3.1415927}")

    d6 = -Px / np.sin(theta2) / np.sin(theta3)
    print(f"d6: {d6}")

    l2 = L_ - d6
    print(f"l2: {l2}")

    # compute d5
    # print(f"d5^2 - {2*l2*np.cos(np.pi-theta2)} * d5 + {l2**2-l1**2} = 0")
    d5_1 = l2 * np.cos(np.pi - theta2) + np.sqrt(l1**2 -
                                                 l2**2 * np.sin(theta2)**2)
    d5_2 = l2 * np.cos(np.pi - theta2) - np.sqrt(l1**2 -
                                                 l2**2 * np.sin(theta2)**2)
    d5 = (d5_1 > 0) * d5_1 + (d5_2 > 0) * d5_2
    print(f"d5: {d5}")

    return theta3, l2, d5


# For verification only
'''
#                     persai1,      persai2,    theta3,       d3,         d5,        d,         d6
inv_data = np.array([[21.4456,    23.5408,    -91.458979,  79.860918,  73.023483, 61.883088, 4.116904],
                     [55.8793,    -31.9768,   48.140870,   37.673202,  83.830159, 49.477857,  16.522138],
                     [10.438745,  -14.336781, 52.626030,   48.152901,  84.652676, 62.017707, 3.982292],
                     [-10.438745, -14.336781, -12.968561,  72.120706,  68.039481, 46.160261,  19.839739]
                  ])

#                                      T06,                                T36,                               T03
inv_result = np.array([[538.424515, 420.096537, 353.053352, 3.629233,   407.941398, 0.092435,   247.332279, 134.273750, 352.960918],
                       [566.62165,  378.199937, 301.576746, -10.264338, 415.113157, -9.196456,  191.269062, 200.619744, 310.773202],
                       [690.538249,   5.165818, 319.287381, -2.573207,  408.318192, -1.965514,  282.989727, 30.3564800, 321.252901],
                       [647.410728, -248.94932, 326.75419,  4.25266,    411.875196, -18.466515, 275.228078, -72.485905, 345.220706],
                    ])
'''


def inv_kine_03(Px, Py, Pz, a1=200., a2=86.5, d1=242.8, d2=30.3, L=406.0):
    '''
    Given position in coordinate system0 and control joint4, 5, 6
    '''
    d3 = Pz - d1 - d2
    print(f"d3: {d3}")

    persai2_1 = np.arccos((Px**2 + Py**2 - a1**2 - a2**2) / (2 * a1 * a2))
    persai2_2 = -persai2_1
    print(f"persai2: {persai2_1* 180 / np.pi, persai2_2* 180 / np.pi}")

    persai1_1 = np.arctan(
        ((a1 + a2 * np.cos(persai2_1)) * Py - a2 * np.sin(persai2_1) * Px) /
        ((a1 + a2 * np.cos(persai2_1)) * Px + a2 * np.sin(persai2_1) * Py))
    persai1_2 = np.arctan(
        ((a1 + a2 * np.cos(persai2_2)) * Py - a2 * np.sin(persai2_2) * Px) /
        ((a1 + a2 * np.cos(persai2_2)) * Px + a2 * np.sin(persai2_2) * Py))
    print(f"persai1: {persai1_1* 180 / np.pi, persai1_2* 180 / np.pi}")

    # TODO: return resonable values
