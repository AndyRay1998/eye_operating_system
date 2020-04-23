#!/usr/bin/env python3
import sys
sys.path.append(sys.path[0] + '/../src')
import gclib_python.gclib as gclib
import eye_op_jacobian as jacob

# TODO: test

def g_init(ip_address='192.168.0.42'):
    '''
    Connection of galil card through ip address
    '''
    # make an instance of the gclib python class
    g = gclib.py()

    # try device network connection
    try:
        g.GOpen(ip_address + ' --direct -s ALL')
        #g.GOpen('COM1 --direct')
        print(g.GInfo())
        return g
    except:
        print("\033[1;31;45m[WARN] Galil connection failed\033[0m")
        print("\033[1;31;45m[WARN] Stopping Galil script\033[0m")
        sys.exit()


def g_jog(v1, v2, v3, g):
    '''
    function: complete JOG mode motion; used for velocity control
    params: v -> velocities of theta3, theta2, d6
            g_flag -> flag that avoid repetitive definition of AC, DC
    unit: mm/s
    '''
    # TODO: convert position values to degrees
    # TP returns value of encoder in string form
    p1 = float(g.GCommand('TP A')) # theta3
    p2 = float(g.GCommand('TP B')) # theta2
    p3 = float(g.GCommand('TP C')) #

    # TODO: also in robotics package -> simulation is not successful; must be an error inside
    # convert velocity from cartesian space to joint space --- we currently assume velocities from omni is in cartesian space
    jac_matrix = jacob.cal_jacobian_63(p1, p2, p3)
    ctrl_val = jacob.cal_speed_control(v1, v2, v3, jac_matrix)
    v1 = ctrl_val[0][0]
    v2 = ctrl_val[0][1]
    v3 = ctrl_val[0][2]

    # TODO: confirm all values in the following command
    # motion command sent to galil card
    g.GCommand('AC 20000,20000,20000') # acceleration 20000 cts/s^2
    g.GCommand('DC 20000,20000,20000') # deceleration 20000 cts/s^2

    g.GCommand('JG v1,v2,v3') # JOG motion mode; set velocity
    # print(' Starting move...')
    g.GCommand('BG ABC') #begin motion
    g.GMotionComplete('ABC')
    # print(' done.')
