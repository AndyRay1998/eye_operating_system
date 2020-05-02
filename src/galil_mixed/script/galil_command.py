#!/usr/bin/env python3

'''
For better encapsulation and param sharing, I implement OOP-style.
'''
import subprocess
import sys
sys.path.append(subprocess.getoutput("rospack find galil_mixed") + '/src')
import gclib_python.gclib as gclib
import eye_op_jacobian as jacob


class g_control():
    def __init__(self, ip_address='192.168.0.42'):
        self.ip_address = ip_address
        # make an instance of the gclib python class
        self.g = gclib.py()


    def connect(self):
        '''
        Connection of galil card through ip address
        '''
        # try device network connection
        try:
            self.g.GOpen(ip_address + ' --direct -s ALL')
            # self.g.GOpen('COM1 --direct')
            print(self.g.GInfo())
        except:
            print("\033[1;31;45m[WARN] Galil connection failed\033[0m")
            print("\033[1;31;45m[WARN] Stopping Galil script\033[0m")
            sys.exit()


    def g_jog(self, v1, v2, v3):
        '''
        function: complete JOG mode motion; used for velocity control
        params: v -> velocities of theta3, theta2, d6
                g_flag -> flag that avoid repetitive definition of AC, DC
        unit: mm/s
        '''
        # TODO: convert position values to degrees; confirm axis number of last three joints of eye op robot
        # TP returns value of position encoder in string form with unit of 'cts'
        p1 = float(self.g.GCommand('TP A')) # theta3
        p2 = float(self.g.GCommand('TP B')) # theta2
        p3 = float(self.g.GCommand('TP C')) # d6
        # P.convert_to_degrees()

        # TODO: omni_vel.convert_to_CoordinateSystem6
        jac_matrix = jacob.cal_jacobian_63(p1, p2, p3)
        # omni_vel.convert_to_CoordinateSystem6
        ctrl_val = jacob.cal_speed_control(v1, v2, v3, jac_matrix)
        v1 = ctrl_val[0][0]
        v2 = ctrl_val[0][1]
        v3 = ctrl_val[0][2]

        # TODO: confirm all values in the following command
        # motion command sent to galil card
        self.g.GCommand('AC 20000,20000,20000') # acceleration 20000 cts/s^2
        self.g.GCommand('DC 20000,20000,20000') # deceleration 20000 cts/s^2

        self.g.GCommand('JG v1,v2,v3') # JOG motion mode; set velocity
        # print(' Starting move...')
        self.g.GCommand('BG ABC') #begin motion
        self.g.GMotionComplete('ABC')
        # print(' done.')


    def diconnect(self):
        '''
        shut down galil instance
        '''
        self.g.Gclose()


    def motor_off(self, axis):
        '''
        Shut down control system, and keep position-monitor on.
        This is used for manual adjustment of robot, after which use 'SH' to activate servo.
        Input: A to F
        '''
        self.g.GCommand('MO' + axis)


    def servo_on(self, axis):
        '''
        Activate servo state.
        This alters coordinate system. Any commends of position must be resent!!!
        Input: A to F
        '''
        self.g.GCommand('SH' + axis)


    def ask_servo(self, axis):
        '''
        Request servo state
        Input: A to F
        return: off -> 0; on -> 1
        '''
        self.g.GCommand('_MO' + axis)

    def iap(self, axis, distance=5):
        '''
        Independent axis positioning.
        Input: axis -> A, B, C; distance unit: mm or degree
        '''
        # TODO: convert distance to cts unit
        # distance = convert_to_cts()
        self.g.GCommand('PR' + axis + "=" + str(distance))
