#!/usr/bin/env python3
# coding: utf-8

import sys
import os
import threading
import time
import subprocess

import libtmux # reference: https://github.com/tmux-python/libtmux/

from PyQt5.QtWidgets import (QPushButton, QApplication, QWidget, QMessageBox, QTabWidget, QGridLayout,
                             QHBoxLayout, QVBoxLayout, QFormLayout, QComboBox)
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QCoreApplication, QEvent, Qt

from PyQt5.QtWidgets import *


#### class for tab1 ####
class tab1UI(QTabWidget):
    def __init__(self):
        # init of parent class
        super().__init__()

        # tmux related
        # init a tmux instance
        self.tmux_server = libtmux.Server()
        # avoid namesake conflict
        try:
            self.main_sess = self.tmux_server.new_session("main_sess")
        except:
            os.system('tmux kill-session -t main_sess')
            self.main_sess = self.tmux_server.new_session("main_sess")
        # create a window
        self.main_win_1 = self.main_sess.new_window(attach=False, window_name="main_win_1")
        # split two panes
        self.main_pane_1 = self.main_win_1.split_window(attach=False)
        # choose the first pane
        self.main_pane_1 = self.main_win_1.panes[0]

        # button
        self.sButton = QPushButton("Start all nodes")
        self.sButton.setToolTip("Start all nodes")
        self.eButton = QPushButton("End")
        self.qButton = QPushButton("quit")
        self.eButton.setEnabled(False)

        # device name labels
        self.gLabel = QLabel('galil')
        self.hLabel = QLabel('hyperion')
        self.oLabel = QLabel('omni')
        self.yLabel = QLabel('yamaha')
        # state label: ON / OFF
        self.gsLabel = QLabel('OFF')
        self.hsLabel = QLabel('OFF')
        self.osLabel = QLabel('OFF')
        self.ysLabel = QLabel('OFF')

        # grid layout
        self.grid = QGridLayout()

        # layout
        self.hbox = QHBoxLayout()
        self.vbox = QVBoxLayout()

        self.sButton = QPushButton("Start")
        self.eButton = QPushButton("End")
        self.eButton.setEnabled(False)

        # exit flag, for while loop break
        self.exit_flag = 0

        # UI plotting
        self.initUI()


    def initUI(self):
        ## button event
        self.sButton.clicked.connect(self.start_click)
        self.eButton.clicked.connect(self.end_click)
        self.qButton.clicked.connect(self.closeEvent)

        ## vbox and hbox layout
        self.hbox.addStretch(1)
        self.hbox.addWidget(self.sButton)
        self.hbox.addWidget(self.eButton)
        self.hbox.addWidget(self.qButton)
        self.vbox.addStretch(1)
        self.vbox.addLayout(self.hbox)

        ## grid layout
        self.grid.setSpacing(10)
        # add labels
        self.grid.addWidget(self.gLabel, 2, 0)
        self.grid.addWidget(self.hLabel, 3, 0)
        self.grid.addWidget(self.oLabel, 4, 0)
        self.grid.addWidget(self.yLabel, 5, 0)
        self.grid.addWidget(self.gsLabel, 2, 1)
        self.grid.addWidget(self.hsLabel, 3, 1)
        self.grid.addWidget(self.osLabel, 4, 1)
        self.grid.addWidget(self.ysLabel, 5, 1)

        self.grid.addLayout(self.vbox, 7, 3)
        # holistically setup
        self.setLayout(self.grid)


    def state_monitor_thread(self):
        '''
        While starting operating, begin state monitor in another thread.
        This will update state labels of all devices.
        '''
        time.sleep(1) # wait for eButton state to be changed
        while True:
            # see active ROS nodes
            nodes = subprocess.getoutput("rosnode list")
            if(self.eButton.isEnabled()==True):
                # update nodes' state
                if('Galil' in nodes): self.gsLabel.setText('ON')
                else: self.gsLabel.setText('OFF')
                if('Hyperion' in nodes): self.hsLabel.setText('ON')
                else: self.hsLabel.setText('OFF')
                if('Omni' in nodes): self.osLabel.setText('ON')
                else: self.osLabel.setText('OFF')
                if('Yamaha' in nodes): self.ysLabel.setText('ON')
                else: self.ysLabel.setText('OFF')
            else:
                # stop thread when all nodes are off
                if('Galil' in nodes or 'Hyperion' in nodes or 'Omni' in nodes or 'Yamaha' in nodes):
                    pass
                else:
                    self.gsLabel.setText('OFF')
                    self.hsLabel.setText('OFF')
                    self.osLabel.setText('OFF')
                    self.ysLabel.setText('OFF')
                    break


    def start_click_thread(self):
        self.sButton.setEnabled(False)
        self.eButton.setEnabled(True)
        self.main_pane_1.send_keys('roslaunch eye_op_common eye_op_robot.launch')
        self.main_pane_1 = self.main_win_1.panes[1]
        self.main_pane_1.send_keys('rqt_graph')


    def start_click(self):
        thread_1 = threading.Thread(target=self.state_monitor_thread)
        thread_2 = threading.Thread(target=self.start_click_thread)
        # start thread; do not change starting sequence!!
        thread_2.start()
        thread_1.start()


    def end_click(self):
        '''
        When trying to exit, reinit tmux windows and change button state.
        '''
        try:
            self.main_sess.kill_window(1)
        except:
            pass
        # re-init a window
        self.main_win_1 = self.main_sess.new_window(attach=False, window_name="main_win_1")
        self.main_pane_1 = self.main_win_1.split_window(attach=False)
        # choose the first pane
        self.main_pane_1 = self.main_win_1.panes[0]
        # afterwards, change button state
        self.sButton.setEnabled(True)
        self.eButton.setEnabled(False)


    def closeEvent(self, event):
        '''
        When user tries to close the window, pop out confirmation message.
        '''
        # cannot quit while devices are under operation
        if self.eButton.isEnabled():
            QMessageBox.warning(self, 'Warning',
                "Devices are operating. Please end first!")
        else:
            reply = QMessageBox.question(self, 'Message',
                "Are you sure to quit?", QMessageBox.Yes |
                QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                # quit
                self.exit_flag = 1
                time.sleep(1)
                QCoreApplication.instance().quit()
            else:
                event.ignore()


#### class for tab2 ####
class tab2UI(QTabWidget):
    def __init__(self):
        super().__init__()
        # use 'src' as project source path
        self.src_path = subprocess.getoutput("rospack find eye_op_common") + "/.."
        # create yamaha instance for sending command
        sys.path.append(self.src_path + "/yamaha_mixed/script")
        from yamaha_serial import command
        # TODO: confirm port number of YAMAHA
        # here we assign port to "" just for null initialization; do not change it here!!!
        self.yamaha = command(port="")
        self.yamaha_port = "TODO"

        # create galil instance for sending command
        sys.path.append(self.src_path + "/galil_mixed/script")
        from galil_command import g_control
        # TODO: confirm ip of GALIL
        # here we assign port to "" just for null initialization; do not change it here!!!
        self.galil = g_control(ip_address="")
        self.galil_ip = "TODO"
        '''
        # common button
        self.sButton = QPushButton("Start")
        self.eButton = QPushButton("End")
        self.eButton.setEnabled(False)
        # click event
        self.sButton.clicked.connect(self.start_click)
        self.eButton.clicked.connect(self.end_click)
        '''
        # device connect and disconnect
        self.gButton = QPushButton("connect galil") # galil connect
        self.hButton = QPushButton("connect hyperion") # hyperion connect
        self.oButton = QPushButton("connect omni") # omni connect
        self.yButton = QPushButton("connect yamaha") # yamaha connect
        # click event
        self.gButton.clicked.connect(lambda:self.connect_click("g"))
        self.hButton.clicked.connect(lambda:self.connect_click("h"))
        self.oButton.clicked.connect(lambda:self.connect_click("o"))
        self.yButton.clicked.connect(lambda:self.connect_click("y"))

        self.hbox = QHBoxLayout()
        self.vbox = QVBoxLayout()

        # device name labels
        self.gLabel = QLabel('galil')
        self.hLabel = QLabel('hyperion')
        self.oLabel = QLabel('omni')
        self.yLabel = QLabel('yamaha')
        # state label: ON / OFF
        self.gsLabel = QLabel('OFF')
        self.hsLabel = QLabel('OFF')
        self.osLabel = QLabel('OFF')
        self.ysLabel = QLabel('OFF')
        # motor name labels(for jog motion)
        self.m1Label = QLabel('motor1')
        self.m2Label = QLabel('motor2')
        self.m3Label = QLabel('motor3')
        self.m4Label = QLabel('motor4')
        self.m5Label = QLabel('motor5')
        self.m6Label = QLabel('motor6')
        # unit labels
        self.unit1Label = QLabel('degree(s)')
        self.unit2Label = QLabel('degree(s)')
        self.unit3Label = QLabel('mm')
        self.unit4Label = QLabel('degree(s)')
        self.unit5Label = QLabel('degree(s)')
        self.unit6Label = QLabel('mm')

        # buttons for jog motion
        # TODO: determine servo number of each motor in Robot
        # RCM is controlled by GALIL while SCARA by YAMAHA controller
        # here we assume 1,2,3 are SCARA and 4,5,6 are RCM
        self.ser1Button = QPushButton("Servo OFF")
        self.ser1Button.setCheckable(True)
        self.ser2Button = QPushButton("Servo OFF")
        self.ser2Button.setCheckable(True)
        self.ser3Button = QPushButton("Servo OFF")
        self.ser3Button.setCheckable(True)
        self.ser4Button = QPushButton("Servo OFF")
        self.ser4Button.setCheckable(True)
        self.ser5Button = QPushButton("Servo OFF")
        self.ser5Button.setCheckable(True)
        self.ser6Button = QPushButton("Servo OFF")
        self.ser6Button.setCheckable(True)
        # button event
        self.ser1Button.clicked.connect(lambda:self.jog_click("self.ser1Button"))
        self.ser2Button.clicked.connect(lambda:self.jog_click("self.ser2Button"))
        self.ser3Button.clicked.connect(lambda:self.jog_click("self.ser3Button"))
        self.ser4Button.clicked.connect(lambda:self.jog_click("self.ser4Button"))
        self.ser5Button.clicked.connect(lambda:self.jog_click("self.ser5Button"))
        self.ser6Button.clicked.connect(lambda:self.jog_click("self.ser6Button"))
        # jo1p -> jog 1 plus; jo1m -> jog 1 minus
        self.jo1pButton = QPushButton("JOG +")
        self.jo1mButton = QPushButton("JOG -")
        self.jo2pButton = QPushButton("JOG +")
        self.jo2mButton = QPushButton("JOG -")
        self.jo3pButton = QPushButton("JOG +")
        self.jo3mButton = QPushButton("JOG -")
        self.jo4pButton = QPushButton("JOG +")
        self.jo4mButton = QPushButton("JOG -")
        self.jo5pButton = QPushButton("JOG +")
        self.jo5mButton = QPushButton("JOG -")
        self.jo6pButton = QPushButton("JOG +")
        self.jo6mButton = QPushButton("JOG -")
        # button event
        self.jo1pButton.clicked.connect(lambda:self.jog_click("self.jo1pButton+"))
        self.jo1mButton.clicked.connect(lambda:self.jog_click("self.jo1mButton-"))
        self.jo2pButton.clicked.connect(lambda:self.jog_click("self.jo2pButton+"))
        self.jo2mButton.clicked.connect(lambda:self.jog_click("self.jo2mButton-"))
        self.jo3pButton.clicked.connect(lambda:self.jog_click("self.jo3pButton+"))
        self.jo3mButton.clicked.connect(lambda:self.jog_click("self.jo3mButton-"))
        self.jo4pButton.clicked.connect(lambda:self.jog_click("self.jo4pButton+"))
        self.jo4mButton.clicked.connect(lambda:self.jog_click("self.jo4mButton-"))
        self.jo5pButton.clicked.connect(lambda:self.jog_click("self.jo5pButton+"))
        self.jo5mButton.clicked.connect(lambda:self.jog_click("self.jo5mButton-"))
        self.jo6pButton.clicked.connect(lambda:self.jog_click("self.jo6pButton+"))
        self.jo6mButton.clicked.connect(lambda:self.jog_click("self.jo6mButton-"))

        # line edit: input inching distance
        self.d1Edit = QLineEdit("5")
        self.d2Edit = QLineEdit("5")
        self.d3Edit = QLineEdit("5")
        self.d4Edit = QLineEdit("5")
        self.d5Edit = QLineEdit("5")
        self.d6Edit = QLineEdit("5")
        self.d1Edit.setFixedSize(100, 23)
        self.d2Edit.setFixedSize(100, 23)
        self.d3Edit.setFixedSize(100, 23)
        self.d4Edit.setFixedSize(100, 23)
        self.d5Edit.setFixedSize(100, 23)
        self.d6Edit.setFixedSize(100, 23)

        # grid layout
        self.grid = QGridLayout()

        self.initUI()


    def initUI(self):
        '''
        # vbox and hbox layout
        self.hbox.addStretch(1)
        self.hbox.addWidget(self.sButton)
        self.hbox.addWidget(self.eButton)
        self.vbox.addStretch(1)
        self.vbox.addLayout(self.hbox)
        '''

        ## grid layout
        self.grid.setSpacing(10)
        # add labels
        '''
        self.grid.addWidget(self.gLabel, 2, 0)
        self.grid.addWidget(self.hLabel, 3, 0)
        self.grid.addWidget(self.oLabel, 4, 0)
        self.grid.addWidget(self.yLabel, 5, 0)
        self.grid.addWidget(self.gsLabel, 2, 1)
        self.grid.addWidget(self.hsLabel, 3, 1)
        self.grid.addWidget(self.osLabel, 4, 1)
        self.grid.addWidget(self.ysLabel, 5, 1)
        '''
        self.grid.addWidget(self.m1Label, 6, 0)
        self.grid.addWidget(self.m2Label, 7, 0)
        self.grid.addWidget(self.m3Label, 8, 0)
        self.grid.addWidget(self.m4Label, 9, 0)
        self.grid.addWidget(self.m5Label, 10, 0)
        self.grid.addWidget(self.m6Label, 11, 0)
        self.grid.addWidget(self.unit1Label, 6, 5)
        self.grid.addWidget(self.unit2Label, 7, 5)
        self.grid.addWidget(self.unit3Label, 8, 5)
        self.grid.addWidget(self.unit4Label, 9, 5)
        self.grid.addWidget(self.unit5Label, 10, 5)
        self.grid.addWidget(self.unit6Label, 11, 5)

        # add buttons
        '''
        self.grid.addWidget(self.gButton, 2, 2)
        self.grid.addWidget(self.hButton, 3, 2)
        self.grid.addWidget(self.oButton, 4, 2)
        self.grid.addWidget(self.yButton, 5, 2)
        '''
        self.grid.addWidget(self.ser1Button, 6, 1)
        self.grid.addWidget(self.ser2Button, 7, 1)
        self.grid.addWidget(self.ser3Button, 8, 1)
        self.grid.addWidget(self.ser4Button, 9, 1)
        self.grid.addWidget(self.ser5Button, 10, 1)
        self.grid.addWidget(self.ser6Button, 11, 1)
        self.grid.addWidget(self.jo1pButton, 6, 2)
        self.grid.addWidget(self.jo1mButton, 6, 3)
        self.grid.addWidget(self.jo2pButton, 7, 2)
        self.grid.addWidget(self.jo2mButton, 7, 3)
        self.grid.addWidget(self.jo3pButton, 8, 2)
        self.grid.addWidget(self.jo3mButton, 8, 3)
        self.grid.addWidget(self.jo4pButton, 9, 2)
        self.grid.addWidget(self.jo4mButton, 9, 3)
        self.grid.addWidget(self.jo5pButton, 10, 2)
        self.grid.addWidget(self.jo5mButton, 10, 3)
        self.grid.addWidget(self.jo6pButton, 11, 2)
        self.grid.addWidget(self.jo6mButton, 11, 3)

        # add line edit
        self.grid.addWidget(self.d1Edit, 6, 4)
        self.grid.addWidget(self.d2Edit, 7, 4)
        self.grid.addWidget(self.d3Edit, 8, 4)
        self.grid.addWidget(self.d4Edit, 9, 4)
        self.grid.addWidget(self.d5Edit, 10, 4)
        self.grid.addWidget(self.d6Edit, 11, 4)

        # self.grid.addLayout(self.vbox, 12, 4)
        # holistically setup
        self.setLayout(self.grid)

    '''
    def start_click(self):
        self.sButton.setEnabled(False)
        self.eButton.setEnabled(True)

    def end_click(self):

        self.sButton.setEnabled(True)
        self.eButton.setEnabled(False)

    def connect_click(self, flag):
        print(flag)
    '''

    def jog_click(self, flag):
        # TODO: uncomment and test
        '''
        # connect to devices if not connected yet
        if self.yamaha.port == "":
            self.yamaha.port = self.yamaha_port
            self.yamaha.connect()
        if self.galil.ip_address == "":
            self.galil.ip_address = self.galil_ip
            self.galil.connect()
        '''
        # map axis number to alphabet for galil control
        # assuming using A,B,C axis in galil
        # TODO: confirm axis used in galil device
        dic = {1:'A', 2:'B', 3:'C'}
        # servo control
        if 'ser' in flag:
            # TODO: uncomment and test
            '''
            # monitor and change servo state indicator
            thread_servo = threading.Thread(target=self.servo_monitoring)
            thread_servo.start()
            '''
            # map string to param names
            param = eval(flag)
            if param.isChecked():
                param.setText("SERVO ON")
                # TODO: determine what kind of status to be used
                servo_status = "ON"
            else:
                param.setText("SERVO OFF")
                servo_status = "OFF"
            ## YAMAHA servo control
            if '1' in flag or '2' in flag or '3' in flag:
                # TODO: uncomment and test
                # self.yamaha.servo_set(servo_status, flag[-7])
                pass
            ## galil servo control
            else:
                # TODO: uncomment and Test
                '''
                # turn on servo
                if servo_status == 'ON':
                    self.galil.servo_on(dic[flag[-7]-3])
                # turn off servo
                else:
                    self.galil.motor_off(dic[int(flag[-7])-3])
                '''
                pass
        # PTP motion
        else:
            # TODO: uncomment and test
            '''
            # YAMAHA PTP motion
            if '1' in flag and '+' in flag:
                self.movei([5,0,0,0,0,0])
            elif '1' in flag and '-' in flag:
                self.movei([-5,0,0,0,0,0])
            if '2' in flag and '+' in flag:
                self.movei([0,5,0,0,0,0])
            elif '2' in flag and '-' in flag:
                self.movei([0,-5,0,0,0,0])
            if '3' in flag and '+' in flag:
                self.movei([0,0,5,0,0,0])
            elif '3' in flag and '-' in flag:
                self.movei([0,0,-5,0,0,0])
            # galil PTP motion
            else:
                # judge direction
                if flag[-1] == '+':
                    self.galil.iap(dic[int(flag[-9])-3], 5)
                else:
                    self.galil.iap(dic[int(flag[-9])-3], -5)
            '''
            pass


    def servo_monitoring(self):
        while True:
            if((self.ser1Button.isChecked() or self.ser2Button.isChecked() or
                self.ser3Button.isChecked() or self.ser4Button.isChecked() or
                self.ser5Button.isChecked() or self.ser6Button.isChecked())
                or
                (self.ser1Button.text()=='Servo ON' or self.ser2Button.text()=='Servo ON' or
                self.ser3Button.text()=='Servo ON' or self.ser4Button.text()=='Servo ON' or
                self.ser5Button.text()=='Servo ON' or self.ser6Button.text()=='Servo ON')):
                # galil request servo state
                if(self.galil.ask_servo('A')): self.ser4Button.setText('Servo ON')
                else: self.ser4Button.setText('Servo OFF')
                if(self.galil.ask_servo('B')): self.ser5Button.setText('Servo ON')
                else: self.ser4Button.setText('Servo OFF')
                if(self.galil.ask_servo('C')): self.ser6Button.setText('Servo ON')
                else: self.ser4Button.setText('Servo OFF')
                # yamaha request servo state
                state = self.yamaha.servo_state()
                if state[-1]=='1': self.ser1Button.setText('Servo ON')
                else: self.ser1Button.setText('Servo OFF')
                if state[-2]=='1': self.ser2Button.setText('Servo ON')
                else: self.ser1Button.setText('Servo OFF')
                if state[-3]=='1': self.ser3Button.setText('Servo ON')
                else: self.ser1Button.setText('Servo OFF')
            else:
                break




    def closeEvent(self, event):
        '''
        When user tries to close the window, pop out confirmation message.
        '''
        reply = QMessageBox.question(self, 'Message',
            "Are you sure to quit?", QMessageBox.Yes |
            QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            # quit
            QCoreApplication.instance().quit()
        else:
            event.ignore()


#### class for tab3 ####
class tab3UI(QTabWidget):
    def __init__(self):
        super().__init__()
        # general buttons
        self.sButton = QPushButton("Start")
        self.eButton = QPushButton("End")
        self.eButton.setEnabled(False)

        self.hbox = QHBoxLayout()
        self.vbox = QVBoxLayout()

        self.v_x = QLabel('v_x')
        self.v_y = QLabel('v_y')
        self.v_z = QLabel('v_z')
        self.unit_x = QLabel('mm/s')
        self.unit_y = QLabel('mm/s')
        self.unit_z = QLabel('mm/s')
        self.combo1 = QLabel('control object')
        self.combo2 = QLabel('velocity reference')

        self.v_xEdit = QLineEdit()
        self.v_yEdit = QLineEdit()
        self.v_zEdit = QLineEdit()

        self.cb1 = QComboBox()
        self.cb2 = QComboBox()

        self.grid = QGridLayout()

        self.initUI()

    def initUI(self):
        ## button event
        self.sButton.clicked.connect(self.start_click)
        self.eButton.clicked.connect(self.end_click)


        # vbox and hbox layout
        self.hbox.addStretch(1)
        self.hbox.addWidget(self.sButton)
        self.hbox.addWidget(self.eButton)

        self.vbox.addStretch(1)
        self.vbox.addLayout(self.hbox)

        # combo widget
        self.cb1.addItems(['Select control object', 'RCM point', 'RCM machanism'])
        self.cb2.addItems(['Select control object first!'])

        self.cb1.currentIndexChanged[str].connect(self.combo1_change)

        # grid layout
        self.grid.setSpacing(10)
        # add combo
        self.grid.addWidget(self.combo1, 1, 0)
        self.grid.addWidget(self.cb1, 1, 1)
        self.grid.addWidget(self.combo2, 2, 0)
        self.grid.addWidget(self.cb2, 2, 1)

        self.grid.addWidget(self.v_x, 3, 0)
        self.grid.addWidget(self.v_xEdit, 3, 1)
        self.grid.addWidget(self.unit_x, 3, 2)

        self.grid.addWidget(self.v_y, 4, 0)
        self.grid.addWidget(self.v_yEdit, 4, 1)
        self.grid.addWidget(self.unit_y, 4, 2)

        self.grid.addWidget(self.v_z, 5, 0)
        self.grid.addWidget(self.v_zEdit, 5, 1)
        self.grid.addWidget(self.unit_z, 5, 2)

        self.grid.addLayout(self.vbox, 10, 3)

        # holistically setup
        self.setLayout(self.grid)


    def combo1_change(self, choice):
        if choice == 'RCM machanism':
            self.cb2.clear()
            self.cb2.addItems(['system6', 'system3'])
        elif choice == 'RCM point':
            self.cb2.clear()
            self.cb2.addItems(['system0'])
        elif choice == 'Select control object':
            self.cb2.clear()
            self.cb2.addItems(['Select control object first!'])


    def start_click(self):
        self.sButton.setEnabled(False)
        self.eButton.setEnabled(True)
        self.v_xEdit.setText(self.v_yEdit.text())


    def end_click(self):
        # os.system("gnome-terminal -x bash -c '^C'&");
        # TODO: not cleanly
        # os.kill(sub.pid, signal.SIGINT)
        # print('killed')
        self.sButton.setEnabled(True)
        self.eButton.setEnabled(False)


#### class for tab4 ####
class tab4UI(QTabWidget):
    def __init__(self):
        # init of parent class
        super().__init__()

        self.text = QLabel("""
        # Construction and Implementation of Software Control Platform for Ophthalmic Operation Robot Based on ROS
        This project tries to develop a flexible and modular platform for researches into ophthalmic operation robot based on ROS.
        At present, it only provides a good software frame but not wholistic and systematic functions from input to output.
        Four devices are used:
        * Galil Motion Control Card http://www.galil.com/
        * YAMAHA SCARA
        * Hyperion Device http://www.micronoptics.com/product/hyperion-single-board-interrogator/#tab-manuals
        * Omni Phantom https://www.3dsystems.com/haptics-devices/touch

        For more infomation, see https://github.com/AndyRay1998/eye_operating_system""")

        # grid layout
        self.grid = QGridLayout()
        # UI plotting
        self.initUI()


    def initUI(self):
        # can be selected and copied
        self.text.setTextInteractionFlags(Qt.TextInteractionFlag(1))
        self.text.setAlignment(Qt.AlignLeft)
        self.text.setAlignment(Qt.AlignTop)
        self.setStyleSheet("QLabel{font-size:15px;font-weight:normal;}")
        # grid layout
        self.grid.setSpacing(10)
        # add combo
        self.grid.addWidget(self.text, 0, 0)
        # holistically setup
        self.setLayout(self.grid)


#### class for tab5 ####
class tab5UI(QTabWidget):
    def __init__(self):
        # init of parent class
        super().__init__()

        self.text = QLabel("""
        Project owner: Beihang University
        Address: No. 37 Xueyuan Road, Haidian District, Beijing, P.R. China, 100191. Tel: +86-10-82317114
        Link: https://ev.buaa.edu.cn/

        Contact: Changyi Lei
        Email NO.1: lcy1998@buaa.edu.cn
        Email NO.2: andyray1998@qq.com
        """)

        # grid layout
        self.grid = QGridLayout()
        # UI plotting
        self.initUI()


    def initUI(self):
        # can be selected and copied
        self.text.setTextInteractionFlags(Qt.TextInteractionFlag(1))
        self.text.setOpenExternalLinks(True)
        self.text.setAlignment(Qt.AlignLeft)
        self.text.setAlignment(Qt.AlignTop)
        self.setStyleSheet("QLabel{font-size:15px;font-weight:normal;}")

        # grid layout
        self.grid.setSpacing(10)
        # add combo
        self.grid.addWidget(self.text, 0, 0)
        # holistically setup
        self.setLayout(self.grid)


#### class of entrance ####
class eye_op_system(QTabWidget, QWidget):

    def __init__(self):
        super().__init__()

        self.initUI() #界面绘制交给InitUi方法


    def initUI(self):
        #### Tab views and tab UI init ####
        self.tab1 = tab1UI()
        self.addTab(self.tab1,"Operation Process")

        self.tab2 = tab2UI()
        self.addTab(self.tab2,"Adjustment and Test")

        self.tab3 = tab3UI()
        self.addTab(self.tab3,"Kinematic Simulation")

        self.tab4 = tab4UI()
        self.addTab(self.tab4,"About")

        self.tab5 = tab5UI()
        self.addTab(self.tab5,"Contact")

        #### main window setting ####
        # window position and size setting
        self.setGeometry(300, 300, 300, 220)
        # window title setup
        self.setWindowTitle('Eye Operating Robot System')
        # maximized view in default
        self.showMaximized()

        # window show
        self.show()
        thread_tab = threading.Thread(target=self.tab_restrict_thread)
        thread_tab.start()


    def tab_restrict_thread(self):
        '''
        When tab1 started, tab2 is not available.
        '''
        while True:
            time.sleep(0.5) # avoid high memory usage
            if self.tab1.exit_flag==1:
                break
            else:
                if self.tab1.sButton.isEnabled():
                    self.tab2.setEnabled(True)
                else:
                    self.tab2.setEnabled(False)


    def closeEvent(self, event):
        '''
        When user tries to close the window, pop out confirmation message.
        '''
        # cannot quit while devices are under operation
        if self.tab1.eButton.isEnabled():
            QMessageBox.warning(self, 'Warning',
                "Devices are operating. Please end first!")
        else:
            reply = QMessageBox.question(self, 'Message',
                "Are you sure to quit?", QMessageBox.Yes |
                QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                # quit
                self.tab1.exit_flag = 1
                time.sleep(1) # wait for the change of exit_flag be detected
                self.tab1.tmux_server.kill_server() # close ROS node by killing background terminal progress
                QCoreApplication.instance().quit() # close PyQt5 instance
            else:
                event.ignore()


if __name__ == '__main__':
    # avoid multiple instance being created
    if not QApplication.instance():
        app = QApplication(sys.argv)
    else:
        app = QApplication.instance()
    app.aboutToQuit.connect(app.deleteLater)
    ex = eye_op_system()
    # app.exec_()
    sys.exit(app.exec_())
