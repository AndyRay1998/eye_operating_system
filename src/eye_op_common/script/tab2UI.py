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


class tab2UI(QTabWidget):
    def __init__(self):
        super().__init__()
        # create yamaha instance for sending command
        sys.path.append(subprocess.getoutput("rospack find yamaha_mixed") + "/script")
        from yamaha_serial import command
        # here we assign port to "" just for null initialization; do not change it here!!!
        self.yamaha = command(port="")
        # TODO: confirm port number of YAMAHA
        self.yamaha_port = "TODO"

        # create galil instance for sending command
        sys.path.append(subprocess.getoutput("rospack find galil_mixed") + "/script")
        from galil_command import g_control
        # here we assign port to "" just for null initialization; do not change it here!!!
        self.galil = g_control(ip_address="")
        # TODO: confirm ip of GALIL
        self.galil_ip = "TODO"

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
        # button event: click evoke
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

        ## grid layout
        self.grid.setSpacing(10)
        # add labels
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

        # holistically setup
        self.setLayout(self.grid)


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
            # map string to param names
            param = eval(flag)
            if param.isChecked():
                # TODO: determine what kind of status to be used /
                # go check yamaha programming manual
                servo_status = "ON"
            else:
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
            axis = int(flag[-9])
            param = eval(f'self.d{axis}Edit')
            try:
                dis = flag[-1] + param.text()
            except:
                # default distance is 5
                dis = flag[-1] + '5'
                param.setText('5')
            dis = int(dis)
            # TODO: convert all control value(dis) to cts unit
            # dis.convert_to_cts()

            # TODO: uncomment and test
            '''
            # YAMAHA PTP motion
            if axis==1 or axis==2 or axis==3:
                dis_list = [0, 0, 0, 0, 0, 0]
                dis_list[axis-1] = dis
                print(dis_list)

                self.yamaha.movei(dis_list)
            # galil PTP motion
            else:
                self.galil.iap(dic[axis-3], dis)
            '''
