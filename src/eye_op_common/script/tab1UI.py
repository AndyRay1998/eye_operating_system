#!/usr/bin/env python3
# coding: utf-8

import sys
import os
import threading
import time
import subprocess
import rospy
from std_msgs.msg import Float32MultiArray
from omni_msgs.msg import OmniFeedback

import libtmux # reference: https://github.com/tmux-python/libtmux/

from PyQt5.QtWidgets import (QPushButton, QApplication, QWidget, QMessageBox, QTabWidget, QGridLayout,
                             QHBoxLayout, QVBoxLayout, QFormLayout, QComboBox)
from PyQt5.QtGui import QIcon, QFont
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
        self.sButton = QPushButton("启动")
        self.eButton = QPushButton("停止")
        self.qButton = QPushButton("退出")
        self.eButton.setEnabled(False)

        # device name labels
        self.gLabel = QLabel('galil')
        self.hLabel = QLabel('hyperion')
        self.oLabel = QLabel('omni')
        self.yLabel = QLabel('yamaha')
        # state label: ON / OFF
        self.gsLabel = QLabel('关')
        self.hsLabel = QLabel('关')
        self.osLabel = QLabel('关')
        self.ysLabel = QLabel('关')
        # position feed labels
        self.pfLabel1 = QLabel('1关节')
        self.pfLabel1.setFixedSize(100, 23)
        self.pfLabel2 = QLabel('2关节')
        self.pfLabel2.setFixedSize(100, 23)
        self.pfLabel3 = QLabel('3关节')
        self.pfLabel3.setFixedSize(100, 23)
        self.pfLabel4 = QLabel('4关节')
        self.pfLabel4.setFixedSize(100, 23)
        self.pfLabel5 = QLabel('5关节')
        self.pfLabel5.setFixedSize(100, 23)
        self.pfLabel6 = QLabel('6关节')
        self.pfLabel6.setFixedSize(100, 23)
        self.pfLabel7 = QLabel('7关节')
        self.pfLabel7.setFixedSize(100, 23)
        self.pfLabel8 = QLabel('8关节')
        self.pfLabel8.setFixedSize(100, 23)
        self.pf0Label = QLabel('关节位置')
        self.pf1Label = QLabel('0 度')
        self.pf2Label = QLabel('0 度')
        self.pf3Label = QLabel('0 毫米')
        self.pf4Label = QLabel('0 度')
        self.pf5Label = QLabel('0 度')
        self.pf6Label = QLabel('0 毫米')
        self.pf7Label = QLabel('0 度')
        self.pf8Label = QLabel('0')
        # speed feed labels
        self.sf0Label = QLabel('关节速度')
        self.sf1Label = QLabel('0 度/秒')
        self.sf2Label = QLabel('0 度/秒')
        self.sf3Label = QLabel('0 度/秒')
        self.sf4Label = QLabel('毫米/秒')
        self.sf5Label = QLabel('0 度/秒')
        self.sf6Label = QLabel('0 度/秒')
        self.sf7Label = QLabel('毫米/秒')
        self.sf8Label = QLabel('/')


        # grid layout
        self.grid = QGridLayout()

        # layout
        self.hbox = QHBoxLayout()
        self.vbox = QVBoxLayout()

        # exit flag, for while loop break
        self.exit_flag = 0
        # exit flag for ROS node
        self.ros_exit = 0

        # slider for speed ration adjustment
        self.ratioLabel = QLabel('速度比例：')
        self.SlideLabel = QLabel('0')
        self.splider = QSlider(Qt.Horizontal)
        self.splider.valueChanged.connect(self.valChange)
        self.splider.setMinimum(0) # 最小值
        self.splider.setMaximum(100) # 最大值
        self.splider.setSingleStep(1) # 步长
        # self.splider.setTickPosition(QSlider.TicksBelow) # 设置刻度位置，在下方
        # self.splider.setTickInterval(10) # 设置刻度间隔

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
        # position feed
        self.grid.addWidget(self.pfLabel1, 7, 1)
        self.grid.addWidget(self.pfLabel2, 7, 2)
        self.grid.addWidget(self.pfLabel3, 7, 3)
        self.grid.addWidget(self.pfLabel4, 7, 4)
        self.grid.addWidget(self.pfLabel5, 7, 5)
        self.grid.addWidget(self.pfLabel6, 7, 6)
        self.grid.addWidget(self.pfLabel7, 7, 7)
        self.grid.addWidget(self.pfLabel8, 7, 8)
        self.grid.addWidget(self.pf0Label, 8, 0)
        self.grid.addWidget(self.pf1Label, 8, 1)
        self.grid.addWidget(self.pf2Label, 8, 2)
        self.grid.addWidget(self.pf3Label, 8, 3)
        self.grid.addWidget(self.pf4Label, 8, 4)
        self.grid.addWidget(self.pf5Label, 8, 5)
        self.grid.addWidget(self.pf6Label, 8, 6)
        self.grid.addWidget(self.pf7Label, 8, 7)
        self.grid.addWidget(self.pf8Label, 8, 8)
        # speed feed
        self.grid.addWidget(self.sf0Label, 9, 0)
        self.grid.addWidget(self.sf1Label, 9, 1)
        self.grid.addWidget(self.sf2Label, 9, 2)
        self.grid.addWidget(self.sf3Label, 9, 3)
        self.grid.addWidget(self.sf4Label, 9, 4)
        self.grid.addWidget(self.sf5Label, 9, 5)
        self.grid.addWidget(self.sf6Label, 9, 6)
        self.grid.addWidget(self.sf7Label, 9, 7)
        self.grid.addWidget(self.sf8Label, 9, 8)

        self.grid.addWidget(self.ratioLabel, 6, 0)
        self.grid.addWidget(self.SlideLabel, 6, 5)
        self.grid.addWidget(self.splider, 6, 1, 1, 4)

        self.grid.addLayout(self.vbox, 10, 8)
        # holistically setup
        self.setLayout(self.grid)


    def valChange(self):
        print(self.splider.value())
        self.SlideLabel.setNum(self.splider.value()/100)


    def state_monitor_thread(self):
        '''
        While starting operating, begin state monitor in another thread.
        This will update state labels of all devices.
        '''
        time.sleep(0.1) # wait for eButton state to be changed
        while True:
            # see active ROS nodes
            nodes = subprocess.getoutput("rosnode list")
            if(self.eButton.isEnabled()==True):
                # update nodes' state
                if('Galil' in nodes): self.gsLabel.setText('开')
                else: self.gsLabel.setText('关')
                if('Hyperion' in nodes): self.hsLabel.setText('开')
                else: self.hsLabel.setText('关')
                if('Omni' in nodes): self.osLabel.setText('开')
                else: self.osLabel.setText('关')
                if('Yamaha' in nodes): self.ysLabel.setText('开')
                else: self.ysLabel.setText('关')
            else:
                # stop thread when all nodes are off
                if('Galil' in nodes or 'Hyperion' in nodes or 'Omni' in nodes or 'Yamaha' in nodes):
                    pass
                else:
                    self.gsLabel.setText('关')
                    self.hsLabel.setText('关')
                    self.osLabel.setText('关')
                    self.ysLabel.setText('关')
                    break


    def start_click_thread(self):
        self.sButton.setEnabled(False)
        self.eButton.setEnabled(True)
        # send terminal command
        self.main_pane_1.send_keys('roslaunch eye_op_common eye_op_robot.launch')
        self.main_pane_1 = self.main_win_1.panes[1]
        self.main_pane_1.send_keys('rqt_graph')
        self.UI_listener()




    def start_click(self):
        self.ros_exit = 0
        thread_1 = threading.Thread(target=self.state_monitor_thread)
        thread_2 = threading.Thread(target=self.start_click_thread)
        # thread_3 = threading.Thread(target=self.UI_listener)
        # start thread; do not change starting sequence!!
        thread_2.start()
        thread_1.start()
        time.sleep(3)
        # thread_3.start()


    def end_click(self):
        '''
        When trying to end, reinit tmux windows and change button state.
        '''
        self.ros_exit = 1
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


    def callback1(self, data):
        self.pf4Label.setText(str(data.data[0]) + '度')
        self.pf5Label.setText(str(data.data[1]) + '度')
        self.pf6Label.setText(str(data.data[2]) + '毫米')
        self.pf7Label.setText(str(data.data[3]) + '度')

    def callback2(self, data):
        self.pf1Label.setText(str(data.data[0]) + '度')
        self.pf2Label.setText(str(data.data[1]) + '度')
        self.pf3Label.setText(str(data.data[2]) + '毫米')

    def callback3(self, data):
        if self.ros_exit:
            # In UI_listener we have disable_signals=True, so we end ros like this.
            rospy.signal_shutdown('ROS shutdown')
        else:
            self.sf1Label.setText(str(data.data[0]))
            self.sf2Label.setText(str(data.data[1]))
            self.sf3Label.setText(str(data.data[2]))

    def callback4(self, data):
        data.force.x
        self.pf1Label.setText(str(data.data[0]) + '度')
        self.pf2Label.setText(str(data.data[1]) + '度')
        self.pf3Label.setText(str(data.data[2]) + '毫米')

    def UI_listener(self):
        # It is not in main thread, so we have disable_signals=True
        rospy.init_node('UI_listener', anonymous=False, disable_signals=True)
        rospy.Subscriber("galil/position", Float32MultiArray, self.callback1)
        rospy.Subscriber("yamaha/position", Float32MultiArray, self.callback2)
        rospy.Subscriber("yamaha/speed", Float32MultiArray, self.callback3)
        # TODO: rospy.Subscriber("phantom/force_feedback", OmniFeedback, self.callback4)
        rospy.spin()


    def closeEvent(self, event):
        '''
        When user tries to close the window, pop out confirmation message.
        '''
        # cannot quit while devices are under operation
        if self.eButton.isEnabled():
            QMessageBox.warning(self, '警告',
                "设备正在运行，请先结束！")
        else:
            reply = QMessageBox.question(self, '消息',
                "确定要退出吗?", QMessageBox.Yes |
                QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                # quit
                self.exit_flag = 1
                time.sleep(1)
                QCoreApplication.instance().quit()
            else:
                pass
