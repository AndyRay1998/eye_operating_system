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
