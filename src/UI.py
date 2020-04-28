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
from PyQt5.QtCore import QCoreApplication, QEvent

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
        self.sButton.setEnabled(True)
        self.eButton.setEnabled(False)
        try:
            self.main_sess.kill_window(1)
        except:
            pass
        # re-init a window
        self.main_win_1 = self.main_sess.new_window(attach=False, window_name="main_win_1")
        self.main_pane_1 = self.main_win_1.split_window(attach=False)
        # choose the first pane
        self.main_pane_1 = self.main_win_1.panes[0]


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


#### class for tab2 ####
class tab2UI(QTabWidget):
    def __init__(self):
        super().__init__()

        self.sButton = QPushButton("Start")
        self.eButton = QPushButton("End")
        self.eButton.setEnabled(False)

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

        # device name labels
        self.gLabel = QLabel('galil')
        self.hLabel = QLabel('hyperion')
        self.oLabel = QLabel('omni')
        self.yLabel = QLabel('yamaha')

        # grid layout
        self.grid = QGridLayout()

        self.initUI()


    def initUI(self):
        self.sButton.clicked.connect(self.start_click)
        self.eButton.clicked.connect(self.end_click)
        # connect and disconnect
        self.gButton = QPushButton("connect galil") # galil connect
        self.hButton = QPushButton("connect hyperion") # hyperion connect
        self.oButton = QPushButton("connect omni") # omni connect
        self.yButton = QPushButton("connect yamaha") # yamaha connect
        self.gButton.clicked.connect(lambda:self.connect_click("g"))
        self.hButton.clicked.connect(lambda:self.connect_click("h"))
        self.oButton.clicked.connect(lambda:self.connect_click("o"))
        self.yButton.clicked.connect(lambda:self.connect_click("y"))

        # vbox and hbox layout
        self.hbox.addStretch(1)
        self.hbox.addWidget(self.sButton)
        self.hbox.addWidget(self.eButton)
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
        # add buttons
        self.grid.addWidget(self.gButton, 2, 2)
        self.grid.addWidget(self.hButton, 3, 2)
        self.grid.addWidget(self.oButton, 4, 2)
        self.grid.addWidget(self.yButton, 5, 2)

        self.grid.addLayout(self.vbox, 6, 3)
        # holistically setup
        self.setLayout(self.grid)

    def start_click(self):
        self.sButton.setEnabled(False)
        self.eButton.setEnabled(True)


    def end_click(self):

        self.sButton.setEnabled(True)
        self.eButton.setEnabled(False)


    def connect_click(self, flag):
        print(flag)


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

        self.tab4 = QWidget()
        self.addTab(self.tab4,"Help")
        # self.tab4UI()
        self.tab5 = QWidget()
        self.addTab(self.tab5,"Contact")
        # self.tab5UI()

        #### main window setting ####
        # window position and size setting
        self.setGeometry(300, 300, 300, 220)
        # window title setup
        self.setWindowTitle('Eye Operating Robot System')
        # window icon setting
        self.setWindowIcon(QIcon('web.png'))
        # maximized view in default
        self.showMaximized()

        # window show
        self.show()


    def closeEvent(self, event):
        '''
        When user tries to close the window, pop out confirmation message.
        '''
        reply = QMessageBox.question(self, 'Message',
            "Are you sure to quit?", QMessageBox.Yes |
            QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            # quit
            self.tab1.tmux_server.kill_server()
            QCoreApplication.instance().quit()
        else:
            event.ignore()



if __name__ == '__main__':
    if not QApplication.instance():
        app = QApplication(sys.argv)
    else:
        app = QApplication.instance()
    app.aboutToQuit.connect(app.deleteLater)
    ex = eye_op_system()
    # app.exec_()
    sys.exit(app.exec_())
