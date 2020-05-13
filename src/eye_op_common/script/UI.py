#!/usr/bin/env python3
# coding: utf-8

# NOTE: $ ./UI.py to run this scripy

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
from tab1UI import tab1UI

#### class for tab2 ####
from tab2UI import tab2UI

#### class for tab3 ####
# NOTE: tab3 was used for kinematic simulation, but I thought it was unnecessary,
# so neither do I implement it, nor did I finish tab3UI.py
# from tab3UI import tab3UI

#### class for tab4 ####
from tab4UI import tab4UI

#### class for tab5 ####
from tab5UI import tab5UI

#### class of entrance ####
class eye_op_system(QTabWidget, QWidget):

    def __init__(self):
        super().__init__()

        self.initUI() # interface plotting


    def initUI(self):
        #### Tab views and tab UI init ####
        self.tab1 = tab1UI()
        self.addTab(self.tab1, "手术进程")
        self.tab1.sButton.clicked.connect(self.tab_restrict)
        self.tab1.eButton.clicked.connect(self.tab_restrict)

        self.tab2 = tab2UI()
        self.addTab(self.tab2, "调整和测试")

        # NOTE: I dont think this tab is necessary in user interface
        # self.tab3 = tab3UI()
        # self.addTab(self.tab3,"Kinematic Simulation")

        self.tab4 = tab4UI()
        self.addTab(self.tab4, "关于")

        self.tab5 = tab5UI()
        self.addTab(self.tab5, "联系方式")

        #### main window setting ####
        # window position and size setting
        self.setGeometry(300, 300, 300, 220)
        # window title setup
        self.setWindowTitle('Eye Operating Robot System')
        # maximized view in default
        self.showMaximized()

        # window show
        self.show()


    def servo_monitoring(self):
        '''
        monitor and change servo state indicator in tab2
        '''
        while True:
            time.sleep(0.5) # avoid high memory usage
            if self.tab1.exit_flag==0:
                # galil request servo state
                if(self.tab2.galil.ask_servo('A')): self.tab2.ser4Button.setText('Servo ON')
                else: self.tab2.ser4Button.setText('Servo OFF')
                if(self.tab2.galil.ask_servo('B')): self.tab2.ser5Button.setText('Servo ON')
                else: self.tab2.ser4Button.setText('Servo OFF')
                if(self.tab2.galil.ask_servo('C')): self.tab2.ser6Button.setText('Servo ON')
                else: self.tab2.ser4Button.setText('Servo OFF')
                # yamaha request servo state
                self.tab2.yamaha.servo_state()
                # TODO: test this ser_read
                state = self.tab2UI.ser_read()
                if state[-1]=='1': self.tab2.ser1Button.setText('Servo ON')
                else: self.tab2.ser1Button.setText('Servo OFF')
                if state[-2]=='1': self.tab2.ser2Button.setText('Servo ON')
                else: self.tab2.ser1Button.setText('Servo OFF')
                if state[-3]=='1': self.tab2.ser3Button.setText('Servo ON')
                else: self.tab2.ser1Button.setText('Servo OFF')
            else:
                break


    def tab_restrict(self):
        '''
        When tab1 started, tab2 is not available.
        '''
        if self.tab1.sButton.isEnabled():
            self.tab2.setEnabled(True)
        else:
            self.tab2.setEnabled(False)


    def closeEvent(self, event):
        '''
        Rewrite closeEvent method.
        When user tries to close the window, pop out confirmation message.
        '''
        # cannot quit while devices are under operation
        if self.tab1.eButton.isEnabled():
            event.ignore()
            QMessageBox.warning(self, '警告',
                "设备正在运行，请先结束！")
        else:
            reply = QMessageBox.question(self, '消息',
                "确定要退出吗?", QMessageBox.Yes |
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
