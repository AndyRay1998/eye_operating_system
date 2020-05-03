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
from tab1UI import tab1UI

#### class for tab2 ####
from tab2UI import tab2UI

#### class for tab3 ####
from tab3UI import tab3UI

#### class for tab4 ####
from tab4UI import tab4UI

#### class for tab5 ####
from tab5UI import tab5UI

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
