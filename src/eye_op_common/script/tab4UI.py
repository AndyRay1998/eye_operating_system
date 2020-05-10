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
