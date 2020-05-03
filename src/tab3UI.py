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
