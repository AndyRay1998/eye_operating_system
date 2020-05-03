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
