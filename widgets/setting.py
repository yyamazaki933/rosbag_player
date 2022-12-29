#!/usr/bin/env python3

import os
import yaml

from PyQt5 import uic, QtWidgets, QtCore
from PyQt5.QtWidgets import QFileDialog


class SettingWindow(QtWidgets.QWidget):

    updateCommonSetting = QtCore.pyqtSignal(list)

    def __init__(self, ui_file, conf_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)
        self.ui_file = ui_file
        self.conf_file = conf_file
        self.rosver = 2
        self.rosdistro = 'humble'
        self.rospath = 'source /opt/ros/' + self.rosdistro + '/setup.bash'

        self.pb_path.clicked.connect(self.pb_path_cb)
        self.pb_save.clicked.connect(self.save_log)

    def pb_path_cb(self):
        HOME_DIR = os.getenv('HOME')
        rospath = QFileDialog.getOpenFileName(
            self, 'Choose Optional Path File', HOME_DIR, 'Bash File (*.bash)')[0]
        if rospath == '':
            return
        self.le_path.setText(rospath)

    def save_log(self):
        rosver = self.cb_rosver.currentText()

        if rosver == 'ROS':
            self.rosver = 1
        elif rosver == 'ROS2':
            self.rosver = 2

        self.rospath = self.le_distro.text()

        self.rospath = self.le_path.text()

        log = {
            'rosver': self.rosver,
            'rosdistro': self.rosdistro,
            'rospath': self.rospath,
        }
        with open(self.conf_file, 'w') as file:
            yaml.dump(log, file)

        self.updateCommonSetting.emit([self.rosver, self.rosdistro, self.rospath])
        print("[INFO] save log:", self.conf_file)
        
    def load_log(self):
        try:
            with open(self.conf_file, 'r') as file:
                log = yaml.safe_load(file)
                self.rosver = log['rosver']
                self.rosdistro = log['rosdistro']
                self.rospath = log['rospath']

            if self.rosver == 1:
                self.cb_rosver.setCurrentText('ROS')
            elif self.rosver == 2:
                self.cb_rosver.setCurrentText('ROS2')
                
            self.le_distro.setText(self.rosdistro)
            self.le_path.setText(self.rospath)
            self.updateCommonSetting.emit([self.rosver, self.rosdistro, self.rospath])
            print("[INFO] load log:", self.conf_file)

        except FileNotFoundError:
            self.save_log()