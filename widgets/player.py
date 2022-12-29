#!/usr/bin/env python3

import os
import re
import yaml
import subprocess

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtGui import QTextCursor

import util.player as player
import util.common as common
from widgets.setting import SettingWindow


class PlayerWindow(QtWidgets.QWidget):

    def __init__(self, ui_file, conf_file, *args, **kwargs):
        super().__init__(*args, **kwargs)
        uic.loadUi(ui_file, self)

        self.home_dir = os.getenv('HOME')
        self.player = None
        self.ros_ver = 1
        self.ros_distro = 'humble'
        self.ros_path = '/opt/ros/' + self.ros_distro + '/setup.bash'
        self.ui_file = ui_file
        self.conf_file = conf_file
        self.duration = 0
        self.loop = False

        setting_ui_file = ui_file.replace('player.ui', 'setting.ui')
        setting_conf_file = setting_ui_file.replace(
            '/ui/setting.ui', '/config/setting.conf')

        self.load_log()
        self.bag_info()

        self.pb_bag.clicked.connect(self.pb_bag_cb)
        self.pb_play.clicked.connect(self.pb_play_cb)
        self.pb_pause.clicked.connect(self.pb_pause_cb)
        self.pb_reset.clicked.connect(self.pb_reset_cb)
        self.sb_offset.valueChanged.connect(self.sb_offset_cb)
        self.sb_rate.valueChanged.connect(self.sb_rate_cb)
        self.pb_setting.clicked.connect(self.pb_setting_cb)
        self.pb_once.clicked.connect(self.pb_loop_cb)
        self.pb_loop.clicked.connect(self.pb_loop_cb)

        self.ui_setting = SettingWindow(setting_ui_file, setting_conf_file)
        self.ui_setting.updateCommonSetting.connect(self.set_common_setting)
        self.ui_setting.load_log()

        self.pb_once.setStyleSheet(
            'QPushButton {background-color: rgb(53, 53, 255);}')
        self.pb_loop.setStyleSheet(
            'QPushButton {background-color: rgb(43, 43, 43);}')

    def save_log(self):
        bagdir = self.le_bag.text()

        log = {
            'rosbag_dir': bagdir,
        }

        with open(self.conf_file, 'w') as file:
            yaml.dump(log, file)

    def load_log(self):
        try:
            with open(self.conf_file, 'r') as file:
                log = yaml.safe_load(file)
                bagdir = log['rosbag_dir']

            self.le_bag.setText(bagdir)

        except FileNotFoundError:
            self.save_log()

    def set_common_setting(self, settings):
        self.ros_ver = settings[0]
        self.ros_distro = settings[1]
        self.ros_path = settings[2]

    def pb_setting_cb(self):
        self.ui_setting.show()

    def pb_bag_cb(self):
        bag = QFileDialog.getOpenFileName(
            self, 'Choose Rosbag2 File', self.home_dir, 'SQLite3 database File (*.db3)')[0]
        self.set_rosbag(bag)

    def pb_loop_cb(self):
        if self.loop:
            self.loop = False
            self.pb_once.setStyleSheet(
                'QPushButton {background-color: rgb(53, 53, 255);}')
            self.pb_loop.setStyleSheet(
                'QPushButton {background-color: rgb(43, 43, 43);}')
        else:
            self.loop = True
            self.pb_loop.setStyleSheet(
                'QPushButton {background-color: rgb(53, 53, 255);}')
            self.pb_once.setStyleSheet(
                'QPushButton {background-color: rgb(43, 43, 43);}')
        print('[INFO] set loop:', self.loop)

    def sb_rate_cb(self, value):
        print('[INFO] set rate:', value)

    def sb_offset_cb(self, value):
        self.set_progress_offset(value)
        print('[INFO] set start offset:', value)

    def set_rosbag(self, bag):
        bagdir = os.path.dirname(bag)

        if bagdir != '':
            bagdir = bagdir
            self.le_bag.setText(bagdir)
            self.bag_info()
            self.save_log()
            print('[INFO] set rosbag dir:', bagdir)

    def bag_info(self):
        bagdir = self.le_bag.text()

        isValid, info, self.duration = common.getRosbagInfo(self.ros_distro, bagdir)
        
        self.pb_play.setEnabled(isValid)

        self.pte_bag.clear()
        self.pte_bag.setPlainText(info)

        self.progress.setRange(0, self.duration)
        self.progress.reset()

        row0 = self.pte_bag.document().findBlockByLineNumber(0)
        self.pte_bag.setTextCursor(QTextCursor(row0))

    def pb_play_cb(self):
        bagdir = self.le_bag.text()
        rate = self.sb_rate.value()
        offset = self.sb_offset.value()

        self.player = player.RosbagPlayer()
        self.player.playerProglessTick.connect(self.set_progress_offset)
        self.player.playerFinished.connect(self.pb_reset_cb)
        self.player.setRosbag(bagdir, self.duration)
        self.player.setSource(self.ros_path)
        self.player.setRate(rate)
        self.player.setStartOffset(offset)
        self.player.setLoop(self.loop)
        self.player.start()

        self.pb_play.setEnabled(False)
        self.sb_rate.setEnabled(False)
        self.sb_offset.setEnabled(False)
        self.pb_reset.setEnabled(True)
        self.pb_pause.setEnabled(True)

    def pb_pause_cb(self):
        self.player.pause()
        if self.player.is_running:
            self.pb_pause.setText('Pause')
        else:
            self.pb_pause.setText('Resume')

    def pb_reset_cb(self):
        self.player.stop()
        self.player = None

        common.kill_proc("ros2 bag play")

        start = self.sb_offset.value()
        self.progress.setValue(start)
        self.pb_play.setEnabled(True)
        self.sb_rate.setEnabled(True)
        self.sb_offset.setEnabled(True)
        self.pb_reset.setEnabled(False)
        self.pb_pause.setEnabled(False)
        self.pb_pause.setText('Pause')

    def set_progress_offset(self, value):
        self.progress.setValue(value)
