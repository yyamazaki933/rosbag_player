#!/usr/bin/env python3

import os
import yaml

from PyQt5 import uic, QtWidgets
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtGui import QTextCursor

import util.player as player
import util.common as common


class PlayerWindow(QtWidgets.QWidget):

    def __init__(self, ui_file:str, ros_distro:str):
        super().__init__()
        uic.loadUi(ui_file, self)

        self.home_dir = os.getenv('HOME')
        self.player = None
        self.ros_distro = ros_distro
        self.ros_path = '/opt/ros/'+ros_distro+'/setup.bash'
        self.paths = [self.ros_path]
        self.bagdir = ''
        self.conf_file = ui_file.replace('.ui', '.conf')
        self.duration = 0
        self.loop = False

        self.load_log()
        for item in self.paths:
            self.cb_path.addItem(item)
        
        self.pb_bag.clicked.connect(self.pb_bag_cb)
        self.pb_path.clicked.connect(self.pb_path_cb)
        self.pb_play.clicked.connect(self.pb_play_cb)
        self.pb_pause.clicked.connect(self.pb_pause_cb)
        self.pb_reset.clicked.connect(self.pb_reset_cb)
        self.sb_offset.valueChanged.connect(self.sb_offset_cb)
        self.sb_rate.valueChanged.connect(self.sb_rate_cb)
        self.pb_once.clicked.connect(self.pb_loop_cb)
        self.pb_loop.clicked.connect(self.pb_loop_cb)
        self.cb_path.currentTextChanged.connect(self.cb_path_cb)

        self.pb_once.setStyleSheet(
            'QPushButton {background-color: rgb(53, 53, 255);}')
        self.pb_loop.setStyleSheet(
            'QPushButton {background-color: rgb(43, 43, 43);}')

        self.bag_info()
    
    def cb_path_cb(self, text):
        self.ros_path = text

    def save_log(self):

        log = {
            'bagdir': self.bagdir,
            'paths': self.paths,
        }

        with open(self.conf_file, 'w') as file:
            yaml.dump(log, file)

    def load_log(self):
        try:
            with open(self.conf_file, 'r') as file:
                log = yaml.safe_load(file)
                self.bagdir = log['bagdir']
                self.paths = log['paths']

            self.le_bag.setText(self.bagdir)

        except FileNotFoundError:
            self.save_log()

    def pb_bag_cb(self):
        bag = QFileDialog.getOpenFileName(
            self, 'Choose Rosbag2 File', self.home_dir, 'SQLite3 database File (*.db3)')[0]
        if bag == '':
            return
        self.set_rosbag(bag)

    def pb_path_cb(self):
        path = QFileDialog.getOpenFileName(
            self, 'Add Workspace Path', self.home_dir, 'Bash File (*.bash)')[0]
        if path == '':
            return
        self.cb_path.addItem(path)
        self.paths.append(path)
        self.save_log()

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
        self.bagdir = os.path.dirname(bag)
        self.le_bag.setText(self.bagdir)
        self.bag_info()
        self.save_log()
        print('[INFO] set rosbag dir:', self.bagdir)

    def bag_info(self):
        isValid, info, self.duration = common.getRosbagInfo(
            self.ros_distro, self.bagdir)

        self.pb_play.setEnabled(isValid)

        self.pte_bag.clear()
        self.pte_bag.setPlainText(info)
        self.pte_bag.setTextCursor(QTextCursor(
            self.pte_bag.document().findBlockByLineNumber(0)))
        self.progress.setRange(0, self.duration)

    def pb_play_cb(self):
        rate = self.sb_rate.value()
        offset = self.sb_offset.value()

        self.player = player.RosbagPlayer()
        self.player.playerProglessTick.connect(self.set_progress_offset)
        self.player.playerFinished.connect(self.pb_reset_cb)
        self.player.setRosbag(self.bagdir, self.duration)
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

        common.kill_proc("bag play")

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
