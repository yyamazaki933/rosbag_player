#!/usr/bin/env python3

import os
import sys
import yaml

from PyQt5 import uic, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox
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
        self.cb_path.currentTextChanged.connect(self.cb_path_cb)

        if self.bagdir != '':
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

        if not isValid:
            return
        
        if not os.path.exists(self.bagdir + '/metadata.yaml'):
            resp = QMessageBox.critical(self, "Error", "Rosbag is broken! \nAre you wants to reindex?", QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel, QMessageBox.StandardButton.Cancel)
            print(resp)
            if resp == QMessageBox.StandardButton.Ok:
                common.reindexBag(self.bagdir, self.ros_distro)
                self.bag_info()
            else:
                print("cancel")

    def pb_play_cb(self):
        rate = self.sb_rate.value()
        offset = self.sb_offset.value()
        if self.chb_loop.checkState() == 0:
            loop = False
        else:
            loop = True

        self.player = player.RosbagPlayer()
        self.player.playerProglessTick.connect(self.set_progress_offset)
        self.player.playerFinished.connect(self.pb_reset_cb)
        self.player.setRosbag(self.bagdir, self.duration)
        self.player.setSource(self.ros_path)
        self.player.setRate(rate)
        self.player.setStartOffset(offset)
        self.player.setLoop(loop)
        self.player.start()

        self.pb_play.setEnabled(False)
        self.sb_rate.setEnabled(False)
        self.sb_offset.setEnabled(False)
        self.chb_loop.setEnabled(False)
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
        self.chb_loop.setEnabled(True)
        self.pb_reset.setEnabled(False)
        self.pb_pause.setEnabled(False)
        self.pb_pause.setText('Pause')

    def set_progress_offset(self, value):
        self.progress.setValue(value)

if __name__ == '__main__':
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

    rosbag = ''
    try:
        rosbag = sys.argv[1]
        print("[INFO] app start with rosbag", rosbag)
    except:
        print("[INFO] app start")

    app = QApplication(sys.argv)
    with open(SCRIPT_DIR + '/ui/stylesheet.css', 'r') as f:
        style = f.read()
        app.setStyleSheet(style)

    ui_player = PlayerWindow(SCRIPT_DIR + '/ui/player.ui', "humble")
    ui_player.setWindowIcon(QtGui.QIcon(SCRIPT_DIR + '/img/rosbag_player.png'))
    ui_player.show()

    if rosbag != '':
        ui_player.set_rosbag(rosbag)
        
    sys.exit(app.exec())
