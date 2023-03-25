#!/usr/bin/env python3

import os
import sys
import yaml
import time
import re
import subprocess
import signal

from PyQt5 import QtCore, uic, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox
from PyQt5.QtGui import QTextCursor


DEFAULT_PATH = "/opt/ros/humble/setup.bash"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def execCmd(cmd, timeout=3):
    print("[INFO] execCmd():", cmd)

    resp = subprocess.run(
        cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=timeout)

    return resp


def getRosbagInfo(bagdir:str, path:str):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 bag info ' + bagdir

    resp = execCmd(cmd)

    if resp.stdout != '':
        info = ''
        dur = 0
        lines = resp.stdout.split('\n')

        for line in lines:
            if line == '':
                continue
            info += line + '\n'

            if 'Duration:' in line:
                dur = int(re.split('[:.]', line)[1])

        return True, info, dur

    else:
        return False, resp.stderr, 0


def reindexBag(bagdir:str, path:str):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 bag reindex ' + bagdir

    resp = execCmd(cmd, timeout=None)


class RosbagPlayer(QtCore.QThread):

    playerProglessTick = QtCore.pyqtSignal(int)
    playerFinished = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__(None)

        self.is_running = False
        self.path = ''
        self.rosbag_dir = ''
        self.rate = 1.0
        self.offset = 0
        self.elapsed = 0
        self.duration = 0
        self.loop = False

    def setRosbag(self, rosbag_dir:str, duration:int):
        self.rosbag_dir = rosbag_dir
        self.duration = duration

    def setSource(self, path:str):
        self.path = path

    def setRate(self, rate):
        self.rate = rate

    def setStartOffset(self, offset):
        self.offset = offset

    def setLoop(self, loop):
        self.loop = loop

    def run(self):
        cmd = 'source ' + self.path
        cmd += ' && '
        cmd += 'ros2 bag play ' + self.rosbag_dir
        cmd += ' --clock 200'
        if self.rate != 1.0:
            cmd += ' --rate ' + str(self.rate)
        if self.offset != 0:
            cmd += ' --start-offset ' + str(self.offset)
            self.elapsed = self.offset
        if self.loop:
            cmd += ' --loop'

        print("[INFO] RosbagPlayer.run():", cmd)

        self.proc = subprocess.Popen(
            cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
        self.is_running = True

        timer_tick = 1.0 / self.rate

        while True:
            if not self.is_running:
                continue

            if self.proc.poll() != None:
                self.playerFinished.emit()
                print("[INFO] RosbagPlayer Finished")
                break

            self.playerProglessTick.emit(self.elapsed)
            if self.elapsed <= self.duration:
                self.elapsed += 1

            time.sleep(timer_tick)

    def pause(self):
        cmd = 'source ' + self.path
        cmd += ' && '
        cmd += 'ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused'
        print("[INFO] RosbagPlayer.pause():", cmd)

        _ = subprocess.run(
            cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=3)

        if self.is_running:
            self.is_running = False
        else:
            self.is_running = True

    def stop(self):
        print("[INFO] RosbagPlayer.stop()")

        os.killpg(self.proc.pid, signal.SIGINT)
        self.is_running = False
        self.playerFinished.emit()


class PlayerWindow(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()
        uic.loadUi(SCRIPT_DIR + "/ui/player.ui", self)

        self.home_dir = os.getenv('HOME')
        self.player = None
        self.duration = 0
        self.log_file = SCRIPT_DIR + "/player.log"

        self.pb_bag.clicked.connect(self.pb_bag_cb)
        self.pb_path.clicked.connect(self.pb_path_cb)
        self.pb_play.clicked.connect(self.pb_play_cb)
        self.pb_pause.clicked.connect(self.pb_pause_cb)
        self.pb_reset.clicked.connect(self.pb_reset_cb)
        self.sb_offset.valueChanged.connect(self.sb_offset_cb)
        self.sb_rate.valueChanged.connect(self.sb_rate_cb)
        self.le_path.setText(DEFAULT_PATH)

        self.load_log()
        bagdir = self.le_bag.text()
        if bagdir != "":
            self.load_config()
            self.bag_info()

    def save_log(self):
        bagdir = self.le_bag.text()
        log = {'bagdir': bagdir}
        with open(self.log_file, 'w') as f:
            yaml.dump(log, f)

    def load_log(self):
        if os.path.exists(self.log_file):
            with open(self.log_file, 'r') as f:
                log = yaml.safe_load(f)
                bagdir = log['bagdir']
            self.le_bag.setText(bagdir)
        else:
            self.save_log()

    def save_config(self):
        bagdir = self.le_bag.text()
        config_file = bagdir + "/player.conf"
        path = self.le_path.text()
        start = self.sb_offset.value()
        rate = self.sb_rate.value()
        config = {
            'path': path,
            'start': start,
            'rate': rate, }
        with open(config_file, 'w') as f:
            yaml.dump(config, f)

    def load_config(self):
        bagdir = self.le_bag.text()
        config_file = bagdir + "/player.conf"
        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                path = config['path']
                start = config['start']
                rate = config['rate']
            self.le_path.setText(path)
            self.sb_offset.setValue(start)
            self.sb_rate.setValue(rate)
        else:
            self.save_config()

    def pb_bag_cb(self):
        bag = QFileDialog.getOpenFileName(
            self, 'Choose Rosbag2 File', self.home_dir, 'SQLite3 database File (*.db3)')[0]
        if bag == '':
            return
        self.set_rosbag(bag)

    def pb_path_cb(self):
        path = QFileDialog.getOpenFileName(
            self, 'Choose path file', self.home_dir, 'Bash File (setup.bash)')[0]
        if path == '':
            return
        self.le_path.setText(path)
        self.bag_info()
        self.save_config()

    def sb_rate_cb(self, value):
        print('[INFO] set rate:', value)

    def sb_offset_cb(self, value):
        self.set_progress_offset(value)
        print('[INFO] set start offset:', value)

    def set_rosbag(self, bag: str):
        bagdir = os.path.dirname(bag)
        self.le_bag.setText(bagdir)
        self.bag_info()
        self.load_config()
        self.save_log()
        print('[INFO] set rosbag dir:', bagdir)

    def bag_info(self):
        bagdir = self.le_bag.text()
        path = self.le_path.text()
        isValid, info, self.duration = getRosbagInfo(bagdir, path)

        self.pb_play.setEnabled(isValid)

        self.pte_bag.clear()
        self.pte_bag.setPlainText(info)
        self.pte_bag.setTextCursor(QTextCursor(
            self.pte_bag.document().findBlockByLineNumber(0)))
        self.progress.setRange(0, self.duration)

        if not os.path.exists(bagdir):
            return

        if not os.path.exists(bagdir + '/metadata.yaml'):
            resp = QMessageBox.critical(self, "Error", "Rosbag is broken! \nAre you wants to reindex?",
                                        QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel, QMessageBox.StandardButton.Cancel)
            print(resp)
            if resp == QMessageBox.StandardButton.Ok:
                reindexBag(bagdir, path)
                self.bag_info()
            else:
                print("cancel")

    def pb_play_cb(self):
        bagdir = self.le_bag.text()
        path = self.le_path.text()
        rate = self.sb_rate.value()
        offset = self.sb_offset.value()

        if self.chb_loop.checkState() == 0:
            loop = False
        else:
            loop = True

        self.player = RosbagPlayer()
        self.player.playerProglessTick.connect(self.set_progress_offset)
        self.player.playerFinished.connect(self.__finished_call)
        self.player.setRosbag(bagdir, self.duration)
        self.player.setSource(path)
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
        self.save_config()
        self.save_log()

    def pb_pause_cb(self):
        self.player.pause()
        if self.player.is_running:
            self.pb_pause.setText('Pause')
        else:
            self.pb_pause.setText('Resume')

    def pb_reset_cb(self):
        self.player.stop()
    
    def __finished_call(self):
        start = self.sb_offset.value()
        self.progress.setValue(start)
        self.pb_play.setEnabled(True)
        self.sb_rate.setEnabled(True)
        self.sb_offset.setEnabled(True)
        self.chb_loop.setEnabled(True)
        self.pb_reset.setEnabled(False)
        self.pb_pause.setEnabled(False)
        self.pb_pause.setText('Pause')
        self.player = None

    def set_progress_offset(self, value):
        self.progress.setValue(value)


if __name__ == '__main__':

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

    ui_player = PlayerWindow()
    ui_player.setWindowIcon(QtGui.QIcon(SCRIPT_DIR + '/img/rosbag_player.png'))
    ui_player.show()

    if rosbag != '':
        ui_player.set_rosbag(rosbag)

    sys.exit(app.exec())
