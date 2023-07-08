#!/usr/bin/env python3

import os
import sys
import yaml
import time
import re
import subprocess
import signal

from PyQt5 import QtCore, uic, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox, QListWidgetItem
from PyQt5.QtGui import QTextCursor


DEFAULT_PATH = "/opt/ros/humble/setup.bash"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def execCmd(cmd):
    print("[INFO] execCmd():", cmd)
    return subprocess.run(cmd, shell=True, executable='/bin/bash', capture_output=True, text=True)


def getRosbagInfo(bagdir:str, path:str):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 bag info ' + bagdir

    resp = execCmd(cmd)

    if resp.stdout != '':
        info = ''
        dur = 0
        topics = []
        lines = resp.stdout.split('\n')

        for line in lines:
            if line == '':
                continue
            info += line + '\n'

            if 'Duration:' in line:
                dur = int(re.split('[:.]', line)[1])

            if 'Topic:' in line:
                topics.append(re.split(r'Topic: | \|', line)[1])

        return True, info, dur, topics

    else:
        return False, resp.stderr, 0, []


def reindexBag(bagdir:str, path:str):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 bag reindex ' + bagdir

    resp = execCmd(cmd)


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
        self.loop = False
        self.topics = []

    def setRosbag(self, rosbag_dir: str):
        self.rosbag_dir = rosbag_dir

    def setSource(self, path: str):
        self.path = path

    def setRate(self, rate):
        self.rate = rate

    def setStartOffset(self, offset):
        self.offset = offset

    def setLoop(self, loop):
        self.loop = loop

    def setPubTopics(self, topics):
        self.topics = topics

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
        if self.topics:
            cmd += ' --topics ' + str.join(' ', self.topics)

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
            time.sleep(timer_tick)
            self.elapsed += 1

    def pause(self):
        cmd = 'source ' + self.path
        cmd += ' && '
        cmd += 'ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused'
        print("[INFO] RosbagPlayer.pause():", cmd)

        _ = subprocess.run(
            cmd, shell=True, executable='/bin/bash', capture_output=True, text=True)

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

        self.filter_ui = uic.loadUi(SCRIPT_DIR + "/ui/filter.ui")

        self.home_dir = os.getenv('HOME')
        self.player = None
        self.log_file = SCRIPT_DIR + "/player.log"

        self.pb_bag.clicked.connect(self.__pb_bag_call)
        self.pb_path.clicked.connect(self.__pb_path_call)
        self.pb_play.clicked.connect(self.__pb_play_call)
        self.pb_pause.clicked.connect(self.__pb_pause_call)
        self.pb_reset.clicked.connect(self.__pb_reset_call)
        self.pb_filter.clicked.connect(self.__pb_filter_call)
        self.sb_offset.valueChanged.connect(self.__sb_offset_call)
        self.sb_rate.valueChanged.connect(self.__sb_rate_call)
        self.le_path.setText(DEFAULT_PATH)

        self.load_log()
        bagdir = self.le_bag.text()
        if bagdir != "":
            self.bag_info()
            self.load_config()

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
        filtered_topics = self.__get_filtered_topics()
        loop = int(self.chb_loop.checkState())

        config = {
            'path': path,
            'start': start,
            'rate': rate,
            'filtered_topics': filtered_topics,
            'loop': loop,
        }

        if os.path.exists(bagdir):
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
                filtered_topics = config['filtered_topics']
                loop = config['loop']

            self.__set_filtered_topics(filtered_topics)
            self.le_path.setText(path)
            self.sb_offset.setValue(start)
            self.sb_rate.setValue(rate)
            self.chb_loop.setCheckState(loop)
        else:
            self.save_config()

    def __pb_bag_call(self):
        bag = QFileDialog.getOpenFileName(
            self, 'Choose Rosbag2 File', self.home_dir, 'SQLite3 database File (*.db3)')[0]
        if bag == '':
            return
        self.set_rosbag(bag)

    def __pb_path_call(self):
        path = QFileDialog.getOpenFileName(
            self, 'Choose path file', self.home_dir, 'Bash File (setup.bash)')[0]
        if path == '':
            return
        self.le_path.setText(path)
        self.bag_info()
        self.save_config()

    def __sb_rate_call(self, value):
        print('[INFO] set rate:', value)

    def __sb_offset_call(self, value):
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
        isValid, info, duration, topics = getRosbagInfo(bagdir, path)

        self.pb_play.setEnabled(isValid)

        self.pte_bag.clear()
        self.pte_bag.setPlainText(info)
        self.pte_bag.setTextCursor(QTextCursor(
            self.pte_bag.document().findBlockByLineNumber(0)))
        self.progress.setRange(0, duration)
        self.sb_offset.setRange(0, duration)

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

        self.filter_ui.topic_list.clear()

        for topic in topics:
            item = QListWidgetItem(topic)
            item.setCheckState(QtCore.Qt.CheckState.Checked)
            self.filter_ui.topic_list.addItem(item)

    def __pb_filter_call(self):
        self.filter_ui.show()

    def __get_filtered_topics(self):
        checked_topics = []
        unchecked_cnt = 0
        for i in range(self.filter_ui.topic_list.count()):
            item = self.filter_ui.topic_list.item(i)
            if item.checkState() == QtCore.Qt.CheckState.Checked:
                checked_topics.append(item.text())
            else:
                unchecked_cnt += 1

        if unchecked_cnt == 0:
            return []
        else:
            return checked_topics

    def __set_filtered_topics(self, enabled_topics):
        if not enabled_topics:
            return
        for i in range(self.filter_ui.topic_list.count()):
            item = self.filter_ui.topic_list.item(i)
            if item.text() not in enabled_topics:
                item.setCheckState(QtCore.Qt.CheckState.Unchecked)

    def __pb_play_call(self):
        bagdir = self.le_bag.text()
        path = self.le_path.text()
        rate = self.sb_rate.value()
        offset = self.sb_offset.value()

        if self.chb_loop.checkState() == QtCore.Qt.CheckState.Checked:
            loop = True
        else:
            loop = False

        filterd_topics = self.__get_filtered_topics()

        self.player = RosbagPlayer()
        self.player.playerProglessTick.connect(self.set_progress_offset)
        self.player.playerFinished.connect(self.__finished_call)
        self.player.setRosbag(bagdir)
        self.player.setSource(path)
        self.player.setRate(rate)
        self.player.setStartOffset(offset)
        self.player.setLoop(loop)
        self.player.setPubTopics(filterd_topics)
        self.player.start()

        self.pb_play.setEnabled(False)
        self.sb_rate.setEnabled(False)
        self.sb_offset.setEnabled(False)
        self.chb_loop.setEnabled(False)
        self.pb_reset.setEnabled(True)
        self.pb_pause.setEnabled(True)
        self.save_config()
        self.save_log()

    def __pb_pause_call(self):
        self.player.pause()
        if self.player.is_running:
            self.pb_pause.setText('Pause')
        else:
            self.pb_pause.setText('Resume')

    def __pb_reset_call(self):
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
        time.sleep(1)
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
