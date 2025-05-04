#!/usr/bin/env python3

import os
import sys
import yaml
import time
import re
import subprocess
import signal
from pathlib import Path

from PyQt5 import QtCore, uic, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox, QListWidgetItem
from PyQt5.QtGui import QTextCursor


DEFAULT_PATH = "/opt/ros/noetic/setup.bash"
SCRIPT_DIR = Path(__file__).parent


def execCmd(cmd):
    print(cmd)
    return subprocess.run(cmd, shell=True, executable='/bin/bash', capture_output=True, text=True)


def getRosbagInfo(bag: str):
    cmd = 'source ' + DEFAULT_PATH
    cmd += ' && '
    cmd += 'rosbag info ' + bag
    resp = execCmd(cmd)

    baginfo = {}
    baginfo["desc"] = ''
    baginfo["start"] = 0
    baginfo["end"] = 0
    baginfo["topics"] = []
    if resp.stdout != '':
        desc = ""
        lines = resp.stdout.split('\n')

        category = ''
        for line in lines:
            if line == '':
                continue

            if "start" in line:
                baginfo["start"] = float(re.split(r'[()]', line)[1])

            if "end" in line:
                baginfo["end"] = float(re.split(r'[()]', line)[1])

            if 'types' in line:
                category = 'type'
                continue

            if category == 'type':
                if 'topics' in line:
                    category = 'topics'
                    baginfo["topics"].append(re.search(r'/[^ ]*', line).group())
                continue
                
            if category == 'topics':
                baginfo["topics"].append(re.search(r'/[^ ]*', line).group())
                continue

            desc += line + '\n'

        baginfo["desc"] = desc
        return True, baginfo
    else:
        baginfo["desc"] = resp.stderr
        return False, baginfo


def reindexBag(bag: str):
    cmd = 'source ' + DEFAULT_PATH
    cmd += ' && '
    cmd += 'rosbag reindex ' + bag
    ret = execCmd(cmd)
    if ret.returncode == 0:
        execCmd(f"mv {bag} {bag.replace('.active', '')}")
        execCmd(f"mv {bag.replace('.active', '.orig.active')} {bag}")
        return True
    print(ret.stderr)
    return False


class RosbagPlayer(QtCore.QThread):

    playerProglessTick = QtCore.pyqtSignal(int)
    playerFinished = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__(None)

        self.is_running = False
        self.path = ''
        self.bags = []
        self.rate = 1.0
        self.offset = 0
        self.elapsed = 0
        self.loop = False
        self.topics = []

    def setRosbag(self, bags: list):
        self.bags = bags

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
        cmd += 'rosbag play ' + str.join(' ', self.bags)
        cmd += ' __name:=rosbag_player '
        cmd += ' --clock '
        if self.rate != 1.0:
            cmd += ' --rate ' + str(self.rate)
        if self.offset != 0:
            cmd += ' --start ' + str(self.offset)
        if self.loop:
            cmd += ' --loop'
        if self.topics:
            cmd += ' --topics ' + str.join(' ', self.topics)

        print("[INFO] RosbagPlayer.run():", cmd)

        self.proc = subprocess.Popen(
            cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
        self.is_running = True

        timer_tick = 1.0 / self.rate

        # self.timer = threading.Thread(target=self.update_timer, daemon=True)
        # self.timer.start()

        self.elapsed = self.offset
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
        cmd = 'source ' + DEFAULT_PATH
        cmd += ' && '
        cmd += 'rosservice call /rosbag_player/pause_playback '

        if self.is_running:
            cmd += 'true'
            self.is_running = False
        else:
            cmd += 'false'
            self.is_running = True
        execCmd(cmd).stdout.replace(' ', '')

    def stop(self):
        print("[INFO] RosbagPlayer.stop()")

        os.killpg(self.proc.pid, signal.SIGINT)
        self.is_running = False
        self.playerFinished.emit()

    # def update_timer(self):
    #     elapsed = self.offset

    #     cmd = "source " + DEFAULT_PATH + " && "
    #     cmd += "rostopic echo /clock -p"
    #     self.clk_proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, text=True, preexec_fn=os.setsid)

    #     last_stamp = 0
    #     while self.is_running:
    #         if self.is_paused:
    #             continue
    #         try:
    #             line = self.clk_proc.stdout.readline()
    #             now = float(line.split(',')[1]) / 1000000000.0
    #             now = int(now)
    #         except:
    #             continue
    #         if last_stamp != now:
    #             elapsed = now - int(self.baginfo["start"])
    #             self.set_progress(elapsed)
    #             last_stamp = now


class PlayerWindow(QtWidgets.QWidget):

    def __init__(self, arg_bag=None):
        super().__init__()
        uic.loadUi(SCRIPT_DIR / "ui/player.ui", self)

        self.filter_ui = uic.loadUi(SCRIPT_DIR / "ui/filter.ui")

        self.home_dir = os.getenv('HOME')
        self.player = None
        self.log_file = SCRIPT_DIR / "player.log"

        self.pb_bag.clicked.connect(self.__pb_bag_call)
        self.pb_path.clicked.connect(self.__pb_path_call)
        self.pb_play.clicked.connect(self.__pb_play_call)
        self.pb_pause.clicked.connect(self.__pb_pause_call)
        self.pb_stop.clicked.connect(self.__pb_stop_call)
        self.pb_filter.clicked.connect(self.__pb_filter_call)
        self.sb_offset.valueChanged.connect(self.__set_progress)
        self.sb_rate.valueChanged.connect(self.__sb_rate_call)
        self.slider.valueChanged.connect(self.__set_progress)

        self.le_path.setText(DEFAULT_PATH)
        self.pb_pause.setEnabled(False)
        self.pb_stop.setEnabled(False)

        if arg_bag and Path(arg_bag).suffix == '.active':
            ret = QMessageBox.critical(self, "Error", "Bloken rosbag detected! Exec rosbag reindex?", QMessageBox.Yes|QMessageBox.Cancel)
            if ret == QMessageBox.Yes:
                reindexBag(arg_bag)
                arg_bag = arg_bag.replace('.active', '')

        self.load_log(arg_bag)

    def save_log(self):
        bags_str = self.le_bag.text()
        path = self.le_path.text()

        if bags_str:
            bags = bags_str.split(',')
        else:
            bags = ''

        log = {
            'bags': bags,
            'path': path
            }

        with open(self.log_file, 'w') as f:
            yaml.dump(log, f)

    def load_log(self, arg_bag=None):
        if os.path.exists(self.log_file):
            with open(self.log_file, 'r') as f:
                log = yaml.safe_load(f)
                if arg_bag:
                    bags = [arg_bag]
                else:
                    bags = log['bags']
                if bags:
                    self.set_rosbag(bags)
                self.le_path.setText(log['path'])
        else:
            self.save_log()

    def save_config(self):
        bags_str = self.le_bag.text()
        bags = bags_str.split(',')
        config_file = bags[0] + ".conf"
        start = self.sb_offset.value()
        rate = self.sb_rate.value()
        filtered_topics = self.__get_filtered_topics()
        loop = int(self.chb_loop.checkState())

        config = {
            'start': start,
            'rate': rate,
            'filtered_topics': filtered_topics,
            'loop': loop,
        }

        with open(config_file, 'w') as f:
            yaml.dump(config, f)

    def load_config(self):
        bags_str = self.le_bag.text()
        bags = bags_str.split(',')
        config_file = bags[0] + ".conf"

        if os.path.exists(config_file):
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                start = config['start']
                rate = config['rate']
                filtered_topics = config['filtered_topics']
                loop = config['loop']

            self.__set_filtered_topics(filtered_topics)
            self.__set_progress(start)
            self.sb_rate.setValue(rate)
            self.chb_loop.setCheckState(loop)

    def __pb_bag_call(self):
        bags = QFileDialog.getOpenFileNames(
            self, 'Choose Rosbag Files', self.home_dir, 'Rosbag (*.bag)')[0]
        if bags == []:
            return
        self.set_rosbag(bags)

    def __pb_path_call(self):
        path = QFileDialog.getOpenFileName(
            self, 'Choose path file', '/opt/ros', 'Bash File (setup.bash)')[0]
        if path == '':
            return
        self.le_path.setText(path)
        self.save_config()

    def __sb_rate_call(self, value):
        print('[INFO] set rate:', value)

    def __set_progress(self, value):
        if not self.player:
            self.sb_offset.setValue(value)
        self.slider.setValue(value)
        self.label_time.setText(f"{value} / {self.slider.maximum()}")

    def set_rosbag(self, bags: list):
        self.le_bag.setText(str.join(',', bags))
        self.show_bag_info()
        self.load_config()
        self.save_log()
        print('[INFO] set rosbag:', str.join(',', bags))

    def show_bag_info(self):
        bags_str = self.le_bag.text()
        bags = bags_str.split(',')

        topics = []
        start = 0
        end = 0
        baginfo = ''
        for bag in bags:
            _, info = getRosbagInfo(bag)
            baginfo += info["desc"]
            baginfo += '---\n'
            topics.extend(info["topics"])
            if start == 0 or info["start"] < start:
                start = info["start"]
            if end == 0 or info["end"] > end:
                end = info["end"]
        # _, info = getRosbagInfo(bags_str.split(',')[0])
        # baginfo += info["desc"]
        # topics = info["topics"]
        # start = info["start"]
        # end = info["end"]

        self.pb_play.setEnabled(True)
        self.pte_bag.clear()
        self.pte_bag.setPlainText(baginfo)
        self.pte_bag.setTextCursor(QTextCursor(
            self.pte_bag.document().findBlockByLineNumber(0)))

        duration = int(end - start)
        self.slider.setRange(0, duration)
        self.sb_offset.setRange(0, duration)
        self.__set_progress(0)

        self.filter_ui.topic_list.clear()
        topics = list(set(topics))
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
        bags = self.le_bag.text()
        path = self.le_path.text()
        rate = self.sb_rate.value()
        offset = self.sb_offset.value()

        if self.chb_loop.checkState() == QtCore.Qt.CheckState.Checked:
            loop = True
        else:
            loop = False

        filterd_topics = self.__get_filtered_topics()

        self.player = RosbagPlayer()
        self.player.playerProglessTick.connect(self.__set_progress)
        self.player.playerFinished.connect(self.__finished_call)
        self.player.setRosbag(bags.split(','))
        self.player.setSource(path)
        self.player.setRate(rate)
        self.player.setStartOffset(offset)
        self.player.setLoop(loop)
        self.player.setPubTopics(filterd_topics)
        self.player.start()

        self.pb_play.setEnabled(False)
        self.pb_pause.setEnabled(True)
        self.pb_stop.setEnabled(True)
        self.sb_rate.setEnabled(False)
        self.sb_offset.setEnabled(False)
        self.chb_loop.setEnabled(False)
        self.save_config()
        self.save_log()

    def __pb_pause_call(self):
        self.player.pause()
        if self.player.is_running:
            self.pb_pause.setText('Pause')
        else:
            self.pb_pause.setText('Resume')

    def __pb_stop_call(self):
        self.player.stop()

    def __finished_call(self):
        start = self.sb_offset.value()
        self.__set_progress(start)
        self.pb_play.setEnabled(True)
        self.pb_pause.setEnabled(False)
        self.pb_stop.setEnabled(False)
        self.sb_rate.setEnabled(True)
        self.sb_offset.setEnabled(True)
        self.chb_loop.setEnabled(True)
        time.sleep(1)
        self.player = None

    def closeEvent(self, e):
        self.save_config()
        self.save_log()
        return super().closeEvent(e)


if __name__ == '__main__':
    rosbag = None
    try:
        rosbag = sys.argv[1]
        print("[INFO] app start with rosbag", rosbag)
    except:
        print("[INFO] app start")

    cmd = f'source {DEFAULT_PATH} && roscore'
    roscore = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

    app = QApplication(sys.argv)

    ui_player = PlayerWindow(rosbag)
    ui_player.setWindowIcon(QtGui.QIcon(str(SCRIPT_DIR / 'img/rosbag_player.png')))
    ui_player.show()

    sys.exit(app.exec())
