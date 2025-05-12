#!/usr/bin/env python3
import os, sys, signal, json, re
from time import sleep
from pathlib import Path
from subprocess import run, Popen, PIPE, STDOUT

from PyQt5 import QtCore, uic, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox, QListWidgetItem, QProgressDialog
from PyQt5.QtGui import QTextCursor

DEF_ROS_PATH = "/opt/ros/noetic/setup.bash"
SCRIPT_DIR = Path(__file__).parent

def execCmd(cmd):
    print(cmd)
    return run(cmd, shell=True, executable='/bin/bash', capture_output=True, text=True)

def getRosbagInfo(bag:str, rospath:str):
    cmd = f'source {rospath} && rosbag info {bag}'
    resp = execCmd(cmd)

    baginfo = {}
    baginfo["desc"] = ''
    baginfo["start"] = 0
    baginfo["end"] = 0
    baginfo["topics"] = []
    if resp.stdout != '':
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

            baginfo["desc"] += line + '\n'
        return True, baginfo
    else:
        baginfo["desc"] = resp.stderr
        return False, baginfo

def reindexBag(bag:str, rospath:str):
    cmd = f'source {rospath} && rosbag reindex ' + bag
    ret = execCmd(cmd)
    if ret.returncode == 0:
        execCmd(f"mv {bag} {bag.replace('.active', '')}")
        execCmd(f"mv {bag.replace('.active', '.orig.active')} {bag}")
        return True
    print(ret.stderr)
    return False

class StdoutMonitor(QtCore.QThread):
    msgUpdated = QtCore.pyqtSignal(str)

    def __init__(self, proc):
        super().__init__(None)
        self.proc = proc
        self.stopping = False

    def run(self):
        while (not self.stopping):
            line = self.proc.stdout.readline()
            self.msgUpdated.emit(line)
    
    def stop(self):
        self.stopping = True

class PlayerWindow(QtWidgets.QWidget):

    def __init__(self, arg_bag=None):
        super().__init__()
        uic.loadUi(SCRIPT_DIR / "ui/player.ui", self)

        self.rospath    = DEF_ROS_PATH
        self.proc       = None
        self.start      = 0
        self.end        = 0
        self.offset     = 0
        self.elapsed_t  = 0
        self.log_file   = SCRIPT_DIR / "player.log"

        self.pb_bag.clicked.connect(self.__pb_bag_call)
        self.pb_play.clicked.connect(self.__pb_play_call)
        self.pb_pause.clicked.connect(self.__pb_pause_call)
        self.pb_stop.clicked.connect(self.__pb_stop_call)
        self.slider.valueChanged.connect(self.__set_progress)

        self.pb_pause.setEnabled(False)
        self.pb_stop.setEnabled(False)

        log = self.load_log()

        if arg_bag and Path(arg_bag).suffix == '.active':
            ret = QMessageBox.critical(self, "Error", "Bloken rosbag detected! Exec rosbag reindex?", QMessageBox.Yes|QMessageBox.Cancel)
            if ret == QMessageBox.Yes:
                reindexBag(arg_bag, self.rospath)
                exit()

        cmd = f'source {self.rospath} && roscore'
        self.roscore = Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

        if arg_bag:
            self.set_rosbag([arg_bag])
        else:
            self.set_rosbag(log['bags'])
            self.set_filtered_topics(log['topicfilter'])
            self.__set_progress(log['start'])
            self.sb_rate.setValue(log['rate'])
            self.chb_loop.setCheckState(log['loop'])

    def save_log(self):
        bags_str = self.le_bag.text()
        rate    = self.sb_rate.value()
        topics  = self.get_filtered_topics()
        loop    = int(self.chb_loop.checkState())

        if bags_str:
            bags = bags_str.split(',')
        else:
            bags = ''

        log = {
            'rospath': self.rospath,
            'bags': bags,
            'start': self.offset,
            'rate': rate,
            'topicfilter': topics,
            'loop': loop,
        }
        with open(self.log_file, 'w') as f:
            json.dump(log, f, indent=2)
        return log

    def load_log(self):
        try:
            with open(self.log_file, 'r') as f:
                log = json.load(f)
            self.rospath = log['rospath']
            return log
        except Exception as e:
            print("[ERROR] load_log: ", e)
            return self.save_log()

    def __pb_bag_call(self):
        home_dir = os.getenv('HOME')
        bags = QFileDialog.getOpenFileNames(self, 'Choose Rosbag Files', home_dir, 'Rosbag (*.bag)')[0]
        if bags == []:
            return
        self.set_rosbag(bags)

    def set_rosbag(self, bags):
        if not bags:
            self.pb_play.setEnabled(False)
            return
        self.le_bag.setText(str.join(',', bags))

        prog_ui = QProgressDialog(labelText="Loading rosbags...", parent=self)
        prog_ui.setWindowModality(QtCore.Qt.WindowModal)
        prog_ui.setWindowTitle('Info')
        prog_ui.setMaximum(len(bags))
        prog_ui.setCancelButton(None)
        prog_ui.show()

        topics = []
        self.start = 0
        self.end = 0
        baginfo = ''
        allvalid = True
        loaded = 0
        for bag in bags:
            QApplication.processEvents()
            valid, info = getRosbagInfo(bag, self.rospath)
            baginfo += info["desc"]
            baginfo += '---\n'
            topics.extend(info["topics"])
            if self.start == 0 or info["start"] < self.start:
                self.start = info["start"]
            if self.end == 0 or info["end"] > self.end:
                self.end = info["end"]
            if not valid:
                allvalid = False
            loaded += 1
            prog_ui.setValue(loaded)
        prog_ui.close()

        if allvalid:
            self.pb_play.setEnabled(True)
        else:
            self.pb_play.setEnabled(False)

        self.pte_bag.clear()
        self.pte_bag.setPlainText(baginfo)
        self.pte_bag.setTextCursor(QTextCursor(
            self.pte_bag.document().findBlockByLineNumber(0)))

        duration = int(self.end - self.start)
        self.slider.setRange(0, duration)
        self.__set_progress(0)
        self.label_start_t.setText(str(self.start))

        self.sb_rate.setValue(1)
        self.chb_loop.setCheckState(QtCore.Qt.CheckState.Unchecked)

        self.topic_list.clear()
        topics = list(set(topics))
        for topic in topics:
            item = QListWidgetItem(topic)
            item.setCheckState(QtCore.Qt.CheckState.Checked)
            self.topic_list.addItem(item)

        print('[INFO] set rosbag:', str.join(',', bags))

    def get_filtered_topics(self):
        checked_topics = []
        unchecked_cnt = 0
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            if item.checkState() == QtCore.Qt.CheckState.Checked:
                checked_topics.append(item.text())
            else:
                unchecked_cnt += 1

        if unchecked_cnt == 0:
            return []
        else:
            return checked_topics

    def set_filtered_topics(self, enabled_topics):
        if not enabled_topics:
            return
        for i in range(self.topic_list.count()):
            item = self.topic_list.item(i)
            if item.text() not in enabled_topics:
                item.setCheckState(QtCore.Qt.CheckState.Unchecked)

    def __pb_play_call(self):
        bags = self.le_bag.text()
        rate = self.sb_rate.value()
        filterd_topics = self.get_filtered_topics()

        cmd = f"source {self.rospath} && rosbag play {bags.replace(',', ' ')} __name:=rosbag_player --clock"
        if rate != 1.0:
            cmd += f' --rate {rate}'
        if self.offset != 0:
            cmd += f' --start {self.offset}'
        if self.chb_loop.checkState() == QtCore.Qt.CheckState.Checked:
            cmd += ' --loop'
        if filterd_topics:
            cmd += ' --topics ' + str.join(' ', filterd_topics)
        print(cmd)
        self.proc = Popen(cmd, shell=True, executable='/bin/bash', stdout=PIPE, stderr=STDOUT, text=True, preexec_fn=os.setsid)

        self.monitor = StdoutMonitor(self.proc)
        self.monitor.msgUpdated.connect(self.__stdout_call)
        self.monitor.start()

        self.timer = QtCore.QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.__timer_call)
        self.timer.start()

        self.elapsed_t = 0
        self.pb_play.setEnabled(False)
        self.pb_pause.setEnabled(True)
        self.pb_stop.setEnabled(True)
        self.wd_input.setEnabled(False)
        self.wd_config.setEnabled(False)

    def __stdout_call(self, msg: str):
        msg = msg.strip()
        if not msg:
            return
        print(msg)
        
        match = re.search(r'Duration: [\d]+', msg)
        if not match:
            return
        elapsed_t = int(re.search(r'[\d]+', match.group()).group())
        if elapsed_t != self.elapsed_t:
            self.elapsed_t = elapsed_t
            self.__set_progress(self.offset + self.elapsed_t)

        match = re.search(r'Time: [\d]+.[\d]+', msg)
        if not match:
            return
        curr_t = re.search(r'[\d]+.[\d]+', match.group()).group()
        self.label_start_t.setText(curr_t[:13])

    def __timer_call(self):
        if self.proc.poll() is None:
            return
        self.timer.stop()
        self.monitor.stop()
        self.timer = None
        self.monitor = None
        self.proc = None
        self.pb_pause.setEnabled(False)
        print("[INFO] rosbag play completed")

    def __set_progress(self, value):
        if not self.proc:
            self.offset = value
            self.label_start_t.setText(str(self.start+value))
        self.slider.setValue(value)
        self.label_time.setText(f"{value} / {self.slider.maximum()}")

    def __pb_pause_call(self):
        cmd = f'source {self.rospath} && rosservice call /rosbag_player/pause_playback '
        if self.pb_pause.text() == ('Pause'):
            cmd += 'true'
            self.pb_pause.setText('Resume')
        else:
            cmd += 'false'
            self.pb_pause.setText('Pause')
        execCmd(cmd).stdout.replace(' ', '')

    def __pb_stop_call(self):
        if self.proc:
            os.killpg(self.proc.pid, signal.SIGINT)
            while self.proc:
                print("[INFO] stopping...")
                QApplication.processEvents()
                sleep(0.1)

        self.__set_progress(self.offset)
        self.pb_play.setEnabled(True)
        self.pb_pause.setEnabled(False)
        self.pb_pause.setText('Pause')
        self.pb_stop.setEnabled(False)
        self.wd_input.setEnabled(True)
        self.wd_config.setEnabled(True)
        print("[INFO] player reset")

    def closeEvent(self, e):
        os.killpg(self.roscore.pid, signal.SIGINT)
        self.save_log()
        return super().closeEvent(e)

if __name__ == '__main__':
    rosbag = None
    try:
        rosbag = sys.argv[1]
        print("[INFO] app start with rosbag", rosbag)
    except:
        print("[INFO] app start")

    app = QApplication(sys.argv)
    ui_player = PlayerWindow(rosbag)
    ui_player.setWindowIcon(QtGui.QIcon(str(SCRIPT_DIR / 'img/rosbag_player.png')))
    ui_player.show()
    sys.exit(app.exec())
