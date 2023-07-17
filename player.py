#!/usr/bin/env python3

import os
import sys
from time import sleep
import re
import subprocess
import signal
import threading

import yaml
import flet as ft
import flet_core.colors as color


ROS_DISTRO = "humble"
DEFAULT_PATH = "/opt/ros/" + ROS_DISTRO + "/setup.bash"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def execCmd(cmd):
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

            if 'Duration:' in line:
                dur = int(re.split('[:.]', line)[1])

            if 'Topic:' in line:
                topics.append(re.split(r'Topic: | \|', line)[1])
                continue

            info += line + '\n'

        return True, info, dur, topics

    else:
        return False, resp.stderr, 0, []


def reindexBag(bagdir:str, path:str):
    cmd = 'source ' + path
    cmd += ' && '
    cmd += 'ros2 bag reindex ' + bagdir
    resp = execCmd(cmd)


class RosbagPlayer():

    def __init__(self):
        self.home_dir = os.getenv('HOME')
        self.log_file = SCRIPT_DIR + "/player.log"
        self.rate = 1.0
        self.offset = 0
        self.loop = False
        self.is_running = False
        self.is_paused = False

    def createSwitch(self, label, value):
        sw = ft.Switch(label=label, value=value)
        sw.active_color=color.INDIGO_ACCENT_100
        sw.track_color=color.GREY_800
        sw.inactive_thumb_color=color.GREY_600
        return sw
    
    def createTextField(self, label=None, value=None, expand=False, width=None):
        tf = ft.TextField(label=label, value=value, expand=expand)
        tf.width=width
        tf.border_color=color.GREY_800
        return tf
    
    def createButton(self, text=None, expand=False, on_click=None):
        return ft.ElevatedButton(text=text, on_click=on_click, expand=expand, bgcolor=color.INDIGO_ACCENT_400)
    
    def enableButton(self, button:ft.ElevatedButton, enable:bool):
        if enable:
            button.bgcolor = color.INDIGO_ACCENT_400
            button.disabled = False
        else:
            button.bgcolor = color.GREY_900
            button.update()
            button.disabled = True

    def createToolButton(self, text=None, expand=False, on_click=None):
        return ft.ElevatedButton(text=text, on_click=on_click, expand=expand, bgcolor=color.GREY_900)

    def make_page(self, page: ft.Page):
        self.page = page
        self.page.theme = ft.Theme(color_scheme_seed="indigo")
        self.page.theme_mode = ft.ThemeMode.DARK
        # self.page.bgcolor = color.BLACK
        self.page.title = "Rosbag2 Player - " + ROS_DISTRO
        self.page.window_width = 1000
        self.page.window_height = 800

        self.bag_pick_dialog = ft.FilePicker(on_result=self.__bag_picked)
        self.path_pick_dialog = ft.FilePicker(on_result=self.__pth_picked)
        self.page.overlay.append(self.bag_pick_dialog)
        self.page.overlay.append(self.path_pick_dialog)

        self.ti_bag = self.createTextField(label="Rosbag", expand=True)
        self.ti_pth = self.createTextField(label="Path", value=DEFAULT_PATH, expand=True)
        self.bt_bag = self.createToolButton(text="...", on_click=self.__bt_bag_clicked)
        self.bt_pth = self.createToolButton(text="...", on_click=self.__bt_pth_clicked)
        row1 = ft.Row(controls=[self.ti_bag, self.bt_bag])
        row2 = ft.Row(controls=[self.ti_pth, self.bt_pth])

        self.tx_info = self.createTextField(label="Information", value="init", expand=True)
        self.tx_info.color = color.GREY
        self.tx_info.read_only = True
        self.tx_info.multiline = True
        self.tx_info.min_lines = 30
        self.lv_tpic = ft.ListView(expand=True)
        row5 = ft.Row(controls=[self.tx_info, self.lv_tpic], expand=True, vertical_alignment=ft.CrossAxisAlignment.STRETCH)

        self.ti_rat = self.createTextField(label="Rate", value=1.0, width=100)
        self.ti_rat.suffix_text = "x"
        self.ti_rat.on_submit = self.__ti_rat_changed
        self.ti_ofs = self.createTextField(label="Start Offset", value=0, width=100)
        self.ti_ofs.suffix_text = "s"
        self.ti_ofs.on_submit = self.__ti_ofs_changed
        self.sw_lop = self.createSwitch(label="Loop", value=False)
        self.bt_play = self.createButton(text="Play", on_click=self.__bt_play_clicked, expand=True)
        self.bt_paus = self.createButton(text="Pause", on_click=self.__bt_paus_call, expand=True)
        self.bt_rset = self.createButton(text="Stop", on_click=self.__bt_rset_call, expand=True)
        row3 = ft.Row(controls=[self.ti_rat, self.ti_ofs, self.sw_lop, self.bt_play, self.bt_paus, self.bt_rset])

        self.pb_prg = ft.Slider(min=0, max=100, expand=True, on_change=self.__sld_changed, disabled=True)
        self.pb_prg.thumb_color = color.INDIGO_ACCENT_100
        self.pb_prg.active_color = color.INDIGO_ACCENT_100
        self.tx_prg = ft.Text(value="-- / -- s")
        row4 = ft.Row(controls=[self.pb_prg, self.tx_prg])

        self.page.add(row1, row2, row5, row3, row4)
        self.enableButton(self.bt_play, False)
        self.enableButton(self.bt_paus, False)
        self.enableButton(self.bt_rset, False)
        self.load_log()
        self.page.update()

    def __bt_bag_clicked(self, e):
        print("__bt_bag_clicked")
        self.bag_pick_dialog.pick_files(allowed_extensions=['db3'])

    def __bt_pth_clicked(self, e):
        print("__bt_pth_clicked")
        self.path_pick_dialog.pick_files(allowed_extensions=['bash'])

    def __bag_picked(self, e: ft.FilePickerResultEvent):
        print("__bag_picked")
        if e.files:
            self.set_bag(e.files[0].path)
            self.page.update()

    def __pth_picked(self, e: ft.FilePickerResultEvent):
        print("__pth_picked")
        if e.files:
            self.ti_pth.value = e.files[0].path
            self.ti_pth.update()

    def __bt_play_clicked(self, e):
        print("__bt_play_clicked")
        bagdir = self.ti_bag.value
        path = self.ti_pth.value

        self.rate = self.ti_rat.value
        self.offset = self.pb_prg.value
        self.loop = self.sw_lop.value

        self.topics = self.__get_filtered_topics()

        cmd = 'source ' + path
        cmd += ' && '
        cmd += 'ros2 bag play ' + bagdir
        cmd += ' --clock 200'
        if self.rate != 1.0:
            cmd += ' --rate ' + str(self.rate)
        if self.offset != 0:
            cmd += ' --start-offset ' + str(self.offset)
        if self.loop:
            cmd += ' --loop'
        if self.topics:
            cmd += ' --topics ' + str.join(' ', self.topics)

        print(cmd)
        self.proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
        self.is_running = True
        self.is_paused = False

        self.timer = threading.Thread(target=self.update_timer, daemon=True)
        self.timer.start()

        self.enableButton(self.bt_play, False)
        self.enableButton(self.bt_paus, True)
        self.enableButton(self.bt_rset, True)
        self.ti_rat.disabled = True
        self.ti_ofs.disabled = True
        self.sw_lop.disabled = True
        self.save_config()
        self.save_log()
        self.page.update()

    def __bt_paus_call(self, e):
        print("__bt_paus_call")
        cmd = 'source ' + DEFAULT_PATH
        cmd += ' && '
        cmd += 'ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/srv/TogglePaused'
        execCmd(cmd)

        if self.is_paused:
            self.is_paused = False
            self.bt_paus.text = "Pause"
        else:
            self.is_paused = True
            self.bt_paus.text = "Resume"
        self.bt_paus.update()

    def __bt_rset_call(self, e):
        print("__bt_rset_call")
        os.killpg(self.proc.pid, signal.SIGINT)
        self.is_running = False

    def update_timer(self):
        timer_tick = 1.0 / self.rate
        elapsed = self.offset

        while self.is_running:
            if self.is_paused:
                continue
            sleep(timer_tick)
            elapsed += 1
            self.set_progress(elapsed)
            self.page.update()

            if self.proc.poll() != None:
                break
        self.player_finished()
    
    def player_finished(self):
        while True:
            if self.proc.poll() != None:
                print("[INFO] player finished")
                break

        self.is_running = False
        self.is_paused = False
        self.enableButton(self.bt_play, True)
        self.enableButton(self.bt_paus, False)
        self.enableButton(self.bt_rset, False)
        self.ti_ofs.disabled = False
        self.ti_rat.disabled = False
        self.sw_lop.disabled = False
        self.set_progress(self.offset)
        self.page.update()

    def __ti_ofs_changed(self, e):
        print('__ti_ofs_changed', e.control.value)
        try:
            int_val = int(e.control.value)
        except:
            int_val = 0
        if int_val > self.pb_prg.max:
            int_val = self.pb_prg.max
        self.ti_ofs.value = int_val
        self.set_progress(int_val)
        self.page.update()

    def __ti_rat_changed(self, e):
        print('__ti_rat_changed', e.control.value)
        try:
            flot_val = float(e.control.value)
        except:
            flot_val = 0
        if flot_val < 0.01:
            flot_val = 0.01
        self.ti_rat.value = flot_val
        self.ti_rat.update()

    def __sld_changed(self, e):
        if self.is_running:
            return
        print('__sld_changed', e.control.value)
        int_val = int(e.control.value)
        self.ti_ofs.value = int_val
        self.tx_prg.value = str(int_val) + " / " + str(self.pb_prg.max) + " s"
        self.page.update()

    def set_progress(self, value: int):
        self.tx_prg.value = str(value) + " / " + str(self.pb_prg.max) + " s"
        self.pb_prg.value = value

    def save_log(self):
        bagdir = self.ti_bag.value
        log = {'bagdir': bagdir}
        with open(self.log_file, 'w') as f:
            yaml.dump(log, f)

    def load_log(self):
        if os.path.exists(self.log_file):
            with open(self.log_file, 'r') as f:
                log = yaml.safe_load(f)
                bagdir = log['bagdir']
            self.ti_bag.value = bagdir
        else:
            self.save_log()

        if bagdir != "":
            self.set_bagdir(bagdir)

    def save_config(self):
        bagdir = self.ti_bag.value
        config_file = bagdir + "/player.conf"
        self.topics = self.__get_filtered_topics()

        config = {
            'path': self.ti_pth.value,
            'start': self.offset,
            'rate': self.rate,
            'enabled_topics': self.topics,
            'loop': self.loop,
        }

        if os.path.exists(bagdir):
            with open(config_file, 'w') as f:
                yaml.dump(config, f)

    def load_config(self):
        bagdir = self.ti_bag.value
        config_file = bagdir + "/player.conf"

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                path = config['path']
                start = config['start']
                rate = config['rate']
                topics = config['enabled_topics']
                loop = config['loop']

            self.ti_pth.value = path
            self.set_progress(start)
            self.ti_ofs.value = start
            self.ti_rat.value = rate
            self.sw_lop.value = loop
            self.__set_filtered_topics(topics)
        except:
            self.save_config()

    def set_bag(self, bag: str):
        self.set_bagdir(os.path.dirname(bag))

    def set_bagdir(self, bagdir: str):
        self.ti_bag.value = bagdir
        is_valid = self.get_baginfo(bagdir)
        if is_valid:
            self.bt_play.disabled = False
            self.pb_prg.disabled = False
            self.bt_play.bgcolor=color.INDIGO_ACCENT_400
            self.load_config()
            self.save_log()
            print('[INFO] set rosbag dir:', bagdir)

    def get_baginfo(self, bagdir: str):
        path = DEFAULT_PATH
        is_valid, info, duration, topics = getRosbagInfo(bagdir, path)

        self.tx_info.value = info
        self.pb_prg.max = duration
        self.set_progress(0)

        # if not os.path.exists(bagdir + '/metadata.yaml'):
        #     resp = QMessageBox.critical(self, "Error", "Rosbag is broken! \nAre you wants to reindex?",
        #                                 QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel, QMessageBox.StandardButton.Cancel)
        #     print(resp)
        #     if resp == QMessageBox.StandardButton.Ok:
        #         reindexBag(bagdir, path)
        #         self.get_baginfo()
        #     else:
        #         print("cancel")

        # self.filter_ui.topic_list.clear()

        self.lv_tpic.clean()
        for topic in topics:
            sw = self.createSwitch(label=topic, value=True)
            self.lv_tpic.controls.append(sw)

        return is_valid

    def __get_filtered_topics(self):
        enabled_topics = []
        for control in self.lv_tpic.controls:
            if control.value:
                enabled_topics.append(control.label)
        return enabled_topics

    def __set_filtered_topics(self, enabled_topics):
        if not enabled_topics:
            return
        for control in self.lv_tpic.controls:
            if control.label in enabled_topics:
                control.value = True
            else:
                control.value = False



if __name__ == '__main__':
    player = RosbagPlayer()

    rosbag = ''
    try:
        rosbag = sys.argv[1]
        print("[INFO] app start with rosbag", rosbag)
        player.set_bag(rosbag)
    except:
        print("[INFO] app start")

    ft.app(target=player.make_page)
