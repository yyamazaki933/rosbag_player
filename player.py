#!/usr/bin/env python3

import os
import sys
import re
import subprocess
import signal
import threading
import json

import yaml
import flet as ft
import flet_core.colors as color


ROS_DISTRO = "noetic"
DEFAULT_PATH = "/opt/ros/" + ROS_DISTRO + "/setup.bash"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def execCmd(cmd, timeout=None):
    print(cmd)
    return subprocess.run(cmd, shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=timeout)


def getRosbagInfo(bag: str):
    cmd = 'source ' + DEFAULT_PATH
    cmd += ' && '
    cmd += 'rosbag info ' + bag
    resp = execCmd(cmd)

    baginfo = {}
    if resp.stdout != '':
        desc = ""
        topics = []
        lines = resp.stdout.split('\n')

        category = ''
        for line in lines:
            if line == '':
                continue

            if "start" in line:
                start = float(re.split(r'[()]', line)[1])

            if "end" in line:
                end = float(re.split(r'[()]', line)[1])

            if 'types' in line:
                category = 'type'
                continue

            if category == 'type':
                if 'topics' in line:
                    category = 'topics'
                    topics.append(re.search(r'/[^ ]*', line).group())
                continue
                
            if category == 'topics':
                topics.append(re.search(r'/[^ ]*', line).group())
                continue

            desc += line + '\n'

        baginfo["desc"] = desc
        baginfo["start"] = start
        baginfo["end"] = end
        baginfo["duration"] = end - start
        baginfo["topics"] = topics
        return True, baginfo
    else:
        baginfo["desc"] = resp.stderr
        baginfo["start"] = 0
        baginfo["end"] = 0
        baginfo["duration"] = 0
        baginfo["topics"] = []
        return False, baginfo


def reindexBag(bagdir: str):
    cmd = 'source ' + DEFAULT_PATH
    cmd += ' && '
    cmd += 'rosbag reindex ' + bagdir
    resp = execCmd(cmd)


class PushButton(ft.ElevatedButton):
    def __init__(self, text: str = None, expand: bool = False, on_click=None, theme_color: str = None):
        super().__init__(text=text, expand=expand, on_click=on_click, bgcolor=theme_color)
        self.theme_color = theme_color

    def enable(self, state=True):
        if state:
            self.bgcolor = self.theme_color
            self.update()
            self.disabled = False
        else:
            self.bgcolor = color.GREY_900
            self.update()
            self.disabled = True


class RosbagPlayer():

    def __init__(self):
        self.home_dir = os.getenv('HOME')
        self.log_file = SCRIPT_DIR + "/player.log"
        self.startup_bag = ""
        self.paths = [DEFAULT_PATH]
        self.rate = 1.0
        self.offset = 0
        self.loop = False
        self.is_running = False
        self.is_paused = False
        self.baginfo = None

    def createSwitch(self, label, value):
        sw = ft.Switch(label=label, value=value)
        sw.active_color = color.INDIGO_ACCENT_100
        sw.track_color = color.GREY_800
        sw.inactive_thumb_color = color.GREY_600
        return sw

    def createTextField(self, label=None, value=None, expand=False, width=None):
        tf = ft.TextField(label=label, value=value, expand=expand)
        tf.width = width
        tf.border_color = color.GREY_800
        return tf

    def createToolButton(self, text=None, expand=False, on_click=None):
        return ft.ElevatedButton(text=text, on_click=on_click, expand=expand, bgcolor=color.GREY_900)

    def make_page(self, page: ft.Page):
        self.page = page
        self.page.theme = ft.Theme(color_scheme_seed="indigo")
        self.page.theme_mode = ft.ThemeMode.DARK
        # self.page.bgcolor = color.BLACK
        self.page.title = "Rosbag Player - " + ROS_DISTRO
        self.page.window_width = 1000
        self.page.window_height = 800

        self.bag_pick_dialog = ft.FilePicker(on_result=self.__bag_picked)
        self.path_pick_dialog = ft.FilePicker(on_result=self.__pth_picked)
        self.page.overlay.append(self.bag_pick_dialog)
        self.page.overlay.append(self.path_pick_dialog)

        # self.ad_ridx = ft.AlertDialog(
        #     modal=True,
        #     title=ft.Text("rosbag reindex"),
        #     content=ft.Text("metadata.yaml is not found!\nAre you wants to rosbag reindex?"),
        #     actions=[
        #         ft.TextButton("Yes", on_click=self.ad_ridx_close),
        #         ft.TextButton("No", on_click=self.ad_ridx_close),
        #     ],
        #     actions_alignment=ft.MainAxisAlignment.END,
        # )

        self.ti_bag = self.createTextField(label="Rosbag", expand=True)
        self.dd_path = ft.Dropdown(label="Path", border_color = color.GREY_800, expand=True)
        self.bt_bag = self.createToolButton(text="...", on_click=self.__bt_bag_clicked)
        self.bt_pth = self.createToolButton(text="...", on_click=self.__bt_pth_clicked)
        row1 = ft.Row(controls=[self.ti_bag, self.bt_bag])
        row2 = ft.Row(controls=[self.dd_path, self.bt_pth])

        self.tx_info = self.createTextField(label="Information", value="init", expand=True)
        self.tx_info.color = color.GREY
        self.tx_info.read_only = True
        self.tx_info.multiline = True
        self.tx_info.min_lines = 30
        self.tx_info.text_size = 14
        self.tx_info.text_style = ft.TextStyle(font_family="Ubuntu Mono")
        self.lv_tpic = ft.ListView(expand=True)
        row5 = ft.Row(controls=[self.tx_info, self.lv_tpic], expand=True, vertical_alignment=ft.CrossAxisAlignment.STRETCH)

        self.ti_rat = self.createTextField(label="Rate", value=1.0, width=100)
        self.ti_rat.suffix_text = "x"
        self.ti_rat.on_submit = self.__ti_rat_changed
        self.ti_ofs = self.createTextField(label="Start Offset", value=0, width=100)
        self.ti_ofs.suffix_text = "s"
        self.ti_ofs.on_submit = self.__ti_ofs_changed
        self.sw_lop = self.createSwitch(label="Loop", value=False)
        self.bt_play = PushButton(text="Play", on_click=self.__bt_play_clicked, expand=True, theme_color=color.INDIGO_ACCENT_400)
        self.bt_paus = PushButton(text="Pause", on_click=self.__bt_paus_call, expand=True, theme_color=color.INDIGO_ACCENT_400)
        self.bt_rset = PushButton(text="Stop", on_click=self.__bt_rset_call, expand=True, theme_color=color.INDIGO_ACCENT_400)
        row3 = ft.Row(controls=[self.ti_rat, self.ti_ofs,
                      self.sw_lop, self.bt_play, self.bt_paus, self.bt_rset])

        self.pb_prg = ft.Slider(min=0, max=100, expand=True, on_change=self.__sld_changed, disabled=True)
        self.pb_prg.thumb_color = color.INDIGO_ACCENT_100
        self.pb_prg.active_color = color.INDIGO_ACCENT_100
        self.tx_prg = ft.Text(value="-- / -- s")
        row4 = ft.Row(controls=[self.pb_prg, self.tx_prg])

        self.page.add(row1, row2, row5, row3, row4)
        self.bt_play.enable(False)
        self.bt_paus.enable(False)
        self.bt_rset.enable(False)

        self.load_log()
        if self.startup_bag:
            self.set_bag(self.startup_bag)
        self.page.update()

    def __bt_bag_clicked(self, e):
        # print("__bt_bag_clicked")
        self.bag_pick_dialog.pick_files(allowed_extensions=['bag'], allow_multiple=True)

    def __bt_pth_clicked(self, e):
        # print("__bt_pth_clicked")
        self.path_pick_dialog.pick_files(allowed_extensions=['bash'])

    def __bag_picked(self, e: ft.FilePickerResultEvent):
        print("__bag_picked:", e.files)
        bags = []
        if e.files:
            for item in e.files:
                bags.append(item.path)
            self.set_bag(bags)
            self.page.update()

    def __pth_picked(self, e: ft.FilePickerResultEvent):
        # print("__pth_picked:", e.files)
        if e.files:
            path = e.files[0].path
            self.paths.append(path)
            self.dd_path.options.append(ft.dropdown.Option(path))
            self.dd_path.value = path
            self.page.update()
            self.save_log()

    def __bt_play_clicked(self, e):
        # print("__bt_play_clicked")
        bagdir = self.ti_bag.value
        path = self.dd_path.value

        self.rate = self.ti_rat.value
        self.offset = self.ti_ofs.value
        self.loop = self.sw_lop.value

        filtered_topics = self.get_filtered_topics()

        cmd = 'source ' + path
        cmd += ' && '
        cmd += 'rosbag play ' + str.join(' ', bagdir.split(','))
        cmd += ' __name:=rosbag_player '
        cmd += ' --clock '
        if self.rate != 1.0:
            cmd += ' --rate ' + str(self.rate)
        if self.offset != 0:
            cmd += ' --start ' + str(self.offset)
        if self.loop:
            cmd += ' --loop'
        if filtered_topics:
            cmd += ' --topics ' + str.join(' ', filtered_topics)

        print(cmd)
        self.proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)
        self.is_running = True
        self.is_paused = False

        self.timer = threading.Thread(target=self.update_timer, daemon=True)
        self.timer.start()

        self.bt_play.enable(False)
        self.bt_paus.enable(True)
        self.bt_rset.enable(True)
        self.ti_rat.disabled = True
        self.ti_ofs.disabled = True
        self.sw_lop.disabled = True
        self.save_config()
        self.page.update()

    def __bt_paus_call(self, e):
        # print("__bt_paus_call")
        cmd = 'source ' + DEFAULT_PATH
        cmd += ' && '
        cmd += 'rosservice call /rosbag_player/pause_playback '

        if self.is_paused:
            cmd += 'false'
            self.is_paused = False
            self.bt_paus.text = "Pause"
        else:
            cmd += 'true'
            self.is_paused = True
            self.bt_paus.text = "Resume"
        execCmd(cmd).stdout.replace(' ', '')
        self.bt_paus.update()

    def __bt_rset_call(self, e):
        # print("__bt_rset_call")
        self.is_running = False
        self.is_paused = False
        os.killpg(self.proc.pid, signal.SIGINT)
        os.killpg(self.clk_proc.pid, signal.SIGINT)
        while True:
            if self.proc.poll() != None:
                print("player killed")
                break
        self.reset_player()

    def __ti_ofs_changed(self, e):
        # print('__ti_ofs_changed', e.control.value)
        dur = self.pb_prg.max
        try:
            int_val = int(e.control.value)
        except:
            int_val = 0
        if int_val > dur:
            int_val = dur
        self.ti_ofs.value = int_val
        self.ti_ofs.update()
        self.set_progress(int_val)

    def __ti_rat_changed(self, e):
        # print('__ti_rat_changed', e.control.value)
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
        # print('__sld_changed', e.control.value)
        int_val = int(e.control.value)
        self.ti_ofs.value = int_val
        self.tx_prg.value = str(int_val) + " / " + str(self.pb_prg.max) + " s"
        self.page.update()

    def update_timer(self):
        elapsed = self.offset

        cmd = "source " + DEFAULT_PATH + " && "
        cmd += "rostopic echo /clock -p"
        self.clk_proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, text=True, preexec_fn=os.setsid)

        last_stamp = 0
        while self.is_running:
            if self.is_paused:
                continue
            try:
                line = self.clk_proc.stdout.readline()
                now = float(line.split(',')[1]) / 1000000000.0
                now = int(now)
            except:
                continue
            if last_stamp != now:
                elapsed = now - int(self.baginfo["start"])
                self.set_progress(elapsed)
                last_stamp = now

    def reset_player(self):
        print("reset player")
        self.bt_play.enable(True)
        self.bt_paus.enable(False)
        self.bt_rset.enable(False)
        self.ti_ofs.disabled = False
        self.ti_rat.disabled = False
        self.sw_lop.disabled = False
        self.page.update()
        self.set_progress(self.offset)

    def set_progress(self, value: int):
        # print("set_progress:", value)
        self.tx_prg.value = str(value) + " / " + str(self.pb_prg.max) + " s"
        self.pb_prg.value = value
        self.tx_prg.update()
        self.pb_prg.update()

    def save_log(self):
        print("save_log:", self.log_file)
        bags = self.ti_bag.value.split(',')
        log = {
            'bags': bags,
            "paths": self.paths
        }
        with open(self.log_file, 'w') as f:
            yaml.dump(log, f)

    def load_log(self):
        print("load_log:", self.log_file)

        bags = []
        if os.path.exists(self.log_file):
            with open(self.log_file, 'r') as f:
                log = yaml.safe_load(f)
                bags = log['bags']
                self.paths = log['paths']
            self.ti_bag.value = str.join(',', bags)
            for path in self.paths:
                self.dd_path.options.append(ft.dropdown.Option(path))
                self.dd_path.value = path
        else:
            self.save_log()

        if bags:
            self.set_bag(bags)

    def save_config(self):
        bags = self.ti_bag.value.split(',')
        config_file = bags[0] + ".conf"
        print("save_config:", config_file)

        config = {
            'path': self.dd_path.value,
            'start': self.offset,
            'rate': self.rate,
            'enabled_topics': self.get_filtered_topics(),
            'loop': self.loop,
        }

        with open(config_file, 'w') as f:
            yaml.dump(config, f)

    def load_config(self):
        bags = self.ti_bag.value.split(',')
        config_file = bags[0] + ".conf"
        print("load_config:", config_file)

        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                path = config['path']
                start = config['start']
                rate = config['rate']
                topics = config['enabled_topics']
                loop = config['loop']

            self.dd_path.value = path
            self.set_progress(start)
            self.ti_ofs.value = start
            self.ti_rat.value = rate
            self.sw_lop.value = loop
            self.set_filtered_topics(topics)
        except:
            self.dd_path.value = DEFAULT_PATH
            self.set_progress(0)
            self.ti_ofs.value = 0
            self.ti_rat.value = 1.0
            self.sw_lop.value = False
            self.set_filtered_topics(self.baginfo["topics"])

    def set_bag(self, bags: list):
        print('set_bag:', bags)
        self.ti_bag.value = str.join(',', bags)
        self.get_baginfo(bags)

    def get_baginfo(self, bags: list):
        print("get_baginfo:", bags)
        self.tx_info.value = ''
        duration = 0
        topics = []
        start = 0
        for bag in bags:
            is_valid, self.baginfo = getRosbagInfo(bag)
            self.tx_info.value += self.baginfo["desc"]
            self.tx_info.value += '---\n'
            duration += self.baginfo["duration"]
            topics.extend(self.baginfo["topics"])
            if start == 0 or start > self.baginfo["start"]:
                start = self.baginfo["start"]
        self.baginfo["duration"] = duration
        self.baginfo["topics"] = list(set(topics))
        self.baginfo["start"] = start

        self.pb_prg.max = int(self.baginfo["duration"])
        self.set_progress(0)

        self.lv_tpic.clean()
        for topic in self.baginfo["topics"]:
            sw = self.createSwitch(label=topic, value=True)
            self.lv_tpic.controls.append(sw)

        if is_valid:
            self.bt_play.enable(True)
            self.pb_prg.disabled = False
            self.load_config()
            self.save_log()

    # def ad_ridx_open(self):
    #     print("ad_ridx_open")
    #     self.page.dialog = self.ad_ridx
    #     self.ad_ridx.open = True
    #     self.page.update()

    # def ad_ridx_close(self, e: ft.ControlEvent):
    #     print("ad_ridx_close", e.control.text)
    #     self.ad_ridx.open = False
    #     if e.control.text == "Yes":
    #         bagdir = self.ti_bag.value
    #         reindexBag(bagdir)
    #         self.get_baginfo(bagdir)
    #     self.page.update()

    def get_filtered_topics(self):
        print("get_filtered_topics")
        enabled_topics = []
        for control in self.lv_tpic.controls:
            if control.value:
                enabled_topics.append(control.label)
        return enabled_topics

    def set_filtered_topics(self, enabled_topics):
        print("set_filtered_topics:", enabled_topics)
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
        player.startup_bag = rosbag
        print("app start with rosbag", rosbag)
    except:
        print("app start")
    
    cmd = 'source ' + DEFAULT_PATH + ' && roscore'
    roscore = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

    ft.app(target=player.make_page)
