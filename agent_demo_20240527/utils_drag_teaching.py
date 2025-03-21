import json
import numpy as np
import os
import time
import threading
import logging
import traceback
from contextlib import nullcontext
from copy import copy

import cv2
import torch
from termcolor import colored

# 假设这些模块来自 lerobot 项目
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
from lerobot.common.robot_devices.utils import busy_wait
from lerobot.common.utils.utils import get_safe_torch_device

# 语言字典，支持中英文
LANGUAGES = {
    "zh": {
        "welcome": "欢迎使用拖动示教程序，请选择语言：\n1. 中文\n2. 英文\n输入 1 或 2：",
        "invalid_language": "无效的选择，默认使用中文",
        "start_teaching": "开始拖动示教...",
        "bus_connected": "舵机总线连接成功",
        "bus_failed": "舵机总线连接失败: {}",
        "menu": "\n拖动示教 (STS3215版本)\nq: 退出程序\nr: 开始录制动作\nc: 停止录制动作\np: 回放动作\ns: 保存录制动作（需先录制，再按's'或数字键1～10）\nl: 读取本地动作\nm: 放松主臂\nu: 锁定主臂\nn: 放松从臂\nv: 锁定从臂\no: 进入远程遥控模式\nx: 退出远程遥控模式\n----------------------------------",
        "exit": "正在退出程序...",
        "prepare_record": "准备录制动作...",
        "start_record": "开始录制动作",
        "recording": "录制中... 时间: {:.2f}s, 位置: {}",
        "record_end": "录制结束，共记录 {} 个位置点",
        "stop_record": "停止录制动作",
        "thread_warning": "警告：录制线程未及时停止",
        "no_data_edit": "当前没有录制数据",
        "edit_info": "录制数据包含 {} 个位置点",
        "trim_start": "删除前3帧？(y/n): ",
        "trim_end": "删除后3帧？(y/n): ",
        "edit_result": "编辑后剩余 {} 个位置点",
        "prepare_stop": "准备停止录制...",
        "prepare_play": "准备回放动作...",
        "start_play": "开始回放动作",
        "input_action": "请输入要回放的动作编号（1-10）：",
        "input_speed": "请输入回放速度倍率（0.5-2.0，默认1.0）：",
        "action_not_exist": "动作 {} 不存在。",
        "invalid_input": "无效输入，请输入 1 到 10 之间的数字。",
        "play_error": "回放过程中出错: {}",
        "prepare_save": "准备保存动作...",
        "no_data_save": "没有动作数据可保存",
        "input_action_save": "请输入要保存的动作编号（1-10）：",
        "input_name": "请输入动作名称（可选，直接回车跳过）：",
        "action_saved": "动作 {} 已保存。",
        "input_filename_save": "请输入保存文件名（默认 recorded_actions.json）：",
        "save_success": "动作数据已保存到 {}",
        "save_failed": "保存动作数据失败: {}",
        "prepare_load": "准备加载动作...",
        "input_filename_load": "请输入加载文件名（默认 recorded_actions.json）：",
        "load_success": "成功加载动作数据从 {}",
        "load_failed": "加载动作数据失败: {}",
        "file_not_found": "未找到文件 {}",
        "config_failed": "加载配置文件失败: {}，使用默认设置",
        "torque_off_warning": "警告：机械臂扭矩已关闭，请确保安全！",
        "torque_error": "设置扭矩出错: {}",
        "arm_relaxed": "机械臂已放松",
        "arm_locked": "机械臂已锁定",
        "invalid_key": "无效按键 '{}’，请按菜单中的选项操作",
        "interrupt": "程序被用户中断",
        "error": "程序出错: {}",
        "end": "程序结束",
        "master_torque_off": "主臂扭矩关闭，可能下落，请注意安全！",
        "slave_torque_off": "从臂扭矩关闭，可能下落，请注意安全！",
        "master_torque_on": "主臂扭矩开启",
        "slave_torque_on": "从臂扭矩开启",
        "remote_mode_started": "进入远程遥控模式。",
        "remote_mode_stopped": "退出远程遥控模式。",
        "already_in_remote_mode": "已处于远程遥控模式中",
        "not_in_remote_mode": "当前不在远程遥控模式中"
    },
    "en": {
        "welcome": "Welcome to the Drag Teaching Program, please select a language:\n1. Chinese\n2. English\nEnter 1 or 2: ",
        "invalid_language": "Invalid selection, defaulting to Chinese",
        "start_teaching": "Starting drag teaching...",
        "bus_connected": "Motor bus connected successfully",
        "bus_failed": "Motor bus connection failed: {}",
        "menu": "\nDrag Teaching (STS3215 Version)\nq: Quit program\nr: Start recording action\nc: Stop recording action\np: Play back action\ns: Save recorded action (after recording, press 's' or number key 1-10)\nl: Load recorded actions from local\nm: Relax master arm\nu: Lock master arm\nn: Relax slave arm\nv: Lock slave arm\no: Enter remote teleoperation mode\nx: Exit remote teleoperation mode\n----------------------------------",
        "exit": "Exiting program...",
        "prepare_record": "Preparing to record action...",
        "start_record": "Starting action recording",
        "recording": " Recording... Time: {:.2f}s, Positions: {}",
        "record_end": "Recording ended, total {} position points recorded",
        "stop_record": "Recording stopped",
        "thread_warning": "Warning: Recording thread did not stop in time",
        "no_data_edit": "No recorded data available for editing",
        "edit_info": "Current recording contains {} position points",
        "trim_start": "Remove the first 3 frames? (y/n): ",
        "trim_end": "Remove the last 3 frames? (y/n): ",
        "edit_result": "Remaining {} position points after editing",
        "prepare_stop": "Preparing to stop recording...",
        "prepare_play": "Preparing to play back action...",
        "start_play": "Starting action playback",
        "input_action": "Enter the action number to play back (1-10): ",
        "input_speed": "Enter playback speed factor (0.5-2.0, default 1.0): ",
        "action_not_exist": "Action {} does not exist.",
        "invalid_input": "Invalid input, please enter a number between 1 and 10.",
        "play_error": "Error during playback: {}",
        "prepare_save": "Preparing to save action...",
        "no_data_save": "No action data available to save",
        "input_action_save": "Enter the action number to save (1-10): ",
        "input_name": "Enter action name (optional, press Enter to skip): ",
        "action_saved": "Action {} saved.",
        "input_filename_save": "Enter save filename (default recorded_actions.json): ",
        "save_success": "Action data saved to {}",
        "save_failed": "Failed to save action data: {}",
        "prepare_load": "Preparing to load action...",
        "input_filename_load": "Enter load filename (default recorded_actions.json): ",
        "load_success": "Successfully loaded action data from {}",
        "load_failed": "Failed to load action data: {}",
        "file_not_found": "File {} not found",
        "config_failed": "Failed to load config: {} using defaults",
        "torque_off_warning": "Warning: Arm torque disabled, ensure safety!",
        "torque_error": "Error setting torque: {}",
        "arm_relaxed": "Arm relaxed",
        "arm_locked": "Arm locked",
        "invalid_key": "Invalid key '{}', please use options from the menu",
        "interrupt": "Program interrupted by user",
        "error": "Program error: {}",
        "end": "Program ended",
        "master_torque_off": "Master arm torque disabled, ensure safety!",
        "slave_torque_off": "Slave arm torque disabled, ensure safety!",
        "master_torque_on": "Master arm torque enabled",
        "slave_torque_on": "Slave arm torque enabled",
        "remote_mode_started": "Entered remote teleoperation mode.",
        "remote_mode_stopped": "Exited remote teleoperation mode.",
        "already_in_remote_mode": "Already in remote teleoperation mode",
        "not_in_remote_mode": "Not in remote teleoperation mode"
    }
}

# 全局变量
current_language = "zh"  # 默认中文
LAST_USED_FILE = "recorded_actions.json"  # 默认保存文件

def select_language():
    """选择语言"""
    global current_language
    print(LANGUAGES["zh"]["welcome"])
    choice = input().strip()
    if choice == "2":
        current_language = "en"
        print("Language set to English")
    elif choice != "1":
        print(LANGUAGES["zh"]["invalid_language"])
    else:
        print("语言设置为中文")

def load_config(config_file="config.json"):
    """加载配置文件"""
    try:
        with open(config_file, "r") as f:
            config = json.load(f)
            if "master" not in config or "slave" not in config:
                raise ValueError("Config must contain 'master' and 'slave' sections")
            return config
    except Exception as e:
        print(LANGUAGES[current_language]["config_failed"].format(e))
        return {
            "master": {"port": "COM5", "baudrate": 1000000, "motor_ids": [1, 2, 3, 4, 5, 6]},
            "slave": {"port": "COM6", "baudrate": 1000000, "motor_ids": [7, 8, 9, 10, 11, 12]},
            "sample_interval": 0.03
        }

def initialize_motors(config):
    """初始化主从臂舵机总线"""
    master_motors = {f"joint_{i+1}": (motor_id, "sts3215") for i, motor_id in enumerate(config["master"]["motor_ids"])}
    master_bus_config = FeetechMotorsBusConfig(port=config["master"]["port"], motors=master_motors, mock=False)
    master_bus = FeetechMotorsBus(master_bus_config)
    try:
        master_bus.connect()
        print(LANGUAGES[current_language]["bus_connected"])
    except Exception as e:
        print(LANGUAGES[current_language]["bus_failed"].format(e))
        raise

    slave_motors = {f"joint_{i+1}": (motor_id, "sts3215") for i, motor_id in enumerate(config["slave"]["motor_ids"])}
    slave_bus_config = FeetechMotorsBusConfig(port=config["slave"]["port"], motors=slave_motors, mock=False)
    slave_bus = FeetechMotorsBus(slave_bus_config)
    try:
        slave_bus.connect()
        print(LANGUAGES[current_language]["bus_connected"])
    except Exception as e:
        print(LANGUAGES[current_language]["bus_failed"].format(e))
        raise

    return master_bus, slave_bus

# 模拟 lerobot 中的 log_control_info 和 busy_wait 使用
def log_control_info(robot_bus, dt_s, fps=None):
    msg = f"dt: {dt_s*1000:.2f} ms ({1/dt_s:.1f} Hz)"
    if fps is not None and (1/dt_s) < fps - 1:
        msg = colored(msg, "yellow")
    logging.info(msg)

# TeachingTest 类：实现录制、回放及远程遥控模式
class TeachingTest:
    def __init__(self, master_bus, slave_bus):
        self.master_bus = master_bus
        self.slave_bus = slave_bus
        self.recording = False
        self.remote_mode = False
        self.remote_thread = None
        self.record_list = {}
        self.current_recording = []
        self.fps = 30  # 默认遥控频率
        self.load_from_local(LAST_USED_FILE)

    def set_master_torque(self, enable):
        try:
            self.master_bus.write("Torque_Enable", enable)
            print(LANGUAGES[current_language]["master_torque_on"] if enable else LANGUAGES[current_language]["master_torque_off"])
        except Exception as e:
            print(LANGUAGES[current_language]["torque_error"].format(e))

    def set_slave_torque(self, enable):
        try:
            self.slave_bus.write("Torque_Enable", enable)
            print(LANGUAGES[current_language]["slave_torque_on"] if enable else LANGUAGES[current_language]["slave_torque_off"])
        except Exception as e:
            print(LANGUAGES[current_language]["torque_error"].format(e))

    def remote_mode_loop(self, fps):
        sample_interval = 1.0 / fps
        while self.remote_mode:
            loop_start = time.perf_counter()
            master_positions = self.master_bus.read("Present_Position")
            if master_positions is not None:
                self.slave_bus.write("Goal_Position", master_positions)
            dt = time.perf_counter() - loop_start
            busy_wait(max(0, sample_interval - dt))
            log_control_info(self.master_bus, dt, fps)
        print("\n" + LANGUAGES[current_language]["remote_mode_stopped"])

    def start_remote_mode(self):
        if not self.remote_mode:
            self.remote_mode = True
            self.remote_thread = threading.Thread(target=self.remote_mode_loop, args=(self.fps,), daemon=True)
            self.remote_thread.start()
            print(LANGUAGES[current_language]["remote_mode_started"])
        else:
            print(LANGUAGES[current_language]["already_in_remote_mode"])

    def stop_remote_mode(self):
        if self.remote_mode:
            self.remote_mode = False
            self.remote_thread.join(timeout=1.0)
            if self.remote_thread.is_alive():
                print(LANGUAGES[current_language]["thread_warning"])
            else:
                print(LANGUAGES[current_language]["remote_mode_stopped"])
        else:
            print(LANGUAGES[current_language]["not_in_remote_mode"])

    def record(self, sample_interval=0.015):
        if self.remote_mode:
            print(LANGUAGES[current_language]["already_in_remote_mode"])
            return
        self.current_recording = []
        self.recording = True

        def _record():
            local_sample_interval = sample_interval
            start_time = time.perf_counter()
            # 设置较低加速度和较高速度，保证平滑跟踪
            self.slave_bus.write("Acceleration", 50)
            self.slave_bus.write("Goal_Speed", 4000)
            while self.recording:
                loop_start = time.perf_counter()
                try:
                    master_positions = self.master_bus.read("Present_Position")
                    if master_positions is not None:
                        self.slave_bus.write("Goal_Position", master_positions)
                        self.current_recording.append(master_positions.tolist())
                        elapsed = time.perf_counter() - start_time
                        print(f"\r{LANGUAGES[current_language]['recording'].format(elapsed, master_positions)}", end="")
                except Exception as e:
                    print(f"\n{LANGUAGES[current_language]['torque_error'].format(e)}")
                dt = time.perf_counter() - loop_start
                busy_wait(max(0, local_sample_interval - dt))
            print(f"\n{LANGUAGES[current_language]['record_end'].format(len(self.current_recording))}")

        print(LANGUAGES[current_language]["start_record"])
        self.record_t = threading.Thread(target=_record, daemon=True)
        self.record_t.start()

    def stop_record(self):
        if self.recording:
            self.recording = False
            self.record_t.join(timeout=1.0)
            if self.record_t.is_alive():
                print(LANGUAGES[current_language]["thread_warning"])
            else:
                print(LANGUAGES[current_language]["stop_record"])
            self.edit_recording()

    def edit_recording(self):
        if not self.current_recording:
            print(LANGUAGES[current_language]["no_data_edit"])
            return
        print(LANGUAGES[current_language]["edit_info"].format(len(self.current_recording)))
        trim_start = input(LANGUAGES[current_language]["trim_start"]).lower() == 'y'
        trim_end = input(LANGUAGES[current_language]["trim_end"]).lower() == 'y'
        if trim_start:
            self.current_recording = self.current_recording[3:]
        if trim_end and len(self.current_recording) > 3:
            self.current_recording = self.current_recording[:-3]
        print(LANGUAGES[current_language]["edit_result"].format(len(self.current_recording)))

    def save_recording(self):
        # 原有保存流程（会提示输入动作编号），保留供菜单选择 's' 时调用
        if not self.current_recording:
            print(LANGUAGES[current_language]["no_data_save"])
            return
        action_number = input(LANGUAGES[current_language]["input_action_save"])
        if action_number.isdigit() and 1 <= int(action_number) <= 10:
            action_number = int(action_number)
            action_name = input(LANGUAGES[current_language]["input_name"])
            self.record_list[action_number] = {
                "positions": self.current_recording.copy(),
                "name": action_name or f"Action_{action_number}",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            print(LANGUAGES[current_language]["action_saved"].format(action_number))
            filename = input(LANGUAGES[current_language]["input_filename_save"]) or "recorded_actions.json"
            global LAST_USED_FILE
            LAST_USED_FILE = filename
            self.save_to_file(filename)
        else:
            print(LANGUAGES[current_language]["invalid_input"])

    def save_recording_with_number(self, action_number):
        # 新增功能：直接使用菜单中数字键保存当前录制动作
        if not self.current_recording:
            print(LANGUAGES[current_language]["no_data_save"])
            return
        action_name = input(LANGUAGES[current_language]["input_name"])
        self.record_list[action_number] = {
            "positions": self.current_recording.copy(),
            "name": action_name or f"Action_{action_number}",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        print(LANGUAGES[current_language]["action_saved"].format(action_number))
        filename = input(LANGUAGES[current_language]["input_filename_save"]) or "recorded_actions.json"
        global LAST_USED_FILE
        LAST_USED_FILE = filename
        self.save_to_file(filename)

    def save_to_file(self, filename="recorded_actions.json"):
        try:
            with open(filename, "w") as f:
                json.dump(self.record_list, f, indent=4)
            print(LANGUAGES[current_language]["save_success"].format(filename))
        except Exception as e:
            print(LANGUAGES[current_language]["save_failed"].format(e))

    def load_from_local(self, filename="recorded_actions.json"):
        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    data = json.load(f)
                    converted_data = {int(k): v for k, v in data.items() if "positions" in v}
                    self.record_list = converted_data
                print(LANGUAGES[current_language]["load_success"].format(filename))
            except Exception as e:
                print(LANGUAGES[current_language]["load_failed"].format(e))
        else:
            print(LANGUAGES[current_language]["file_not_found"].format(filename))

    def play(self):
        if self.remote_mode:
            print(LANGUAGES[current_language]["already_in_remote_mode"])
            return
        print(LANGUAGES[current_language]["start_play"])
        action_number = input(LANGUAGES[current_language]["input_action"])
        if action_number.isdigit() and 1 <= int(action_number) <= 10:
            action_number = int(action_number)
            if action_number in self.record_list:
                try:
                    speed_factor = float(input(LANGUAGES[current_language]["input_speed"]) or 1.0)
                except Exception:
                    speed_factor = 1.0
                speed_factor = max(0.5, min(2.0, speed_factor))
                positions_to_play = self.record_list[action_number]["positions"]
                try:
                    self.slave_bus.write("Acceleration", 100)
                    BASE_SPEED = int(8000 * speed_factor)
                    self.slave_bus.write("Goal_Speed", BASE_SPEED)
                    for i in range(len(positions_to_play)-1):
                        start_pos = np.array(positions_to_play[i], dtype=np.int32)
                        end_pos = np.array(positions_to_play[i+1], dtype=np.int32)
                        diff = end_pos - start_pos
                        max_diff = np.max(np.abs(diff))
                        steps = 3 if max_diff < 100 else 5 if max_diff < 300 else 7
                        for pos in self.interpolate_positions(start_pos, end_pos, steps):
                            self.slave_bus.write("Goal_Position", pos)
                            time.sleep(0.03 / speed_factor)
                except Exception as e:
                    print(LANGUAGES[current_language]["play_error"].format(e))
            else:
                print(LANGUAGES[current_language]["action_not_exist"].format(action_number))
        else:
            print(LANGUAGES[current_language]["invalid_input"])

    def interpolate_positions(self, start_pos, end_pos, steps):
        result = []
        for i in range(steps):
            t = i / (steps - 1)
            t = 2 * t * t if t < 0.5 else 1 - ((-2 * t + 2) ** 2) / 2
            pos = start_pos + (end_pos - start_pos) * t
            result.append(np.round(pos).astype(np.int32))
        return result

    def print_menu(self):
        print(LANGUAGES[current_language]["menu"])

    def start(self):
        self.print_menu()
        # 使用 msvcrt 读取键盘（适用于 Windows）
        import msvcrt
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                if key == "q":
                    print(LANGUAGES[current_language]["exit"])
                    break
                elif key == "r":
                    print(LANGUAGES[current_language]["prepare_record"])
                    config = load_config()
                    self.record(config["sample_interval"])
                elif key == "c":
                    print(LANGUAGES[current_language]["prepare_stop"])
                    self.stop_record()
                elif key == "p":
                    print(LANGUAGES[current_language]["prepare_play"])
                    self.play()
                elif key == "s":
                    print(LANGUAGES[current_language]["prepare_save"])
                    self.save_recording()
                elif key == "l":
                    print(LANGUAGES[current_language]["prepare_load"])
                    filename = input(LANGUAGES[current_language]["input_filename_load"]) or LAST_USED_FILE
                    self.load_from_local(filename)
                elif key == "m":
                    self.set_master_torque(0)
                elif key == "n":
                    self.set_slave_torque(0)
                elif key == "u":
                    self.set_master_torque(1)
                elif key == "v":
                    self.set_slave_torque(1)
                elif key == "o":
                    self.start_remote_mode()
                elif key == "x":
                    self.stop_remote_mode()
                # 新增：数字键1-9以及0代表动作编号1-10的快速保存
                elif key.isdigit():
                    action_number = int(key) if key != "0" else 10
                    print(f"\n保存当前录制动作到编号 {action_number} ...")
                    self.save_recording_with_number(action_number)
                else:
                    print(LANGUAGES[current_language]["invalid_key"].format(key))
        self.master_bus.disconnect()
        self.slave_bus.disconnect()

def drag_teach():
    print(LANGUAGES[current_language]["start_teaching"])
    config = load_config()
    master_bus, slave_bus = initialize_motors(config)
    recorder = TeachingTest(master_bus, slave_bus)
    recorder.start()

if __name__ == '__main__':
    select_language()
    try:
        drag_teach()
    except KeyboardInterrupt:
        print(LANGUAGES[current_language]["interrupt"])
    except Exception as e:
        print(LANGUAGES[current_language]["error"].format(e))
    finally:
        print(LANGUAGES[current_language]["end"])
