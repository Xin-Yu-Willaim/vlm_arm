"""
拖动示教程序 (STS3215版本)
作者: 同济子豪兄 (优化版，支持中英文)
功能:
- q: 退出程序
- r: 开始录制动作
- c: 停止录制并编辑
- p: 回放动作（支持速度调节）
- s: 保存动作到指定文件
- l: 从指定文件加载动作
- f: 放松机械臂
- t: 锁定机械臂
使用说明:
1. 确保舵机连接到指定串口（如COM5）
2. 修改 config.json 配置串口和舵机ID
3. 运行程序，按菜单提示操作
注意事项:
- 放松机械臂时注意安全
- 保存动作时建议命名，便于管理
"""

import json
import numpy as np
import os
import time
import threading
from serial.tools import list_ports
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

# 语言字典
LANGUAGES = {
    "zh": {
        "welcome": "欢迎使用拖动示教程序，请选择语言：\n1. 中文\n2. 英文\n输入 1 或 2：",
        "invalid_language": "无效的选择，默认使用中文",
        "start_teaching": "开始拖动示教...",
        "bus_connected": "舵机总线连接成功",
        "bus_failed": "舵机总线连接失败: {}",
        "menu": "\n拖动示教 同济子豪兄 (STS3215版本)\nq: 退出程序\nr: 开始录制动作\nc: 停止录制动作\np: 回放动作\ns: 将录制的动作保存到本地\nl: 从本地读取录制好的动作\nf: 放松机械臂 (关闭扭矩)\nt: 锁定机械臂 (开启扭矩)\n----------------------------------",
        "exit": "正在退出程序...",
        "prepare_record": "准备录制动作...",
        "start_record": "开始录制动作",
        "recording": " Recording... 时间: {:.2f}s, 位置: {}",
        "record_end": "录制结束，共记录 {} 个位置点",
        "stop_record": "停止录制动作",
        "thread_warning": "警告：录制线程未及时停止",
        "no_data_edit": "当前没有录制数据可编辑",
        "edit_info": "当前录制包含 {} 个位置点",
        "trim_start": "是否删除前3帧？(y/n): ",
        "trim_end": "是否删除后3帧？(y/n): ",
        "edit_result": "编辑后剩余 {} 个位置点",
        "prepare_stop": "准备停止录制...",
        "prepare_play": "准备回放动作...",
        "start_play": "开始回放动作",
        "input_action": "请输入要回放的动作编号（1-10）：",
        "input_speed": "请输入回放速度倍率（0.5-2.0，默认1.0）：",
        "action_not_exist": "动作 {} 不存在。",
        "invalid_input": "无效的输入，请输入 1 到 10 之间的数字。",
        "play_error": "回放过程中出错: {}",
        "prepare_save": "准备保存动作...",
        "no_data_save": "没有可保存的动作数据",
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
        "file_not_found": "没有找到文件 {}",
        "config_failed": "加载配置文件失败: {}，使用默认设置",
        "torque_off_warning": "警告：机械臂扭矩已关闭，可能下落，请确保安全！",
        "torque_error": "设置扭矩出错: {}",
        "arm_relaxed": "机械臂已放松",
        "arm_locked": "机械臂已锁定",
        "invalid_key": "无效按键 '{}’，请按菜单中的选项操作",
        "interrupt": "程序被用户中断",
        "error": "程序出错: {}",
        "end": "程序结束"
    },
    "en": {
        "welcome": "Welcome to the Drag Teaching Program, please select a language:\n1. Chinese\n2. English\nEnter 1 or 2: ",
        "invalid_language": "Invalid selection, defaulting to Chinese",
        "start_teaching": "Starting drag teaching...",
        "bus_connected": "Motor bus connected successfully",
        "bus_failed": "Motor bus connection failed: {}",
        "menu": "\nDrag Teaching by Tongji Zihao (STS3215 Version)\nq: Quit program\nr: Start recording action\nc: Stop recording action\np: Play back action\ns: Save recorded actions locally\nl: Load recorded actions from local\nf: Relax arm (disable torque)\nt: Lock arm (enable torque)\n----------------------------------",
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
        "config_failed": "Failed to load config: {}, using defaults",
        "torque_off_warning": "Warning: Arm torque disabled, it may drop, ensure safety!",
        "torque_error": "Error setting torque: {}",
        "arm_relaxed": "Arm relaxed",
        "arm_locked": "Arm locked",
        "invalid_key": "Invalid key '{}', please use options from the menu",
        "interrupt": "Program interrupted by user",
        "error": "Program error: {}",
        "end": "Program ended"
    }
}

# 全局语言变量和上次使用的文件名
current_language = "zh"  # 默认中文
LAST_USED_FILE = "recorded_actions.json"  # 默认文件，程序启动时尝试加载

def select_language():
    """提示用户选择语言"""
    global current_language
    print(LANGUAGES["zh"]["welcome"])  # 初始提示总是中文
    choice = input().strip()
    if choice == "2":
        current_language = "en"
        print("Language set to English")
    elif choice != "1":
        print(LANGUAGES["zh"]["invalid_language"])
    else:
        print("语言设置为中文")

def load_config(config_file="config.json"):
    try:
        with open(config_file, "r") as f:
            return json.load(f)
    except Exception as e:
        print(LANGUAGES[current_language]["config_failed"].format(e))
        return {"port": "COM5", "baudrate": 1000000, "motor_ids": [1, 2, 3, 4, 5, 6], "sample_interval": 0.01}

def initialize_motors(config):
    motors = {}
    for i, motor_id in enumerate(config["motor_ids"]):
        motors[f"joint_{i+1}"] = (motor_id, "sts3215")
    bus_config = FeetechMotorsBusConfig(port=config["port"], motors=motors, mock=False)
    bus = FeetechMotorsBus(bus_config)
    try:
        bus.connect()
        print(LANGUAGES[current_language]["bus_connected"])
        return bus
    except Exception as e:
        print(LANGUAGES[current_language]["bus_failed"].format(e))
        raise

class TeachingTest:
    def __init__(self, bus):
        self.bus = bus
        self.recording = False
        self.record_list = {}
        self.current_recording = []
        self.load_from_local(LAST_USED_FILE)  # 初始化时加载上次使用的文件

    def get_all_positions(self):
        try:
            positions = self.bus.read("Present_Position")
            return positions
        except Exception as e:
            print(LANGUAGES[current_language]["torque_error"].format(e))
            return None

    def set_all_torque(self, enable):
        try:
            self.bus.write("Torque_Enable", enable)
            if not enable:
                print(LANGUAGES[current_language]["torque_off_warning"])
        except Exception as e:
            print(LANGUAGES[current_language]["torque_error"].format(e))

    def record(self, sample_interval=0.01):
        self.current_recording = []
        self.recording = True
        start_time = time.time()
        
        def _record():
            last_time = time.time()
            min_interval = sample_interval
            positions_buffer = []
            
            while self.recording:
                current_time = time.time()
                if current_time - last_time >= min_interval:
                    try:
                        positions = self.get_all_positions()
                        if positions is not None:
                            positions_buffer.append(positions)
                            if len(positions_buffer) >= 3:
                                smoothed = np.round(np.mean(positions_buffer[-3:], axis=0)).astype(np.int32)
                                self.current_recording.append(smoothed.tolist())
                                print(f"\r{LANGUAGES[current_language]['recording'].format(current_time - start_time, smoothed)}", end="")
                        last_time = current_time
                    except Exception as e:
                        print(f"\n{LANGUAGES[current_language]['torque_error'].format(e)}")
                        time.sleep(0.1)
                else:
                    time.sleep(0.001)
            print(f"\n{LANGUAGES[current_language]['record_end'].format(len(self.current_recording))}")
        
        print(f"\n{LANGUAGES[current_language]['start_record']}")
        self.record_t = threading.Thread(target=_record, daemon=True)
        self.record_t.start()

    def stop_record(self):
        if self.recording:
            self.recording = False
            self.record_t.join(timeout=1.0)
            if self.record_t.is_alive():
                print(f"\n{LANGUAGES[current_language]['thread_warning']}")
            else:
                print(f"\n{LANGUAGES[current_language]['stop_record']}")
            self.edit_recording()

    def edit_recording(self):
        if not self.current_recording:
            print(f"\n{LANGUAGES[current_language]['no_data_edit']}")
            return
        print(f"\n{LANGUAGES[current_language]['edit_info'].format(len(self.current_recording))}")
        trim_start = input(LANGUAGES[current_language]["trim_start"]).lower() == 'y'
        trim_end = input(LANGUAGES[current_language]["trim_end"]).lower() == 'y'
        if trim_start:
            self.current_recording = self.current_recording[3:]
        if trim_end and len(self.current_recording) > 3:
            self.current_recording = self.current_recording[:-3]
        print(f"\n{LANGUAGES[current_language]['edit_result'].format(len(self.current_recording))}")

    def save_recording(self):
        global LAST_USED_FILE
        if not self.current_recording:
            print(f"\n{LANGUAGES[current_language]['no_data_save']}")
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
            print(f"\n{LANGUAGES[current_language]['action_saved'].format(action_number)}")
            filename = input(LANGUAGES[current_language]["input_filename_save"]) or "recorded_actions.json"
            LAST_USED_FILE = filename  # 更新上次使用的文件名
            self.save_to_file(filename)
        else:
            print(f"\n{LANGUAGES[current_language]['invalid_input']}")

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
                    # 创建新字典，避免在迭代时修改原字典
                    converted_data = {}
                    for key, value in data.items():
                        if not isinstance(key, str) or "positions" not in value:
                            raise ValueError("Invalid action data format")
                        converted_data[int(key)] = value  # 转换为整数键
                    self.record_list = converted_data
                print(f"\n{LANGUAGES[current_language]['load_success'].format(filename)}")
            except Exception as e:
                print(f"\n{LANGUAGES[current_language]['load_failed'].format(e)}")
        else:
            print(f"\n{LANGUAGES[current_language]['file_not_found'].format(filename)}")

    def ease_in_out_quad(self, t):
        if t < 0.5:
            return 2 * t * t
        return 1 - pow(-2 * t + 2, 2) / 2

    def interpolate_positions(self, start_pos, end_pos, steps):
        result = []
        for i in range(steps):
            t = i / (steps - 1)
            t = self.ease_in_out_quad(t)
            pos = start_pos + (end_pos - start_pos) * t
            result.append(np.round(pos).astype(np.int32))
        return result

    def play(self):
        print(f"\n{LANGUAGES[current_language]['start_play']}")
        action_number = input(LANGUAGES[current_language]["input_action"])
        if action_number.isdigit() and 1 <= int(action_number) <= 10:
            action_number = int(action_number)
            if action_number in self.record_list:
                speed_factor = float(input(LANGUAGES[current_language]["input_speed"]) or 1.0)
                speed_factor = max(0.5, min(2.0, speed_factor))
                positions_to_play = self.record_list[action_number]["positions"]
                try:
                    ACC = 30
                    self.bus.write("Acceleration", ACC)
                    BASE_SPEED = int(8000 * speed_factor)
                    self.bus.write("Goal_Speed", BASE_SPEED)
                    for i in range(len(positions_to_play)-1):
                        start_pos = np.array(positions_to_play[i], dtype=np.int32)
                        end_pos = np.array(positions_to_play[i+1], dtype=np.int32)
                        diff = end_pos - start_pos
                        max_diff = np.max(np.abs(diff))
                        if max_diff < 100:
                            steps = 3
                        elif max_diff < 300:
                            steps = 5
                        else:
                            steps = 7
                        interpolated = self.interpolate_positions(start_pos, end_pos, steps)
                        for pos in interpolated:
                            self.bus.write("Goal_Position", pos)
                            time.sleep(0.03 / speed_factor)
                except Exception as e:
                    print(f"\n{LANGUAGES[current_language]['play_error'].format(e)}")
            else:
                print(f"\n{LANGUAGES[current_language]['action_not_exist'].format(action_number)}")
        else:
            print(f"\n{LANGUAGES[current_language]['invalid_input']}")

    def print_menu(self):
        print(LANGUAGES[current_language]["menu"])

    def start(self):
        self.print_menu()
        import msvcrt
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                if key == "q":
                    print(f"\n{LANGUAGES[current_language]['exit']}")
                    break
                elif key == "r":
                    print(f"\n{LANGUAGES[current_language]['prepare_record']}")
                    config = load_config()
                    self.record(config["sample_interval"])
                elif key == "c":
                    print(f"\n{LANGUAGES[current_language]['prepare_stop']}")
                    self.stop_record()
                elif key == "p":
                    print(f"\n{LANGUAGES[current_language]['prepare_play']}")
                    self.play()
                elif key == "s":
                    print(f"\n{LANGUAGES[current_language]['prepare_save']}")
                    self.save_recording()
                elif key == "l":
                    print(f"\n{LANGUAGES[current_language]['prepare_load']}")
                    filename = input(LANGUAGES[current_language]["input_filename_load"]) or LAST_USED_FILE
                    self.load_from_local(filename)
                elif key == "f":
                    self.set_all_torque(0)
                    print(f"\n{LANGUAGES[current_language]['arm_relaxed']}")
                elif key == "t":
                    self.set_all_torque(1)
                    print(f"\n{LANGUAGES[current_language]['arm_locked']}")
                else:
                    print(f"\n{LANGUAGES[current_language]['invalid_key'].format(key)}")

def drag_teach():
    print(LANGUAGES[current_language]["start_teaching"])
    config = load_config()
    bus = initialize_motors(config)
    recorder = TeachingTest(bus)
    recorder.start()

if __name__ == '__main__':
    select_language()  # 启动时选择语言
    try:
        drag_teach()
    except KeyboardInterrupt:
        print(f"\n{LANGUAGES[current_language]['interrupt']}")
    except Exception as e:
        print(f"\n{LANGUAGES[current_language]['error'].format(e)}")
    finally:
        print(f"\n{LANGUAGES[current_language]['end']}")
        if 'bus' in globals():
            bus.disconnect()