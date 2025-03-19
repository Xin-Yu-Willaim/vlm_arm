"""
拖动示教程序 (STS3215版本)
作者: 同济子豪兄 (优化版)
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

def load_config(config_file="config.json"):
    try:
        with open(config_file, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"加载配置文件失败: {e}，使用默认设置")
        return {"port": "COM5", "baudrate": 1000000, "motor_ids": [1, 2, 3, 4, 5, 6], "sample_interval": 0.01}

def initialize_motors(config):
    motors = {}
    for i, motor_id in enumerate(config["motor_ids"]):
        motors[f"joint_{i+1}"] = (motor_id, "sts3215")
    bus_config = FeetechMotorsBusConfig(port=config["port"], motors=motors, mock=False)
    bus = FeetechMotorsBus(bus_config)
    try:
        bus.connect()
        print("舵机总线连接成功")
        return bus
    except Exception as e:
        print(f"舵机总线连接失败: {e}")
        raise

class TeachingTest:
    def __init__(self, bus):
        self.bus = bus
        self.recording = False
        self.record_list = {}  # 存储动作的字典: {编号: {positions, name, timestamp}}
        self.current_recording = []  # 当前录制的临时位置列表
        self.load_from_local()  # 初始化时加载已有动作

    def get_all_positions(self):
        """读取所有舵机的当前位置"""
        try:
            positions = self.bus.read("Present_Position")
            return positions
        except Exception as e:
            print(f"\nError reading positions: {e}")
            return None

    def set_all_torque(self, enable):
        """设置所有舵机的扭矩开关"""
        try:
            self.bus.write("Torque_Enable", enable)
            if not enable:
                print("\n警告：机械臂扭矩已关闭，可能下落，请确保安全！")
        except Exception as e:
            print(f"\nError setting torque: {e}")

    def record(self, sample_interval=0.01):
        """录制动作，采样间隔可调"""
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
                                print(f"\r Recording... Time: {current_time - start_time:.2f}s, Positions: {smoothed}", end="")
                        last_time = current_time
                    except Exception as e:
                        print(f"\nError during recording: {e}")
                        time.sleep(0.1)
                else:
                    time.sleep(0.001)
            print("\n录制结束，共记录 {} 个位置点".format(len(self.current_recording)))
        
        print("\n开始录制动作")
        self.record_t = threading.Thread(target=_record, daemon=True)
        self.record_t.start()

    def stop_record(self):
        """停止录制并提供编辑选项"""
        if self.recording:
            self.recording = False
            self.record_t.join(timeout=1.0)
            if self.record_t.is_alive():
                print("\n警告：录制线程未及时停止")
            else:
                print("\n停止录制动作")
            self.edit_recording()

    def edit_recording(self):
        """编辑当前录制的动作，允许剪辑首尾"""
        if not self.current_recording:
            print("\n当前没有录制数据可编辑")
            return
        print(f"\n当前录制包含 {len(self.current_recording)} 个位置点")
        trim_start = input("是否删除前3帧？(y/n): ").lower() == 'y'
        trim_end = input("是否删除后3帧？(y/n): ").lower() == 'y'
        if trim_start:
            self.current_recording = self.current_recording[3:]
        if trim_end and len(self.current_recording) > 3:
            self.current_recording = self.current_recording[:-3]
        print(f"编辑后剩余 {len(self.current_recording)} 个位置点")

    def save_recording(self):
        """保存录制的动作，包含元数据"""
        if not self.current_recording:
            print("\n没有可保存的动作数据")
            return
        action_number = input("请输入要保存的动作编号（1-10）：")
        if action_number.isdigit() and 1 <= int(action_number) <= 10:
            action_number = int(action_number)
            action_name = input("请输入动作名称（可选，直接回车跳过）：")
            self.record_list[action_number] = {
                "positions": self.current_recording.copy(),
                "name": action_name or f"Action_{action_number}",
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            print(f"\n动作 {action_number} 已保存。")
            filename = input("请输入保存文件名（默认 recorded_actions.json）：") or "recorded_actions.json"
            self.save_to_file(filename)
        else:
            print("无效的输入，请输入 1 到 10 之间的数字。")

    def save_to_file(self, filename="recorded_actions.json"):
        """将动作保存到指定文件"""
        try:
            with open(filename, "w") as f:
                json.dump(self.record_list, f, indent=4)
            print(f"动作数据已保存到 {filename}")
        except Exception as e:
            print(f"保存动作数据失败: {e}")

    def load_from_local(self, filename="recorded_actions.json"):
        """从指定文件加载动作，包含校验"""
        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    data = json.load(f)
                    for key, value in data.items():
                        if not isinstance(key, str) or "positions" not in value:
                            raise ValueError("动作数据格式错误")
                        data[int(key)] = value
                    self.record_list = data
                print(f"\n成功加载动作数据从 {filename}")
            except Exception as e:
                print(f"\n加载动作数据失败: {e}")
        else:
            print(f"没有找到文件 {filename}")

    def ease_in_out_quad(self, t):
        """二次缓动函数，用于平滑插值"""
        if t < 0.5:
            return 2 * t * t
        return 1 - pow(-2 * t + 2, 2) / 2

    def interpolate_positions(self, start_pos, end_pos, steps):
        """插值生成平滑位置序列"""
        result = []
        for i in range(steps):
            t = i / (steps - 1)
            t = self.ease_in_out_quad(t)
            pos = start_pos + (end_pos - start_pos) * t
            result.append(np.round(pos).astype(np.int32))
        return result

    def play(self):
        """回放动作，支持速度调节"""
        print("\n开始回放动作")
        action_number = input("请输入要回放的动作编号（1-10）：")
        if action_number.isdigit() and 1 <= int(action_number) <= 10:
            action_number = int(action_number)
            if action_number in self.record_list:
                speed_factor = float(input("请输入回放速度倍率（0.5-2.0，默认1.0）：") or 1.0)
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
                    print(f"\nError during playback: {e}")
            else:
                print(f"\n动作 {action_number} 不存在。")
        else:
            print("无效的输入，请输入 1 到 10 之间的数字。")

    def print_menu(self):
        print(
            """\
            \n拖动示教 同济子豪兄 (STS3215版本)
            \nq: 退出程序
            \nr: 开始录制动作
            \nc: 停止录制动作
            \np: 回放动作
            \ns: 将录制的动作保存到本地
            \nl: 从本地读取录制好的动作
            \nf: 放松机械臂 (关闭扭矩)
            \nt: 锁定机械臂 (开启扭矩)
            \n----------------------------------
            """
        )

    def start(self):
        self.print_menu()
        import msvcrt
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                if key == "q":
                    print("\n正在退出程序...")
                    break
                elif key == "r":
                    print("\n准备录制动作...")
                    config = load_config()
                    self.record(config["sample_interval"])
                elif key == "c":
                    print("\n准备停止录制...")
                    self.stop_record()
                elif key == "p":
                    print("\n准备回放动作...")
                    self.play()
                elif key == "s":
                    print("\n准备保存动作...")
                    self.save_recording()
                elif key == "l":
                    print("\n准备加载动作...")
                    filename = input("请输入加载文件名（默认 recorded_actions.json）：") or "recorded_actions.json"
                    self.load_from_local(filename)
                elif key == "f":
                    self.set_all_torque(0)
                    print("\n机械臂已放松")
                elif key == "t":
                    self.set_all_torque(1)
                    print("\n机械臂已锁定")
                else:
                    print(f"\n无效按键 '{key}'，请按菜单中的选项操作")

def drag_teach():
    print('开始拖动示教...')
    config = load_config()
    bus = initialize_motors(config)
    recorder = TeachingTest(bus)
    recorder.start()

if __name__ == '__main__':
    try:
        drag_teach()
    except KeyboardInterrupt:
        print('\n程序被用户中断')
    except Exception as e:
        print(f'\n程序出错: {e}')
    finally:
        print('\n程序结束')
        if 'bus' in globals():
            bus.disconnect()