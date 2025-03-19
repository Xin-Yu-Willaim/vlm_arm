import json
import numpy as np
import os
import time
import threading
from serial.tools import list_ports

# 导入 lerobot 中的舵机控制模块
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

# 根据实际连接的COM口修改
PORT = 'COM5'  # USB-Enhanced-SERIAL CH343
BAUDRATE = 1000000  # STS3215 默认波特率

# 舵机参数
MOTOR_IDS = list(range(1, 7))  # 6个舵机，ID从1到6

class TeachingTest:
    def __init__(self, bus):
        self.bus = bus
        self.recording = False
        self.record_list = {}  # 字典，用于存储多个动作
        self.current_recording = []  # 临时列表，用于当前录制
        self.load_from_local()  # 在初始化时加载之前保存的动作

    def get_all_positions(self):
        """获取所有舵机的当前位置"""
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
        except Exception as e:
            print(f"\nError setting torque: {e}")

    def record(self):
        """开始录制"""
        self.current_recording = []  # 初始化临时列表
        self.recording = True
        start_time = time.time()
        
        def _record():
            last_time = time.time()
            min_interval = 0.01  # 最小采样间隔：10ms (100Hz)
            positions_buffer = []  # 用于平滑的缓冲区
            
            while self.recording:
                current_time = time.time()
                if current_time - last_time >= min_interval:
                    try:
                        positions = self.get_all_positions()
                        if positions is not None:
                            # 添加到缓冲区
                            positions_buffer.append(positions)
                            
                            # 当缓冲区达到一定大小时进行平滑
                            if len(positions_buffer) >= 3:
                                # 简单的移动平均，并转换为整数
                                smoothed = np.round(np.mean(positions_buffer[-3:], axis=0)).astype(np.int32)
                                self.current_recording.append(smoothed.tolist())  # 存储为列表
                                print(f"\r Recording... Time: {current_time - start_time:.2f}s, Positions: {smoothed}", end="")
                        
                        last_time = current_time
                    except Exception as e:
                        print(f"\nError during recording: {e}")
                        time.sleep(0.1)  # 出错时等待一下
                else:
                    # 短暂休眠以减少CPU使用
                    time.sleep(0.001)

            print("\n录制结束，共记录 {} 个位置点".format(len(self.current_recording)))

        print("\n开始录制动作")
        self.record_t = threading.Thread(target=_record, daemon=True)
        self.record_t.start()

    def stop_record(self):
        """停止录制并提示保存"""
        if self.recording:
            self.recording = False
            self.record_t.join()
            print("\n停止录制动作")
            self.save_recording()

    def save_recording(self):
        """保存录制的动作"""
        if not self.current_recording:
            print("\n没有可保存的动作数据")
            return

        action_number = input("请输入要保存的动作编号（1-10）：")
        if action_number.isdigit() and 1 <= int(action_number) <= 10:
            action_number = int(action_number)
            self.record_list[action_number] = self.current_recording.copy()  # 保存到字典
            print(f"\n动作 {action_number} 已保存。")
            self.save_to_file()  # 保存到文件
        else:
            print("无效的输入，请输入 1 到 10 之间的数字。")

    def save_to_file(self):
        """将所有录制的动作保存到文件"""
        try:
            with open("recorded_actions.json", "w") as f:
                json.dump(self.record_list, f)  # 直接保存字典
            print("动作数据已保存到 recorded_actions.json")
        except Exception as e:
            print(f"保存动作数据失败: {e}")

    def load_from_local(self):
        """从文件加载动作"""
        if os.path.exists("recorded_actions.json"):
            try:
                with open("recorded_actions.json", "r") as f:
                    self.record_list = json.load(f)
                    print("\n成功加载动作数据")
            except Exception as e:
                print(f"\n加载动作数据失败: {e}")
        else:
            print("没有找到保存的动作数据文件。")

    def ease_in_out_quad(self, t):
        """二次缓动函数"""
        if t < 0.5:
            return 2 * t * t
        return 1 - pow(-2 * t + 2, 2) / 2

    def interpolate_positions(self, start_pos, end_pos, steps):
        """使用缓动函数插值位置"""
        result = []
        for i in range(steps):
            t = i / (steps - 1)
            # 应用缓动函数
            t = self.ease_in_out_quad(t)
            # 线性插值
            pos = start_pos + (end_pos - start_pos) * t
            result.append(np.round(pos).astype(np.int32))
        return result

    def play(self):
        """回放动作"""
        print("\n开始回放动作")
        
        action_number = input("请输入要回放的动作编号（1-10）：")
        if action_number.isdigit() and 1 <= int(action_number) <= 10:
            action_number = int(action_number)
            if action_number in self.record_list:
                positions_to_play = self.record_list[action_number]
                try:
                    # 设置适中的加速度
                    ACC = 30
                    self.bus.write("Acceleration", ACC)
                    
                    # 设置基础速度
                    BASE_SPEED = 8000
                    self.bus.write("Goal_Speed", BASE_SPEED)
                    
                    # 遍历所有位置点
                    for i in range(len(positions_to_play)-1):
                        start_pos = np.array(positions_to_play[i], dtype=np.int32)
                        end_pos = np.array(positions_to_play[i+1], dtype=np.int32)
                        
                        # 计算位置差异
                        diff = end_pos - start_pos
                        max_diff = np.max(np.abs(diff))
                        
                        # 根据位置差异决定插值点数量
                        if max_diff < 100:
                            steps = 3
                        elif max_diff < 300:
                            steps = 5
                        else:
                            steps = 7
                        
                        # 生成插值点
                        interpolated = self.interpolate_positions(start_pos, end_pos, steps)
                        
                        # 执行插值后的动作
                        for pos in interpolated:
                            try:
                                self.bus.write("Goal_Position", pos)
                                time.sleep(0.03)  # 等待时间
                            except Exception as e:
                                print(f"\nError during position write: {e}")
                                continue
                except Exception as e:
                    print(f"\nError during playback: {e}")
            else:
                print(f"\n动作 {action_number} 不存在。")
        else:
            print("无效的输入，请输入 1 到 10 之间的数字。")

    def print_menu(self):
        print(
            """\
        \r 拖动示教 同济子豪兄 (STS3215版本)
        \r q: 退出
        \r r: 开始录制动作
        \r c: 停止录制动作
        \r p: 回放动作
        \r s: 将录制的动作保存到本地
        \r l: 从本地读取录制好的动作
        \r f: 放松机械臂
        \r t: 锁定机械臂
        \r----------------------------------
            """
        )

    def start(self):
        self.print_menu()
        
        import msvcrt
        
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                if key == "q":
                    break
                elif key == "r":
                    self.record()
                elif key == "c":
                    self.stop_record()
                elif key == "p":
                    self.play()
                elif key == "s":
                    self.save_recording()
                elif key == "l":
                    self.load_from_local()
                elif key == "f":
                    self.set_all_torque(0)  # 关闭扭矩
                    print("\n机械臂已放松")
                elif key == "t":
                    self.set_all_torque(1)  # 开启扭矩
                    print("\n机械臂已锁定")
                else:
                    print(key)

def drag_teach():
    print('开始拖动示教...')
    # 初始化舵机总线
    motors = {}
    for i, motor_id in enumerate(MOTOR_IDS):
        motors[f"joint_{i+1}"] = (motor_id, "sts3215")
    
    config = FeetechMotorsBusConfig(
        port=PORT,
        motors=motors,
        mock=False
    )
    
    bus = FeetechMotorsBus(config)
    bus.connect()
    
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