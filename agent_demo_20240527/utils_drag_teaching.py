# utils_drag_teaching.py
# 同济子豪兄 2024-5-23
# 拖动示教 - 适用于 STS3215 舵机

print('导入拖动示教模块')

import time
import os
import sys
import threading
import json
import serial
from serial.tools import list_ports
import numpy as np

# 导入 lerobot 中的舵机控制模块
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig

def list_serial_ports():
    """列出所有可用的串口"""
    ports = list_ports.comports()
    print("\n可用的串口:")
    for port in ports:
        print(f"- {port.device}: {port.description}")
    return ports

# 列出所有串口
available_ports = list_serial_ports()

# 根据实际连接的COM口修改
PORT = 'COM5'  # USB-Enhanced-SERIAL CH343
BAUDRATE = 1000000  # STS3215 默认波特率

# 舵机参数
MOTOR_IDS = list(range(1, 7))  # 6个舵机，ID从1到6

# 检查指定的端口是否存在
if not any(PORT == port.device for port in available_ports):
    print(f"\n警告: {PORT} 未找到！请从上面的列表中选择正确的端口。")
    sys.exit(1)

print(f"\n尝试连接到 {PORT}...")

try:
    # 创建舵机配置
    motors = {}
    for i, motor_id in enumerate(MOTOR_IDS):
        # 使用 "sts3215" 作为舵机型号
        motors[f"joint_{i+1}"] = (motor_id, "sts3215")
    
    config = FeetechMotorsBusConfig(
        port=PORT,
        motors=motors,
        mock=False
    )
    
    # 初始化舵机总线
    bus = FeetechMotorsBus(config)
    bus.connect()
    print("舵机初始化成功")

except Exception as e:
    print(f"错误: {e}")
    sys.exit(1)

class TeachingTest:
    def __init__(self, bus):
        self.bus = bus
        self.recording = False
        self.playing = False
        self.record_list = []
        self.record_t = None
        self.play_t = None

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
        self.record_list = []
        self.recording = True
        start_time = time.time()
        
        def _record():
            last_time = time.time()
            min_interval = 0.01  # 最小采样间隔：20ms (50Hz)
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
                                self.record_list.append(smoothed)
                                print(f"\r Recording... Time: {current_time - start_time:.2f}s, Positions: {smoothed}", end="")
                                
                                # 如果变化太大，增加中间点
                                if len(self.record_list) >= 2:
                                    prev = self.record_list[-2]
                                    curr = smoothed
                                    max_diff = np.max(np.abs(curr - prev))
                                    if max_diff > 66:  # 如果相邻位置差异太大
                                        # 插入中间点（确保是整数）
                                        mid = np.round((prev + curr) / 2).astype(np.int32)
                                        self.record_list.insert(-1, mid)
                        
                        last_time = current_time
                    except Exception as e:
                        print(f"\nError during recording: {e}")
                        time.sleep(0.1)  # 出错时等待一下
                else:
                    # 短暂休眠以减少CPU使用
                    time.sleep(0.001)

            print("\n录制结束，共记录 {} 个位置点".format(len(self.record_list)))

        print("\n开始录制动作")
        self.record_t = threading.Thread(target=_record, daemon=True)
        self.record_t.start()

    def stop_record(self):
        """停止录制"""
        if self.recording:
            self.recording = False
            self.record_t.join()
            print("\n停止录制动作")

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
        
        try:
            # 设置适中的加速度
            ACC = 30
            self.bus.write("Acceleration", ACC)
            
            # 设置基础速度
            BASE_SPEED = 8000
            self.bus.write("Goal_Speed", BASE_SPEED)
            
            # 遍历所有位置点
            for i in range(len(self.record_list)-1):
                start_pos = np.array(self.record_list[i], dtype=np.int32)
                end_pos = np.array(self.record_list[i+1], dtype=np.int32)
                
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
                        
                        # 根据位置差异动态调整等待时间
                        if max_diff < 100:
                            time.sleep(0.03)
                        elif max_diff < 300:
                            time.sleep(0.04)
                        else:
                            time.sleep(0.05)
                            
                    except Exception as e:
                        print(f"\nError during position write: {e}")
                        continue
                
        except Exception as e:
            print(f"\nError during playback: {e}")
            
        print("\n回放结束")

    def save_to_local(self):
        """保存动作到文件"""
        if not self.record_list:
            print("\n没有可保存的动作数据")
            return

        # 将 numpy 数组转换为列表后再保存
        save_data = [pos.tolist() if isinstance(pos, np.ndarray) else pos 
                    for pos in self.record_list]
        
        save_path = os.path.dirname(__file__) + "/temp/record.txt"
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "w") as f:
            json.dump(save_data, f, indent=2)
            print(f"\n动作数据已保存到: {save_path}")

    def load_from_local(self):
        """从文件加载动作"""
        try:
            with open(os.path.dirname(__file__) + "/temp/record.txt", "r") as f:
                self.record_list = json.load(f)
                print("\n成功加载动作数据")
        except Exception as e:
            print(f"\n加载动作数据失败: {e}")

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
                    self.save_to_local()
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
