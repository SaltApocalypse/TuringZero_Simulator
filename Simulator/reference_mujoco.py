import mujoco
import mujoco.viewer
import numpy as np
import threading
import time
import os
import sys
import traceback
import matplotlib

matplotlib.use("TkAgg")  # 在导入 pyplot 之前设置后端

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from typing import Callable, Optional, Tuple, Any
from collections import deque

# 将项目根目录添加到Python路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(project_root)

from logUtils import setup_logger

# 设置日志
logger_manager = setup_logger(log_dir="logs", log_file_prefix="lidar_plt", context_names=["main", "simulation", "sensor"], log_level="INFO")

main_logger = logger_manager.get_logger("main")
sim_logger = logger_manager.get_logger("simulation")
sensor_logger = logger_manager.get_logger("sensor")

# 运行频率设置
VIEWER_FREQ = 30  # Hz
SIMULATION_FREQ = 100  # Hz
SENSOR_FREQ = 50  # Hz

MODEL_XML = """
<mujoco model="laser_scanner">
    <asset>
        <!-- 添加棋盘格纹理 -->
        <texture type="2d" name="grid" builtin="checker" rgb1="0.1 0.2 0.3" rgb2="0.2 0.3 0.4" 
                width="512" height="512" mark="edge" markrgb="0.2 0.3 0.4"/>
        <!-- 创建材质 -->
        <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.2" texuniform="true"/>
    </asset>

    <option gravity="0 0 0"/>
    
    <worldbody>
        <!-- 四面墙 -->
        <geom name="wall1" type="box" size="0.5 0.01 0.2" pos="0 0.5 0.2"/>  
        <geom name="wall2" type="box" size="0.5 0.01 0.2" pos="0 -0.5 0.2"/>
        <geom name="wall3" type="box" size="0.01 0.5 0.2" pos="0.5 0 0.2"/>
        <geom name="wall4" type="box" size="0.01 0.5 0.2" pos="-0.5 0 0.2"/>
        
        <!-- 修改地板，添加材质 -->
        <geom name="floor" type="plane" size="2 2 .01" material="grid"/>
        
        <!-- 小球和激光雷达 -->
        <body name="sphere" pos="0 0 0.1">
            <freejoint name="ball_joint"/>
            <geom name="ball" type="sphere" size="0.025" rgba="1 0 0 1"/>
            
            <!-- 使用replicate创建36个rangefinder -->
            <replicate count="36" euler="0 0 10">
                <site name="rf" pos="0.02 0 0" zaxis="1 0 0"/>
            </replicate>
            
        </body>
    </worldbody>
    
    <actuator>
        <motor joint="ball_joint" gear="1 0 0 0 0 0"/>
        <motor joint="ball_joint" gear="0 1 0 0 0 0"/>
        <motor joint="ball_joint" gear="0 0 1 0 0 0"/>
        <motor joint="ball_joint" gear="0 0 0 1 0 0"/>
        <motor joint="ball_joint" gear="0 0 0 0 1 0"/>
        <motor joint="ball_joint" gear="0 0 0 0 0 1"/>
    </actuator>
    
    <sensor>
        <rangefinder site="rf"/>
    </sensor>
</mujoco>
"""


class LidarPlotter:
    """用于可视化激光雷达类型传感器数据的类"""

    def __init__(self, num_sensors: int = 36, angle_range: Tuple[float, float] = (0, 360), max_range: float = 1.0, update_interval: int = 30):
        """
        初始化雷达可视化器

        参数:
            num_sensors: 传感器数量
            angle_range: 角度范围(起始角度,结束角度)
            max_range: 最大测量范围
            update_interval: 更新间隔(毫秒)
        """
        self.num_sensors = num_sensors
        self.angle_range = angle_range
        self.max_range = max_range
        self.update_interval = update_interval
        self.fig = None
        self.ax = None
        self.line = None
        self.running = False
        self.close_callback = None

    def start(self, data_callback, close_callback=None, exit_event=None):
        self.close_callback = close_callback

        def update(frame):
            if exit_event and exit_event.is_set():
                plt.close(self.fig)
                return

            data = data_callback()
            angles = np.linspace(self.angle_range[0], self.angle_range[1], self.num_sensors)
            angles_rad = np.deg2rad(angles)

            x = data * np.cos(angles_rad)
            y = data * np.sin(angles_rad)

            self.line.set_data(x, y)
            return (self.line,)

        def on_close(event):
            self.running = False
            if self.close_callback:
                self.close_callback()

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(-self.max_range, self.max_range)
        self.ax.set_ylim(-self.max_range, self.max_range)
        self.ax.grid(True)
        (self.line,) = self.ax.plot([], [], "ro-", markersize=2)

        self.fig.canvas.mpl_connect("close_event", on_close)

        self.running = True
        ani = FuncAnimation(self.fig, update, interval=self.update_interval, blit=True, cache_frame_data=False)

        plt.show()


class SimulationThread(threading.Thread):
    def __init__(self, model, data, exit_event):
        super().__init__()
        self.model = model
        self.data = data
        self.running = threading.Event()
        self.exit_event = exit_event

    def run(self):
        sim_logger.info("仿真线程开始运行")
        self.running.set()

        while self.running.is_set() and not self.exit_event.is_set():
            try:
                # 计算圆周运动的位置
                t = time.time()
                radius = 0.3
                angular_speed = 1.0
                self.data.qpos[0] = radius * np.cos(t * angular_speed)
                self.data.qpos[1] = radius * np.sin(t * angular_speed)
                self.data.qpos[2] = 0.1

                # 步进仿真
                mujoco.mj_step(self.model, self.data)

                time.sleep(1 / SIMULATION_FREQ)

            except Exception as e:
                sim_logger.error(f"仿真线程错误: {str(e)}\n{traceback.format_exc()}")
                self.exit_event.set()
                break

        sim_logger.info("仿真线程结束运行")

    def stop(self):
        sim_logger.info("正在停止仿真线程...")
        self.running.clear()


class SensorThread(threading.Thread):
    def __init__(self, model, data, exit_event):
        super().__init__()
        self.model = model
        self.data = data
        self.running = threading.Event()
        self.exit_event = exit_event
        self.sensor_data = [1.0] * 36

    def run(self):
        sensor_logger.info("传感器线程开始运行")
        self.running.set()

        while self.running.is_set() and not self.exit_event.is_set():
            try:
                ranges = []
                for i in range(36):
                    range_value = self.data.sensordata[i]
                    ranges.append(1.0 if range_value < 0 else range_value)

                self.sensor_data = ranges
                time.sleep(1 / SENSOR_FREQ)

            except Exception as e:
                sensor_logger.error(f"传感器线程错误: {str(e)}\n{traceback.format_exc()}")
                self.exit_event.set()
                break

        sensor_logger.info("传感器线程结束运行")

    def get_sensor_data(self):
        return self.sensor_data.copy()

    def stop(self):
        sensor_logger.info("正在停止传感器线程...")
        self.running.clear()


def main():
    try:
        main_logger.info("正在初始化系统...")

        # 创建全局退出事件
        exit_event = threading.Event()

        # 创建模型和数据
        model = mujoco.MjModel.from_xml_string(MODEL_XML)
        data = mujoco.MjData(model)

        # 创建线程对象，传入exit_event
        sim_thread = SimulationThread(model, data, exit_event)
        sensor_thread = SensorThread(model, data, exit_event)

        # 创建可视化器
        plotter = LidarPlotter(num_sensors=36, angle_range=(0, 360), max_range=1.0, update_interval=30)

        def run_viewer():
            with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
                while viewer.is_running() and not exit_event.is_set():
                    viewer.sync()
                    time.sleep(1 / VIEWER_FREQ)
                # viewer关闭时触发退出
                exit_event.set()
                main_logger.info("Viewer已关闭，触发程序退出")

        def cleanup():
            main_logger.info("开始清理资源...")
            # 设置退出事件
            exit_event.set()
            # 停止所有线程
            sim_thread.stop()
            sensor_thread.stop()
            # 等待线程结束
            sim_thread.join()
            sensor_thread.join()
            if "viewer_thread" in locals():
                viewer_thread.join()
            main_logger.info("资源清理完成")

        # 启动线程
        sim_thread.start()
        sensor_thread.start()

        # 启动查看器
        viewer_thread = threading.Thread(target=run_viewer)
        viewer_thread.start()

        # 启动可视化，传入exit_event
        plotter.start(sensor_thread.get_sensor_data, close_callback=cleanup, exit_event=exit_event)

        # 等待退出事件
        while not exit_event.is_set():
            time.sleep(0.1)

        # 确保清理被执行
        if "cleanup" in locals():
            cleanup()

    except Exception as e:
        error_msg = f"程序运行出错: {str(e)}\n{traceback.format_exc()}"
        main_logger.error(error_msg)
        if "cleanup" in locals():
            cleanup()
        raise


if __name__ == "__main__":
    main()
