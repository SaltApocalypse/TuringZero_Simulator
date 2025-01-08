import time
import threading
import mujoco
import mujoco.viewer

import SimulatorParam

# ========== val ==========
model = SimulatorParam.model
data = SimulatorParam.data

# 全局退出事件
exit_event = threading.Event()


# ========== viewer ==========
def viewer_run():
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and not exit_event.is_set():
            viewer.sync()
            time.sleep(1 / SimulatorParam.VIEWER_FREQ)
        exit_event.set()


def viewer_run():
    ctx = mujoco.GLContext(max_width=1280, max_height=720)
    ctx.make_current()

    # 获取渲染
    renderer = mujoco.Renderer(model)
    renderer.update_scene(data)
    rgb_arr = renderer.render()  # 返回numpy数组


# ========== Simulation ==========
class SimulationThread(threading.Thread):
    def __init__(self, model, data, exit_event):
        super().__init__()
        self.model = model
        self.data = data
        self.run_event = threading.Event()
        self.exit_event = exit_event
        # Other Info

    def run(self):
        self.run_event.set()

        while self.run_event.is_set() and not self.exit_event.is_set():
            # TODO: 在这里接受来自TBK的命令
            mujoco.mj_step(self.model, self.data)
            time.sleep(1 / SimulatorParam.SIMULATION_FREQ)

    def stop(self):
        self.run_event.clear()


# ========== Send ==========
class SensorThread(threading.Thread):
    def __init__(self, model, data, exit_event):
        super().__init__()
        self.model = model
        self.data = data
        self.run_event = threading.Thread()
        self.exit_event = exit_event
        # Other Info

    def run(self):
        self.run_event.set()

        while self.run_event.is_set() and not self.exit_event.is_set():
            # TODO: 发送数据
            time.sleep(1 / SimulatorParam.SENSOR_FREQ)

    def stop(self):
        self.run_event.clear()
