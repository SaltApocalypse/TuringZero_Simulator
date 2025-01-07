import mujoco
import mujoco.viewer
import time
import threading

# ========== val ==========
model = mujoco.MjModel.from_xml_path("./turingzero_agv/tz_agv.xml")
data = mujoco.MjData(model)

# 全局退出事件
exit_event = threading.Event()

# 频率设置
VIEWER_FREQ = 60


# ========== viewer ==========
def viewer_run():
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and not exit_event.is_set():
            viewer.sync()
            time.sleep(1 / VIEWER_FREQ)
        exit_event.set()
