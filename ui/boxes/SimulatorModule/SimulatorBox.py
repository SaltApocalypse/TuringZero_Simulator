# pip install mujoco-python-viewer

import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
from ui.boxes.SimulatorModule.SimulatorParam import *
import ui.boxes.SimulatorModule.SimulatorUtils as SimulatorUtils
import mujoco
import threading


class SimulatorBox(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (600, 450)
        self.mj_model = mujoco.MjModel.from_xml_path("static/models/turingzero_agv/scene_terrain.xml")
        self.mj_data = mujoco.MjData(self.mj_model)
        self.lock = threading.Lock

    def create(self):
        self.canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data)
        print(SimulatorUtils.get_info_jointstate(self.mj_model, self.mj_data))

    def destroy(self):
        super().destroy()

    def update(self):
        self.canvas.update()
        # imu = SimulatorUtils.get_info_imu(self.mj_model, self.mj_data)
        # print(imu)
