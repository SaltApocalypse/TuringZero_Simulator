# pip install mujoco-python-viewer

import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
from ui.boxes.SimulatorModule.SimulatorParam import *
import mujoco
import threading


class SimulatorBox(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (600, 450)
        self.mj_model = mujoco.MjModel.from_xml_path("static/models/turingzero_agv/tz_agv_with_cameras.xml")
        # self.mj_model = mujoco.MjModel.from_xml_path("ui/boxes/SimulatorModule/test.xml")
        self.mj_data = mujoco.MjData(self.mj_model)
        self.lock = threading.Lock

    def create(self):
        # self.canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data)
        # with self.lock:
        self.depth_camera_canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data)

        camera_id = [
            mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera"),
            mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "left_camera"),
            mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "right_camera"),
        ]

        print(self.mj_model.cam_pos[camera_id[1]])

    def destroy(self):
        super().destroy()

    def update(self):
        # self.canvas.update()
        self.depth_camera_canvas.update()
