# pip install mujoco-python-viewer

import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.Canvas2D import Canvas2D
from ui.boxes.SimulatorModule.SimulatorParam import *
import mujoco
import mujoco_viewer
class SimulatorBox(BaseBox):
    only = True
    
    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (600, 400)
    def create(self):
        self.canvas = Canvas2D(self.tag)
        self.pic = self.canvas.texture_register(self.size)
        ctx = mujoco.GLContext(max_width=self.size[0], max_height=self.size[1])
        ctx.make_current()
        self.model = mujoco.MjModel.from_xml_string(free_body_MJCF)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, 'offscreen',width=self.size[0], height=self.size[1])
        mujoco.mj_forward(self.model, self.data)
        with self.canvas.draw():
            dpg.draw_image(self.pic, (0, 0), self.size)


    def destroy(self):
        super().destroy()

    def update(self):
        mujoco.mj_forward(self.model, self.data)
        # camid = 摄像机id
        img = self.viewer.read_pixels(camid=0)


        self.canvas.texture_update(self.pic, img)