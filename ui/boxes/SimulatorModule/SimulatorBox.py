import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.Canvas2D import Canvas2D
from ui.boxes.SimulatorModule.SimulatorParam import *
import mujoco
import threading
ctx = mujoco.GLContext(max_width=600, max_height=400)
ctx.make_current()
class SimulatorBox(BaseBox):
    only = True
    
    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (600, 400)
        self.lock = threading.Lock()

    def create(self):
        self.canvas = Canvas2D(self.tag)
        self.pic = self.canvas.texture_register(self.size)
        ctx = mujoco.GLContext(max_width=self.size[0], max_height=self.size[1])
        ctx.make_current()
        self.model = mujoco.MjModel.from_xml_string(free_body_MJCF)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, self.size[1], self.size[0])


        with self.canvas.draw():
            dpg.draw_image(self.pic, (0, 0), self.size)


    def destroy(self):
        super().destroy()

    def update(self):
        frame = self.renderer.render()
        print(frame)
        self.canvas.texture_update(self.pic, frame)