import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.Canvas2D import Canvas2D
import numpy as np


class SimulatorBox(BaseBox):
    only = True
    
    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (500, 500)
    def create(self):
        self.canvas = Canvas2D(self.tag)
        self.pic = self.canvas.texture_register(self.size)
        with self.canvas.draw():
            dpg.draw_image(self.pic, [0, 0], self.size)

    def destroy(self):
        super().destroy()

    def update(self):
        data = np.random.randint(0, 256, (self.size[0], self.size[1], 4), dtype=np.uint8)
        self.canvas.texture_update(self.pic, data)