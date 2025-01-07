import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.gfx_engine.Canvas2D import Canvas2D


import numpy as np


class SimulatorUrdfviewerBox(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)

    def create(self):
        self.canvas = Canvas2D(self.tag)
        rgba_noise = np.concatenate([np.random.normal(0, 1, (100, 100, 3)), np.ones((100, 100, 1))], axis=2)
        with dpg.texture_registry():
            self.pic = dpg.add_raw_texture(width=100, height=100, default_value=rgba_noise, format=dpg.mvFormat_Float_rgba, tag="texture_tag")

        with self.canvas.draw():
            dpg.draw_image(self.pic, [0, 0], [500, 500])
            # dpg.draw_arrow([1, 1], [200, 450])

    def destroy(self):
        super().destroy()

    def update(self):
        rgba_noise = np.concatenate([np.random.normal(0, 1, (100, 100, 3)), np.ones((100, 100, 1))], axis=2)
        dpg.set_value(self.pic, rgba_noise)
        pass

    # ==================== feature ====================
