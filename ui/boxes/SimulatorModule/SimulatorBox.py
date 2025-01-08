# pip install mujoco-python-viewer

import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
from ui.boxes.SimulatorModule.SimulatorParam import *
import mujoco
class SimulatorBox(BaseBox):
    only = True
    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (1200, 900)
        self.mj_model = mujoco.MjModel.from_xml_path('ui/boxes/SimulatorModule/test.xml')
        self.mj_data = mujoco.MjData(self.mj_model)
    def create(self):
        self.canvas = CanvasMuJoCo(parent=self.tag,size=self.size,mj_model=self.mj_model,mj_data=self.mj_data)
        
    def destroy(self):
        super().destroy()

    def update(self):
        self.canvas.update()
