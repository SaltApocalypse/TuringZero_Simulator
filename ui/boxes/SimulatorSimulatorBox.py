import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox


class SimulatorSimulatorBox(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)

    def create(self):
        pass

    def destroy(self):
        super().destroy()

    def update(self):
        pass

    # ==================== feature ====================
