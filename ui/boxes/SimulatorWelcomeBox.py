import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox


class SimulatorWelcomeBox(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.btn_simulatorbox = None
        self.btn_urdfviewerbox = None

    def create(self):
        self.btn_simulatorbox = dpg.add_button(label="btn_simulatorbox", parent=self.tag, callback=lambda: self.ui.add_SimulatorSimulatorBox(ui=self.ui))
        self.btn_urdfviewerbox = dpg.add_button(label="btn_urdfviewerbox", parent=self.tag, callback=lambda: self.ui.add_SimulatorUrdfviewerBox(ui=self.ui))

    def destroy(self):
        super().destroy()

    def update(self):
        pass

    # ==================== feature ====================
