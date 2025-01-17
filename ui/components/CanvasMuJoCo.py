import dearpygui.dearpygui as dpg
from ui.components.Canvas2D import Canvas2D

import threading
import glfw
import mujoco
import numpy as np
import mujoco_viewer


class CanvasMuJoCo(Canvas2D):

    def __init__(self, mj_model, mj_data, parent, size, camid=-1, format=dpg.mvFormat_Float_rgb, **kwargs):
        super().__init__(parent=parent, size=size, format=format, auto_mouse_transfrom=False, **kwargs)
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.frame = np.zeros((self.size[1], self.size[0], 4))
        # self.frame_depth = np.zeros((self.size[1], self.size[0], 1))
        self.frame_depth = None
        self.frame_tag = self.texture_register(self.size)
        self.frame_thread()
        self.camid = camid
        self.last_mouse_pos = (0, 0)
        self.now_mouse_pos = (0, 0)

        with self.draw():
            dpg.draw_image(self.frame_tag, (0, 0), self.size)
        self.handler_register()

    def frame_thread(self):
        t = threading.Thread(target=self.update_frame, daemon=True)
        t.start()

    def update_frame(self):
        try:
            self.viewer = mujoco_viewer.MujocoViewer(self.mj_model, self.mj_data, "offscreen", width=self.size[0], height=self.size[1])
            while True:
                glfw.make_context_current(self.viewer.window)
                self.frame, self.frame_depth = self.viewer.read_pixels(camid=self.camid, depth=True)
        except Exception as e:
            print(e)

    def handler_register(self):
        with dpg.handler_registry():
            dpg.add_mouse_wheel_handler(callback=self.mouse_wheel_event)
            dpg.add_mouse_move_handler(callback=self.mouse_move_event)

    def mouse_wheel_event(self, sender, app_data):
        mujoco.mjv_moveCamera(self.mj_model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, 0.01 * app_data, self.viewer.scn, self.viewer.cam)

    def mouse_move_event(self, sender, app_data):
        self.last_mouse_pos = self.now_mouse_pos
        self.now_mouse_pos = dpg.get_drawing_mouse_pos()
        dx = self.now_mouse_pos[0] - self.last_mouse_pos[0]
        dy = self.now_mouse_pos[1] - self.last_mouse_pos[1]
        info = dpg.get_item_info(sender)
        if dpg.is_mouse_button_down(dpg.mvMouseButton_Right):
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif dpg.is_mouse_button_down(dpg.mvMouseButton_Left):
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif dpg.is_mouse_button_down(dpg.mvMouseButton_Middle):
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        else:
            return
        mujoco.mjv_moveCamera(self.mj_model, action, dx / self.size[1], dy / self.size[1], self.viewer.scn, self.viewer.cam)

    def update(self):
        mujoco.mj_step(self.mj_model, self.mj_data)
        self.texture_update(self.frame_tag, self.frame)
