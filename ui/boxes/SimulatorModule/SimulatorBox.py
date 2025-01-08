# pip install mujoco-python-viewer

import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
from ui.boxes.SimulatorModule.SimulatorParam import *
import mujoco
import cv2
import numpy as np
class SimulatorBox(BaseBox):
    only = True
    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (600, 400)
        self.mj_model = mujoco.MjModel.from_xml_path('static/models/turingzero_agv/tz_agv_with_cameras.xml')
        self.mj_data = mujoco.MjData(self.mj_model)
    def create(self):
        self.canvas = CanvasMuJoCo(parent=self.tag,size=self.size,mj_model=self.mj_model,mj_data=self.mj_data)
        
    def destroy(self):
        super().destroy()

    def show_histogram(self,frame):
        # 创建一个空白图像用于显示直方图
        hist_image = np.zeros((300, 256, 3), dtype=np.uint8)

        # 分离 BGR 通道
        channels = cv2.split(frame)
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]  # 蓝、绿、红

        # 遍历每个通道并计算直方图
        for channel, color in zip(channels, colors):
            # 计算直方图
            hist = cv2.calcHist([channel], [0], None, [256], [0, 256])
            cv2.normalize(hist, hist, 0, hist_image.shape[0], cv2.NORM_MINMAX)  # 归一化

            # 绘制直方图
            for x in range(1, 256):
                cv2.line(hist_image, 
                        (x-1, 300 - int(hist[x-1])), 
                        (x, 300 - int(hist[x])), 
                        color, 1)

        return hist_image

    def update(self):
        if self.canvas.frame_depth is None:
            return
        frame = cv2.cvtColor((self.canvas.frame_depth * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        hist_image = self.show_histogram(frame)
        cv2.imshow('depth1',frame)

        cv2.imshow('depth',hist_image)
        cv2.waitKey(1)
        self.canvas.update()
