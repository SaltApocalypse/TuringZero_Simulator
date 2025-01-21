# pip install mujoco-python-viewer

import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
from ui.boxes.SimulatorModule.SimulatorParam import *
import ui.boxes.SimulatorModule.SimulatorUtils as SimulatorUtils
import mujoco
import cv2
import numpy as np
import pylinalg as la
import pickle
import ui.boxes.SimulatorModule.SimTransform as st


class SimulatorBox(BaseBox):
    only = False

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (600, 400)
        # self.mj_model = mujoco.MjModel.from_xml_path("static/models/turingzero_agv/tz_agv_with_plane.xml")
        self.mj_model = mujoco.MjModel.from_xml_path("static/models/turingzero_agv/scene_terrain.xml")

        self.mj_data = mujoco.MjData(self.mj_model)
        self.tf_euler = "0,0,0"
        self.tf_order = "YXZ"

    def create(self):
        with dpg.group(horizontal=True, parent=self.tag):
            self.euler_input = dpg.add_input_text(
                label="Euler", default_value="0,0,0", width=100, callback=self.update_params
            )
            self.order = dpg.add_input_text(label="Order", default_value="YXZ", width=100, callback=self.update_params)
        self.front_camera = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")


        self.now_camera = self.front_camera
        self.canvas = CanvasMuJoCo(
            parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data, camid=self.front_camera
        )
        self.pub_register()

    def update_params(self):
        self.tf_euler = dpg.get_value(self.euler_input)
        self.tf_order = dpg.get_value(self.order)

    def destroy(self):
        super().destroy()

    def rot_cam(self, rot_speed):
        euler = la.quat_to_euler(self.mj_model.cam_quat[self.now_camera])
        euler = ((euler[0] + rot_speed) % (2 * np.pi), *euler[1:])
        self.mj_model.cam_quat[self.now_camera] = la.quat_from_euler(euler, order="YXZ")

    def rotate_camera_by_degrees(self, degrees):

        # Convert degrees to radians since the math operations are in radians
        radians = np.deg2rad(degrees)

        # Get the current orientation of the camera in Euler angles
        euler = la.quat_to_euler(self.mj_model.cam_quat[self.now_camera])

        # Apply the rotation to the yaw (first element, euler[0]) and normalize it into [0, 2π]
        euler = ((euler[0] + radians) % (2 * np.pi), *euler[1:])

        # Update the camera's quaternion with the new euler angles
        self.mj_model.cam_quat[self.now_camera] = la.quat_from_euler(euler, order="YXZ")

    def pub_register(self):

        import tbkpy._core as tbkpy

        ep_info = tbkpy.EPInfo()
        ep_info.name = "default"
        ep_info.msg_name = "/cloud_registered"
        self.points_pub = tbkpy.Publisher(ep_info)

    def update(self):
        if self.canvas.frame_depth is None:
            return
        self.rotate_camera_by_degrees(30)
        # self.canvas.get_camera_img(self.now_camera)
        non_linear_depth_buffer = self.canvas.frame_depth[:, :, 0]
        linear_depth_buffer = st.nonlinear_to_linear_depth(non_linear_depth_buffer, 0.1, 10)
        points = st.depth_to_point_cloud(
            linear_depth_buffer,
            self.mj_model.cam_fovy[self.now_camera],
            self.mj_model.cam_quat[self.now_camera],
            self.mj_model.cam_pos[self.now_camera],
        )
        rot = la.mat_from_euler([0,np.pi / 2, 0], order="YXZ")[:3, :3]
        points = points @ rot

        linear_depth_normalized = cv2.normalize(linear_depth_buffer, None, 0, 255, cv2.NORM_MINMAX)
        linear_depth_normalized = linear_depth_normalized.astype(np.uint8)  # 转为 8 位图像

        cv2.imshow("Linear Depth Map", linear_depth_normalized)
        cv2.waitKey(1)

        batch_size = 5000
        scale_factor = 1
        num_points = points.shape[0]
        points = np.array(points, dtype=np.float32)
        for i in range(0, num_points, batch_size):
            batch_points = points[i : i + batch_size]
            serialized_points = pickle.dumps(batch_points)
            self.points_pub.publish(serialized_points)
        self.canvas.update()


