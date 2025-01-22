# pip install mujoco-python-viewer
import time
import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
import ui.boxes.SimulatorModule.SimulatorUtils as SimulatorUtils
import mujoco
import cv2
import numpy as np
import pylinalg as la
import pickle
import ui.boxes.SimulatorModule.SimulatorTransform as st
import concurrent.futures


class SimulatorBox(BaseBox):
    only = False

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)

        # DPG
        self.size = (800, 600)

        # import mujoco model
        self.mj_model = mujoco.MjModel.from_xml_path("static/models/turingzero_agv/scene_terrain.xml")
        self.mj_data = mujoco.MjData(self.mj_model)

        # tbk
        self.pub_register()

        # threadpool
        self.threadpool = concurrent.futures.ThreadPoolExecutor(max_workers=4)

        # other setting
        # self.tf_euler = "0,0,0"
        # self.tf_order = "YXZ"

    def create(self):
        # create camera id
        self.free_camera_id = -1
        self.fix_camera_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "fix_camera")
        self.lidar_camera_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_camera")

        # 用户操控视窗
        # def create_free_camera_canvas():
        self.free_camera_canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data, camid=self.free_camera_id)

        # future = self.threadpool.submit(create_free_camera_canvas)
        # future.result()
        # time.sleep(0.1)

        # 摄像机主视角

        # 固定视角
        # def create_fix_camera_canvas():
        self.fix_camera_canvas = CanvasMuJoCo(parent=self.tag, size=(self.size[0] // 2, self.size[1] // 2), mj_model=self.mj_model, mj_data=self.mj_data, camid=self.fix_camera_id)

        # time.sleep(0.1)
        # future = self.threadpool.submit(create_fix_camera_canvas)
        # future.result()

        # 激光雷达
        # def create_lidar_camera_canvas():
        self.lidar_camera_canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data, camid=self.lidar_camera_id)
        dpg.hide_item(self.lidar_camera_canvas.group_tag)

        # future = self.threadpool.submit(create_lidar_camera_canvas)
        # future.result()

        # with dpg.group(horizontal=True, parent=self.tag):
        # self.euler_input = dpg.add_input_text(label="Euler", default_value="0,0,0", width=100, callback=self.update_params)
        # self.order = dpg.add_input_text(label="Order", default_value="YXZ", width=100, callback=self.update_params)

    def destroy(self):
        self.threadpool.shutdown()
        super().destroy()

    # def update_params(self):
    #     self.tf_euler = dpg.get_value(self.euler_input)
    #     self.tf_order = dpg.get_value(self.order)

    def rot_cam(self, rot_speed):
        euler = la.quat_to_euler(self.mj_model.cam_quat[self.lidar_camera_id])
        euler = ((euler[0] + rot_speed) % (2 * np.pi), *euler[1:])
        self.mj_model.cam_quat[self.lidar_camera_id] = la.quat_from_euler(euler, order="YXZ")

    def rotate_camera_by_degrees(self, degrees):

        # Convert degrees to radians since the math operations are in radians
        radians = np.deg2rad(degrees)

        # Get the current orientation of the camera in Euler angles
        euler = la.quat_to_euler(self.mj_model.cam_quat[self.lidar_camera_id])

        # Apply the rotation to the yaw (first element, euler[0]) and normalize it into [0, 2π]
        euler = ((euler[0] + radians) % (2 * np.pi), *euler[1:])

        # Update the camera's quaternion with the new euler angles
        self.mj_model.cam_quat[self.lidar_camera_id] = la.quat_from_euler(euler, order="YXZ")

    def pub_register(self):
        import tbkpy._core as tbkpy

        ep_info = tbkpy.EPInfo()
        ep_info.name = "default"
        ep_info.msg_name = "/cloud_registered"
        self.points_pub = tbkpy.Publisher(ep_info)

    def update(self):
        mujoco.mj_step(self.mj_model, self.mj_data)

        if self.lidar_camera_canvas.frame_depth is None:
            return
        self.rotate_camera_by_degrees(30)

        non_linear_depth_buffer = self.lidar_camera_canvas.frame_depth[:, :, 0]
        linear_depth_buffer = st.nonlinear_to_linear_depth(non_linear_depth_buffer, 0.1, 10)
        points = st.depth_to_point_cloud(
            linear_depth_buffer,
            self.mj_model.cam_fovy[self.lidar_camera_id],
            self.mj_model.cam_quat[self.lidar_camera_id],
            self.mj_model.cam_pos[self.lidar_camera_id],
        )
        rot = la.mat_from_euler([0, np.pi / 2, 0], order="YXZ")[:3, :3]
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

        self.lidar_camera_canvas.update()
        self.free_camera_canvas.update()
        self.fix_camera_canvas.update()
