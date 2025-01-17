# pip install mujoco-python-viewer

import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
from ui.boxes.SimulatorModule.SimulatorParam import *
from ui.boxes.SimulatorModule.SimulatorUtils import get_info_actor
from ui.boxes.SimulatorModule.SimulatorUtils import get_info_jointstate
from ui.boxes.SimulatorModule.SimulatorUtils import get_info_imu
from ui.boxes.SimulatorModule.SimulatorUtils import PositionPID
from ui.boxes.SimulatorModule.SimulatorUtils import velocity_to_wheel
import ui.boxes.SimulatorModule.SimulatorTransform as st

from ui.components.TBKManager.TBKManager import tbk_manager
from utils.ClientLogManager import client_logger

import mujoco
import cv2
import numpy as np
import pylinalg as la
import pickle
import tbkpy._core as tbkpy

# proto include
from .proto.python import actor_info_pb2
from .proto.python import imu_info_pb2
from .proto.python import jointstate_info_pb2


class SimulatorBox(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)

        # mujoco
        # self.model = mujoco.MjModel.from_xml_path("static/models/turingzero_agv/tz_agv_with_plane.xml")
        self.model = mujoco.MjModel.from_xml_path("static/models/turingzero_agv/scene_terrain.xml")
        self.data = mujoco.MjData(self.model)

        # dpg
        self.size = (600, 400)

        # dpg params
        self.tf_euler = "0,0,0"
        self.tf_order = "YXZ"

        # cv2
        self.win_name = None

        # tbk
        tbk_manager.load_module(actor_info_pb2)
        tbk_manager.load_module(imu_info_pb2)
        tbk_manager.load_module(jointstate_info_pb2)

        self.__puber_status_imu = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_status_imu", msg_type=imu_info_pb2.IMUInfo)
        self.__puber_status_actor = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_status_actor", msg_type=actor_info_pb2.ActorInfo)
        self.__puber_status_jointstate = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_status_jointstate", msg_type=jointstate_info_pb2.JointStateInfo)

        self.__ep_info_pointcloud = tbkpy.EPInfo()
        self.__ep_info_pointcloud.name = "default"
        self.__ep_info_pointcloud.msg_name = "/cloud_registered"
        self.__puber_pointcloud = tbkpy.Publisher(self.__ep_info_pointcloud)

        # 接收控制命令并进行反馈
        self.__suber_command = tbk_manager.subscriber(name="tz_agv", msg_name="tz_agz_command", tag="command", callback_func=self.get_command_then_positionPID)

        # pid
        self.__wheels_pid = [PositionPID() for _ in range(3)]

    def create(self):
        with dpg.group(horizontal=True, parent=self.tag):
            self.euler_input = dpg.add_input_text(label="Euler", default_value="0,0,0", width=100, callback=self.__update_params)
            self.order = dpg.add_input_text(label="Order", default_value="YXZ", width=100, callback=self.__update_params)

        # 获取模型里的（前置）摄像头并渲染其画面
        self.now_camera = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self.canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.model, mj_data=self.data, camid=self.now_camera)
        # FIXME: 主窗口鼠标事件不触发

    def __update_params(self):
        self.tf_euler = dpg.get_value(self.euler_input)
        self.tf_order = dpg.get_value(self.order)

    def destroy(self):
        # FIXME: 在关闭当前窗口之后关闭cv2的窗口
        cv2.destroyWindow(self.win_name)  # NOTE: Or should we use `cv2.destroyAllWindows()` instead?
        super().destroy()

    # def rot_cam(self, rot_speed):
    #     euler = la.quat_to_euler(self.model.cam_quat[self.now_camera])
    #     euler = ((euler[0] + rot_speed) % (2 * np.pi), *euler[1:])
    #     self.model.cam_quat[self.now_camera] = la.quat_from_euler(euler, order="YXZ")

    # def rotate_camera_by_degrees(self, degrees):

    #     # Convert degrees to radians since the math operations are in radians
    #     radians = np.deg2rad(degrees)

    #     # Get the current orientation of the camera in Euler angles
    #     euler = la.quat_to_euler(self.model.cam_quat[self.now_camera])

    #     # Apply the rotation to the yaw (first element, euler[0]) and normalize it into [0, 2π]
    #     euler = ((euler[0] + radians) % (2 * np.pi), *euler[1:])

    #     # Update the camera's quaternion with the new euler angles
    #     self.model.cam_quat[self.now_camera] = la.quat_from_euler(euler, order="YXZ")

    def get_status_and_publish(self):
        """
        获取状态并返回。
        """
        # imu
        status = get_info_imu(self.model, self.data, "tz_agv", False)
        status_imu = imu_info_pb2.IMUInfo()
        status_imu.orientation.x, status_imu.orientation.y, status_imu.orientation.z, status_imu.orientation.w = status[0]
        status_imu.angular_velocity.x, status_imu.angular_velocity.y, status_imu.angular_velocity.z = status[1]
        status_imu.linear_acceleration.x, status_imu.linear_acceleration.y, status_imu.linear_acceleration.z = status[2]
        self.__puber_status_imu.publish(status_imu.SerializeToString())

        # actor
        status = get_info_actor(self.model, self.data)
        for state in status:
            status_actor = actor_info_pb2.ActorInfo()
            status_actor.actor_name, status_actor.joint_name, status_actor.torque = state
            self.__puber_status_actor.publish(status_actor.SerializeToString())

        # jointstate
        status = get_info_jointstate(self.model, self.data)
        for state in status:
            status_jointstate = jointstate_info_pb2.JointStateInfo()
            status_jointstate.joint_name = state[0]
            status_jointstate.position.x, status_jointstate.position.y, status_jointstate.position.z = state[1]
            status_jointstate.velocity.angular.wx, status_jointstate.velocity.angular.wy, status_jointstate.velocity.angular.wz = state[2][:3]
            status_jointstate.velocity.linear.vx, status_jointstate.velocity.linear.vy, status_jointstate.velocity.linear.vz = state[2][3:]
            # status_jointstate.effort = state[3]
            self.__puber_status_jointstate.publish(status_jointstate.SerializeToString())

    def get_command_then_positionPID(self, msg):
        """
        接受位置PID控制指令并执行
        NOTE: 仅针对AGV小车！

        @param
        - msg: 接受到的命令 (vx, vz, w)
        """
        wheels = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "front_wheel_rolling_joint"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_rolling_joint"),
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_rolling_joint"),
        ]
        print(wheels)

        vx, vz, w = pickle.loads(msg)

        current_velocity = []
        target_velocity = velocity_to_wheel(vx, vz, w)

        for wheel in wheels:
            current_velocity.append(self.data.qvel[wheel])
        for wheel_num in range(len(wheels)):
            self.data.ctrl[wheel_num] = self.__wheels_pid[wheel_num].update(current_velocity[wheel_num], target_velocity[wheel_num])
        # self.data.ctrl[0] = self.__wheels_pid[0].update(current_velocity[0], target_velocity[0])
        # self.data.ctrl[1] = self.__wheels_pid[1].update(current_velocity[1], target_velocity[1])
        # self.data.ctrl[2] = self.__wheels_pid[2].update(current_velocity[2], target_velocity[2])

    def update(self):
        # TODO: "模块化" (对，对吗？)
        # 雷达：深度图部分处理
        if self.canvas.frame_depth is None:
            return

        # 图像处理：获取深度图，线性化深度图
        non_linear_depth_buffer = self.canvas.frame_depth[:, :, 0]
        linear_depth_buffer = st.nonlinear_to_linear_depth(non_linear_depth_buffer, 0.1, 10)
        points = st.depth_to_point_cloud(
            linear_depth_buffer,
            self.model.cam_fovy[self.now_camera],
            self.model.cam_quat[self.now_camera],
            self.model.cam_pos[self.now_camera],
        )
        rot = la.mat_from_euler([0, np.pi / 2, 0], order="YXZ")[:3, :3]
        points = points @ rot

        linear_depth_normalized = cv2.normalize(linear_depth_buffer, None, 0, 255, cv2.NORM_MINMAX)
        linear_depth_normalized = linear_depth_normalized.astype(np.uint8)  # 转为 8 位图像

        self.win_name = "Linear Depth Map"
        cv2.imshow(self.win_name, linear_depth_normalized)
        cv2.waitKey(1)

        # 批量发布点云数据
        batch_size = 5000
        scale_factor = 1
        num_points = points.shape[0]
        points = np.array(points, dtype=np.float32)
        for i in range(0, num_points, batch_size):
            batch_points = points[i : i + batch_size]
            serialized_points = pickle.dumps(batch_points)
            self.__puber_pointcloud.publish(serialized_points)
        self.canvas.update()

        # 返回数据
        self.get_status_and_publish()
