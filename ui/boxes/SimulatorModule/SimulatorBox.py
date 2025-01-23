# pip install mujoco-python-viewer

import dearpygui.dearpygui as dpg
from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
from ui.boxes.SimulatorModule.SimulatorUtils import get_info_actor
from ui.boxes.SimulatorModule.SimulatorUtils import get_info_jointstate
from ui.boxes.SimulatorModule.SimulatorUtils import get_info_imu
from ui.boxes.SimulatorModule.SimulatorUtils import PositionPID
from ui.boxes.SimulatorModule.SimulatorUtils import velocity_to_wheel
import ui.boxes.SimulatorModule.SimulatorTransform as st
from ui.components.TBKManager.TBKManager import tbk_manager
from utils.ClientLogManager import client_logger

import concurrent.futures
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
from .proto.python import image_pb2


class SimulatorBox(BaseBox):
    only = False

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)

        # mujoco
        self.mj_model = mujoco.MjModel.from_xml_path("static/models/turingzero_agv/scene_terrain.xml")
        self.mj_data = mujoco.MjData(self.mj_model)

        # dpg
        self.size = (600, 400)

        # tbk
        tbk_manager.load_module(actor_info_pb2)
        tbk_manager.load_module(imu_info_pb2)
        tbk_manager.load_module(jointstate_info_pb2)

        self.puber_status_imu = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_status_imu", msg_type=imu_info_pb2.IMUInfo)
        self.puber_status_actor = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_status_actor", msg_type=actor_info_pb2.ActorInfo)
        self.puber_status_jointstate = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_status_jointstate", msg_type=jointstate_info_pb2.JointStateInfo)
        self.puber_free_camera_image = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_free_camera_image", msg_type=image_pb2.Image)
        # self.puber_fix_camera_image = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_fix_camera_image", msg_type=image_pb2.Image)
        # self.puber_front_camera_image = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_front_camera_image", msg_type=image_pb2.Image)

        self.ep_info_pointcloud = tbkpy.EPInfo()
        self.ep_info_pointcloud.name = "default"
        self.ep_info_pointcloud.msg_name = "/cloud_registered"
        self.puber_pointcloud = tbkpy.Publisher(self.ep_info_pointcloud)

        # 接收控制命令并进行反馈
        self.suber_command = tbk_manager.subscriber(name="tz_agv", msg_name="tz_agz_command", tag="command", callback_func=self.get_command_then_positionPID)

        # pid
        self.wheels_pid = [PositionPID() for _ in range(3)]

        # tbk
        self.pub_register()

    def create(self):
        # create camera id
        self.free_camera_id = -1
        self.fix_camera_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "fix_camera")
        self.lidar_camera_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, "lidar_camera")

        # 用户操控视窗
        self.free_camera_canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data, camid=self.free_camera_id)

        # 摄像机主视角

        # 固定视角
        self.fix_camera_canvas = CanvasMuJoCo(parent=self.tag, size=(self.size[0] // 2, self.size[1] // 2), mj_model=self.mj_model, mj_data=self.mj_data, camid=self.fix_camera_id)

        # 激光雷达
        self.lidar_camera_canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data, camid=self.lidar_camera_id)
        dpg.hide_item(self.lidar_camera_canvas.group_tag)

        from ui.components.Canvas2D import Canvas2D

        self.cvs = Canvas2D(parent=self.tag, size=(600, 400), format=dpg.mvFormat_Float_rgb)
        self.texture_id = self.cvs.texture_register((400, 600))

        # ========== test ==========
        import ui.boxes.SimulatorModule.SimulatorParam as param

        param.param_init(self.mj_model, self.mj_data)

    def destroy(self):
        super().destroy()

    # ========== 通信 ==========
    def pub_register(self):
        """
        publisher 注册
        """
        import tbkpy._core as tbkpy

        ep_info = tbkpy.EPInfo()
        ep_info.name = "default"
        ep_info.msg_name = "/cloud_registered"
        self.points_pub = tbkpy.Publisher(ep_info)

    def get_status_and_publish(self):
        """
        获取状态并返回。
        """
        # imu
        status = get_info_imu(self.mj_model, self.mj_data, "tz_agv", False)
        status_imu = imu_info_pb2.IMUInfo()
        status_imu.orientation.x, status_imu.orientation.y, status_imu.orientation.z, status_imu.orientation.w = status[0]
        status_imu.angular_velocity.x, status_imu.angular_velocity.y, status_imu.angular_velocity.z = status[1]
        status_imu.linear_acceleration.x, status_imu.linear_acceleration.y, status_imu.linear_acceleration.z = status[2]
        self.puber_status_imu.publish(status_imu.SerializeToString())

        # actor
        status = get_info_actor(self.mj_model, self.mj_data)
        for state in status:
            status_actor = actor_info_pb2.ActorInfo()
            status_actor.actor_name, status_actor.joint_name, status_actor.torque = state
            self.puber_status_actor.publish(status_actor.SerializeToString())

        # jointstate
        status = get_info_jointstate(self.mj_model, self.mj_data)
        for state in status:
            status_jointstate = jointstate_info_pb2.JointStateInfo()
            status_jointstate.joint_name = state[0]
            status_jointstate.position.x, status_jointstate.position.y, status_jointstate.position.z = state[1]
            status_jointstate.velocity.angular.wx, status_jointstate.velocity.angular.wy, status_jointstate.velocity.angular.wz = state[2][:3]
            status_jointstate.velocity.linear.vx, status_jointstate.velocity.linear.vy, status_jointstate.velocity.linear.vz = state[2][3:]
            # status_jointstate.effort = state[3]
            self.puber_status_jointstate.publish(status_jointstate.SerializeToString())

    # ========== 控制 ==========
    def get_command_then_positionPID(self, msg):
        """
        接受位置PID控制指令并执行
        NOTE: 仅针对AGV小车！

        @param
        - msg: 接受到的命令 (vx, vz, w)
        """
        wheels = [
            mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "front_wheel_rolling_joint"),
            mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_rolling_joint"),
            mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_rolling_joint"),
        ]
        print(wheels)

        vx, vz, w = pickle.loads(msg)

        current_velocity = []
        target_velocity = velocity_to_wheel(vx, vz, w)

        for wheel in wheels:
            current_velocity.append(self.mj_data.qvel[wheel])
        for wheel_num in range(len(wheels)):
            self.mj_data.ctrl[wheel_num] = self.wheels_pid[wheel_num].update(current_velocity[wheel_num], target_velocity[wheel_num])
        # self.mj_data.ctrl[0] = self.__wheels_pid[0].update(current_velocity[0], target_velocity[0])
        # self.mj_data.ctrl[1] = self.__wheels_pid[1].update(current_velocity[1], target_velocity[1])
        # self.mj_data.ctrl[2] = self.__wheels_pid[2].update(current_velocity[2], target_velocity[2])

    # ========== lidar ==========
    def rot_cam(self, rot_speed):
        euler = la.quat_to_euler(self.mj_model.cam_quat[self.now_camera])
        euler = ((euler[0] + rot_speed) % (2 * np.pi), *euler[1:])
        self.mj_model.cam_quat[self.now_camera] = la.quat_from_euler(euler, order="YXZ")

    def rotate_camera_by_degrees(self, degrees):

        # Convert degrees to radians since the math operations are in radians
        radians = np.deg2rad(degrees)

        # Get the current orientation of the camera in Euler angles
        euler = la.quat_to_euler(self.mj_model.cam_quat[self.lidar_camera_id])

        # Apply the rotation to the yaw (first element, euler[0]) and normalize it into [0, 2π]
        euler = ((euler[0] + radians) % (2 * np.pi), *euler[1:])

        # Update the camera's quaternion with the new euler angles
        self.mj_model.cam_quat[self.lidar_camera_id] = la.quat_from_euler(euler, order="YXZ")

    def update(self):
        # mujoco 步进
        mujoco.mj_step(self.mj_model, self.mj_data)

        # 雷达：通过深度图获取雷达信息
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

        # linear_depth_normalized = cv2.normalize(linear_depth_buffer, None, 0, 255, cv2.NORM_MINMAX)
        # linear_depth_normalized = linear_depth_normalized.astype(np.uint8)  # 转为 8 位图像
        # cv2.imshow("Linear Depth Map", linear_depth_normalized)
        # cv2.waitKey(1)

        # 批量发布点云数据
        batch_size = 5000
        scale_factor = 1
        num_points = points.shape[0]
        points = np.array(points, dtype=np.float32)
        for i in range(0, num_points, batch_size):
            batch_points = points[i : i + batch_size]
            serialized_points = pickle.dumps(batch_points)
            self.puber_pointcloud.publish(serialized_points)

        # 更新 camera & 传输图像
        free_camera_img = self.free_camera_canvas.update()
        free_camera_image = image_pb2.Image()
        _, compressed_img = cv2.imencode(".jpg", free_camera_img, [cv2.IMWRITE_JPEG_QUALITY, 80])

        # img_array = np.frombuffer(compressed_img, dtype=np.uint8)
        # img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        # # 转换BGR to RGB
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # print(img.shape)

        # self.cvs.texture_update(self.texture_id, img)

        free_camera_image.img = compressed_img.tobytes()
        self.puber_free_camera_image.publish(free_camera_image.SerializeToString())

        # image = cv2.imdecode(compressed_img, cv2.IMREAD_COLOR)

        fix_camera_img = self.fix_camera_canvas.update()
        fix_camera_img = [type(fix_camera_img), len(fix_camera_img), fix_camera_img]

        lidar_camera_img = self.lidar_camera_canvas.update()
        lidar_camera_img = [type(lidar_camera_img), len(lidar_camera_img), lidar_camera_img]

        # 返回数据
        self.get_status_and_publish()
