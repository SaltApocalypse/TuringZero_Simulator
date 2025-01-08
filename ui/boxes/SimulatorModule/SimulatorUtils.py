import math
import numpy as np
import cv2
import glfw
import mujoco


class PositionPID:

    def __init__(self, Kp: int = 0.1, Ki: int = 1, Kd: int = 0):
        """
        位置式 PID 控制器类

        @param
        - Kp: 比例系数
        - Ki: 积分系数
        - Kd: 微分系数

        @return
        - 当前的控制力度
        """

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0

    def update(self, current_velocity, target_velocity):
        # 计算误差
        error = target_velocity - current_velocity

        # 计算积分项
        self.integral += error

        # 计算微分项
        derivative = error - self.prev_error

        # 位置式PID计算
        self.output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # 更新误差
        self.prev_error = error

        return self.output

    def reset(self):
        """
        重置数据
        """
        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0


def velocity_to_wheel(vx: int, vy: int, w: int):
    """
    将整车运动需求转换为各电机速度要求

    @param
    - vx: x 方向的速度 (m/s)
    - vy: y 方向的速度 (m/s)
    - w: w 角度 (rad/s)

    @return
    - np.ndarray: 三电机速度 ([m/s, m/s, m/s])
    """
    LENGTH = 510  # mm
    THETA1 = math.pi / 3
    THETA2 = math.pi / 6

    V = np.array(
        [
            [math.cos(w), math.sin(w), LENGTH],
            [-math.cos(THETA1) * math.cos(w) + math.sin(THETA1) * math.sin(w), -math.cos(THETA1) * math.sin(w) - math.sin(THETA1) * math.cos(w), LENGTH],
            [-math.sin(THETA2) * math.cos(w) - math.cos(THETA2) * math.sin(w), -math.sin(THETA2) * math.sin(w) + math.cos(THETA2) * math.cos(w), LENGTH],
        ]
    )
    result = V @ np.array([vx, vy, w]).transpose()

    return result


# ========== RGBD_Camera ==========
class RGBDCamera:
    def __init__(self):
        self._color_buffer = None
        self._depth_buffer = None
        self._color_image = None
        self._depth_image = None
        # OpenGl render rage
        self._entent = None
        self._z_near = None
        self._z_far = None
        # Camera
        self._f = None  # focal length
        self._cx = None  # principal points
        self._cy = None

    def _linearize_depth(self, depth):
        """
        将 OpenGL 的非线性深度缓冲区转换为线性深度图，以米为单位

        @param
        - depth: OpenGL depth buffer (nonlinearized)

        @return
        - depth image in meters
        """
        depth_img = np.zeros_like(depth, dtype=np.float32)

        if self._z_near is None or self._z_far is None or self._extent is None:
            return -1

        depth_img = self._z_near * self._z_far * self._entent / (self._z_far - depth * (self._z_far - self._z_near))
        return depth_img

    # def set_camera_intrinsics(model, camera, viewport):
    #     fovy = model.cam_fovy[camera.fixedcamid] / 180 * math.pi / 2

    #     # focal length, fx = fy
    #     f = viewport.height / 2 / math.tan(fovy)
    #     # principal points
    #     cx = viewport.width / 2
    #     cy = viewport.height / 2

    #     return f, cx, cy


def init_rgbd_camera(model, camera_name):
    rgbd_camera = mujoco.mjvCamra(type=mujoco.mjCAMERA_FIXED, fixedcamid=mujoco.mj_name2id(model, mujoco.mjOBJ_CAMERA, camera_name))

    sensor_option: mujoco.mjvOption = None
    sensor_perturb: mujoco.mjvPerturb = None
    sensor_scene: mujoco.mjvScene = None
    sensor_context: mujoco.mjrContext = None

    mujoco.mjv_defaultOption(sensor_option)
    mujoco.mjv_defaultScene(sensor_scene)
    mujoco.mjr_defaultContext(sensor_context)

    mujoco.mjv_makeScene(model, sensor_scene, 1000)
    mujoco.mjr_makeContext(model, sensor_context, mujoco.mjFONTSCALE_150)

    mj_RGBD = RGBDCamera()

    # ===========================
    viewport = mujoco.mjrRect(0, 0, 0, 0)
    mj_RGBD.set_camera_intrinsics(model, mj_RGBD, viewport)
    mujoco.mjv_updateScene()
    # TODO:
