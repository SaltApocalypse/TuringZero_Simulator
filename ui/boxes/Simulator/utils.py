import math
import numpy as np


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
