import math

import numpy as np


class IncrementalPID:

    # def __init__(self, Kp=1.0, Ki=0.1, Kd=0.01, velocity_threshold=0.1):
    def __init__(self, Kp=0.02, Ki=0, Kd=0, velocity_threshold=0.1):
        """
        增量式 PID 控制器类

        :param Kp: 比例系数
        :param Ki: 积分系数
        :param Kd: 微分系数
        :param target_velocity: 目标值，默认是0
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.velocity_threshold = velocity_threshold

        # 初始化PID的历史值
        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0

    def update(self, current_value, target_velocity):
        """
        根据当前值更新PID输出

        :param current_value: 当前测量值
        :param setpoint: 目标值，如果为 None 则使用初始化时的目标值
        :return: 当前输出
        """
        self.target_velocity = target_velocity

        # 计算误差
        error = self.target_velocity - current_value

        # 计算积分部分
        self.integral += error

        # 计算微分部分
        derivative = error - self.prev_error

        # 增量式PID控制器的输出
        delta_output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # 更新输出值
        self.output += delta_output

        # 更新误差历史值
        self.prev_error = error

        return self.output, 1 if abs(error) < self.velocity_threshold else 0

    def reset(self):
        """
        重置PID控制器状态
        """
        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0


class PositionPID:
    def __init__(self, Kp=0.1, Ki=1, Kd=0.0, velocity_threshold=0.1):
        """
        位置式 PID 控制器类

        :param Kp: 比例系数
        :param Ki: 积分系数
        :param Kd: 微分系数
        :param velocity_threshold: 速度阈值
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.velocity_threshold = velocity_threshold

        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0

    def update(self, current_value, target_velocity):
        """
        根据当前值更新PID输出

        :param current_value: 当前测量值
        :param target_velocity: 目标值
        :return: (输出值, 是否达到阈值)
        """
        # 计算误差
        error = target_velocity - current_value

        # 计算积分项
        self.integral += error

        # 计算微分项
        derivative = error - self.prev_error

        # 位置式PID计算
        self.output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # 更新误差
        self.prev_error = error

        return self.output, 1 if abs(error) < self.velocity_threshold else 0

    def reset(self):
        """
        重置PID控制器状态
        """
        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0

