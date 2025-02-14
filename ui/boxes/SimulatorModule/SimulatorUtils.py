import math
import numpy as np
import mujoco
import xml.etree.ElementTree as xml_et


class PositionPID:

    def __init__(self, Kp: int = 0.1, Ki: int = 1, Kd: int = 0) :
        """
        位置式 PID 控制器类

        Args:
            Kp (int): 比例系数
            Ki (int): 积分系数
            Kd (int): 微分系数
        """

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0.0
        self.integral = 0.0
        self.output = 0.0

    def update(self, current_velocity:float, target_velocity:float)-> float:
        """
        更新函数

        Args:
            current_velocity (float): 当前速度
            target_velocity (float): 目标速度


        Returns:
            float: 当前的控制力度
        """
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


def velocity_to_wheel(vx: float, vz: float, w: float) -> np.ndarray:
    """
    将整车运动需求转换为各电机速度要求
    NOTE: 仅针对AGV小车！

    Args:
        vx (float): x 方向的速度 (m/s)
        vz (float): z 方向的速度 (m/s)
        w (float): w 角度 (rad/s)

    Returns:
        np.ndarray: 三电机速度 ([m/s, m/s, m/s])
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
    result = V @ np.array([vx, vz, w]).transpose()

    return result


# ========== 获取信息  ========== #
def get_info_imu(model: mujoco.MjModel, data: mujoco.MjData, name: str, inOneList: bool = True):
    #FIXME: 没弄懂 Returns 如何做多行的标注  (Ln 109-112, Ln 139-140, Ln 183-184)
    """
    获取IMU信息并返回：
    - 姿态：没有专门的姿态测量仪，取自`mujoco.MjData`的全局物体姿态
    - 角速度：取自角速度计
    - 线性加速器：取自加速度计

    Args:
        model (mujoco.MjModel): mjc 的模型
        data (mujoco.MjData): mjc 的数据
        name (str): 需要查看的物体
        inOneList: 是否把数据在同一个数组里，默认`true`；否则分为三个变量

    Returns:
        list[10]: [0:4]: orientation | [4:7]: angular_velocity | [7:10]: linear_acceleration
        list[3]: [0]: orientation[4] | [1]: angular_velocity[3] | [2]: linear_acceleration[3]
            
    """

    acc = data.sensordata[0:3]
    gyro = data.sensordata[3:6]
    # vel = data.sensordata[6:]

    # NOTE: 考虑默认参数为主体的名称并用变量填充
    id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

    if inOneList:
        return [data.xquat[id][0], data.xquat[id][1], data.xquat[id][2], data.xquat[id][3], gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]]
    return data.xquat[id], gyro, acc


def get_info_actor(model: mujoco.MjModel, data: mujoco.MjData, name=None):
    """
    获取电机数据并返回：
    - 电机名称
    - 所述物体
    - 电机扭矩`(torque)`大小

    Args:
        model (mujoco.MjModel): mjc 的模型
        data (mujoco.MjData): mjc 的数据
        name (str/list): 需要查找的 actor 名字或名字列表

    Returns:
        list[3]: [0]: actor 的名字 | [1]: actor 附身的 joint | [2]: actor 的扭矩`torque`
    """
    if name is not None and type(name) is not list:
        name = [name]

    res = []

    if name is None:  # 如果没有提供范围，自动获取所有的 actor
        for _id in range(model.nu):
            res.append(
                [
                    model.actuator(_id).name,
                    model.joint(model.actuator_trnid[_id][0]).name,
                    data.ctrl[_id],
                ]
            )
    else:
        for _name in range(len(name)):
            _id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name[_name])
            res.append(
                [
                    name[_id],
                    model.joint(model.actuator_trnid[_id][0]).name,
                    data.ctrl[_id],
                ]
            )

    return res


def get_info_jointstate(model: mujoco.MjModel, data: mujoco.MjData, name=None):
    """
    获取关节数据并返回：
    - 关节名称
    - 关节位置
    - 关节速度
    - 关节效果

    Args:
        model (mujoco.MjModel): mjc 的模型
        data (mujoco.MjData): mjc 的数据
        name (str/list): 需要查找的 joint 名字或名字列表

    Returns:
        list[3]: [0] str: joint 的名字 | [1] list[3]: joint 的位置 | [2] list[6]: joint 的速度 (rot:lin 角速度:线速度) | [3] float: joint 的力/力矩 (x 还没搞定)
    """
    if name is not None and type(name) is not list:
        name = [name]

    res = []

    id = []
    # id.extend(range(model.njnt) if name is None else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, _name) for _name in name)
    if name is None:  # 如果没有提供范围，自动获取所有的 joint
        id.extend(range(model.njnt))
    else:
        for _name in range(len(name)):
            id.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name[_name]))

    for _id in id:
        bodyid = model.joint(_id).bodyid[0]
        res.append(
            [
                model.joint(_id).name,
                data.xpos[bodyid],
                data.cvel[bodyid],  # rot:lin 角速度:线速度
                # 缺少
            ]
        )

    return res


# ========== 获取信息  ========== #
def set_model_attribute(
    model: mujoco.MjModel,
    source,
    model_name: str,
    type: type = mujoco.mjtObj.mjOBJ_BODY,
    attribute: str = None,
    value: any = None,
) -> None:
    """
    设置模型属性

    属性名速查：https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html#mjmodel


    Args:
        model (mujoco.MjModel): mjc 的模型
        source: 数据来源，物体信息等静态的从 mjModel 里面获取，速度信息等动态的从 mjData 里面获取
        model_name (str): 模型名称，XML 标签里面的 name
        type (type): 模型类型
        attribute (str): 属性名，mjModel 结构体里面的
        value (any): 属性值
    """
    model_id = mujoco.mj_name2id(model, type, model_name)
    try:
        getattr(source, attribute)[model_id] = value
    except ValueError as e:
        print(e)
    # FIXME: 写的太丑了，如果可以的话，可以考虑改一下这俩函数


def get_model_attribute(
    model: mujoco.MjModel,
    source,
    model_name: str,
    type: type = mujoco.mjtObj.mjOBJ_BODY,
    attribute: str = None,
):
    """
    获取模型属性

    属性名速查：https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html#mjmodel

    Args:
        model (mujoco.MjModel): mjc 的模型
        source: 数据来源，物体信息等静态的从 mjModel 里面获取，速度信息等动态的从 mjData 里面获取
        model_name (str): 模型名称，XML 标签里面的 name
        type (type): 模型类型
        attribute (str): 属性名，mjModel 结构体里面的

    Returns:
        any: 属性值
    """
    model_id = mujoco.mj_name2id(model, type, model_name)
    return getattr(source, attribute)[model_id]
