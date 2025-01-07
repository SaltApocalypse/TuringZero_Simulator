import mujoco
import mujoco.viewer
import time
import numpy as np

import PID_
import pickle

# import tbkpy
from api.NewTBKApi import tbk_manager
import tbkpy._core as tbkpy

# import keyboard


subInfo = tbkpy.EPInfo()
subInfo.msg_name = "RPM"
subInfo.name = "RPM"

pubInfo = tbkpy.EPInfo()
pubInfo.msg_name = "RPM11"
pubInfo.name = "RPM11"


# ========== Setting ==========
setting = {}
setting["late_start"] = False  # 延迟五秒启动 default = False
setting["press_start"] = False  # 按下启动启动 default = False
setting["get_status"] = True  # 监控电机状态 default = True
setting["debug_keyboard"] = False  # 调试键盘 default = False
setting["tbk_sub"] = False  # TBK 接收消息 default = True
setting["tbk_pub"] = False  # TBK 发送消息 default = True

# ========== val ==========

model = mujoco.MjModel.from_xml_path("./turingzero_agv/tz_agv.xml")
# model = mujoco.MjModel.from_xml_path("./Simulator/turingzero_agv/tz_agv.xml")
data = mujoco.MjData(model)
spec = mujoco.MjSpec()

motor = [6, 15, 24]  # motors' ids
joint_if = 0

motor_torque = np.array([0] * 3)
motor_kv = 1

command = None


global tick

# ========== func ==========
# def set_torque(ctrl: np.array = np.array([0] * 3), kv: int = motor_kv):
#     """设置电机扭矩
#     @param
#     - ctrl(np.array): 前轮、左（后）轮、右（后）轮的控制力量
#     - kv(int): 增益倍数
#     """
#     for i in range(0, len(motor), 2):
#         data.ctrl[motor[i]] = ctrl[i] * kv


def get_motor_status(state: int = 1 << 0, time_interval: int = 1000):
    """报告电机状态
    @param
    - state(int): 电机状态选项
        - bit 0: 扭矩
        - bit 1: 速度
        - bit 2: 加速度
    - time_interval(int): 报告生成周期
    """
    global tick
    if 0 == tick % time_interval:
        print(f"tick = {tick}:")
        if state & (1 << 0):
            print("motors' status:")
            for i in range(0, len(motor)):
                print(f"- motor{motor[i]}: {data.ctrl[i]:.3f}")
        if state & (1 << 1):
            print("velocity status:")
            for i in range(0, len(motor)):
                print(f"- motor{motor[i]}: {data.qvel[motor[i]]:.3f}")
        if state & (1 << 2):
            print("acceleration status:")
            for i in range(0, len(motor)):
                print(f"- motor{motor[i]}: {data.qacc[motor[i]]:.3f}")
            print()
        print("body's status:")
        print(f"vel: {data.qvel[joint_if]}")
        print(f"acc: {data.qacc[joint_if]}")
        print()


def control(msg):
    # print("[CONTROL] I get a cmd!")
    # target_velocity = msg
    current_velocity = []

    # 解包指令
    msg = pickle.loads(msg)
    # print(msg)
    vx, vz, w = msg
    target_velocity = PID_.motor2self(vx, vz, w)

    # 对于三个轮子
    pid = [PID_.PositionPID(), PID_.PositionPID(), PID_.PositionPID()]

    length = len(motor)

    for i in range(length):
        current_velocity.append(data.qvel[motor[i]])
    for i in range(length):
        torque, reached_target = pid[i].update(current_velocity[i], target_velocity[i])
        data.ctrl[i] = torque


if setting["tbk_sub"]:
    subRegister = tbkpy.Subscriber(subInfo, control)

pubRegister = tbkpy.Publisher(subInfo)


# 收集所有信息
def publish_msg(m: mujoco.MjModel, d: mujoco.MjData, spec):
    state = np.zeros(mujoco.mj_stateSize(model, spec))
    mujoco.mj_getState(m, d, state, spec)
    # print(state)
    msg = pickle.dumps(state)
    pubRegister.publish(msg)


# ========== camera ==========

import tkinter as tk
import threading


def camera_module(model: mujoco.MjModel, data: mujoco.MjData):
    cam = mujoco.mjvCamera
    opt = mujoco.mjvoption
    scn = mujoco.mjvScene
    con = mujoco.mjrContext

    ###
    # 这里做一些 GUI 的事情
    ###

    mujoco.mjv_defaultCamera(cam)
    # mujoco.mjv_defaultPerturb(pert)
    mujoco.mjv_defaultOption(opt)
    mujoco.mjr_defaultContext(con)

    mujoco.mjv_makeScene(model, scn, 1000)
    mujoco.mjv_makeContext(model, con, mujoco.mjFONTSCALE_100)

    ###
    # 这里做一些GUI的事情（跑窗口循环）
    ###
    while None:
        pass
        mujoco.mj_step(model, data)

    viewport = mujoco.mjrRect(np.zeros(4))
    None  # 自GUI获取帧缓冲

    # 更新场景和渲染
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjCAT_ALL, scn)
    mujoco.mjr_render(viewport, scn, con)

    None  # GUI交换缓冲
    None  # GUi的事件处理（鼠标事件监听等）

    # 清理
    mujoco.mjv_freeScene(scn)
    mujoco.mjr_freeContext(con)


# ========== main ==========
with mujoco.viewer.launch_passive(model, data) as viewer:
    camera = mujoco.MjvCamera()

    # before running
    tick = 0

    if setting["press_start"]:
        input()
    if setting["late_start"]:
        time.sleep(5)

    # ========== regit tbk_manager ==========

    v = [0, 0, 0]

    # running
    while viewer.is_running():
        mujoco.mj_step(model, data)

        # 监控日志
        if setting["get_status"]:
            get_motor_status(1 << 0 | 1 << 1, 1000)

        if setting["tbk_pub"]:
            publish_msg(model, data, mujoco.mjtState.mjSTATE_PHYSICS)

        # if tick % 1000 == 0:
        #     sensordata = data.sensordata
        #     print(f"{tick}: {sensordata}")

        # if tick != 0 and 0 == tick % 1000:
        #     command = lambda v=v: control(v)  # 发布任务

        # 任务监控
        if command is not None and callable(command):
            pass
            # command()

        # 收集数据
        # if 0 == tick % 1000:
        #     pass
        #     collect_datas(model, data, 1 << 1, spec)

        viewer.sync()
        tick += 1
