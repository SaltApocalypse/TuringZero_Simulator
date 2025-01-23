import mujoco

from ...components.TBKManager.ParamData import ParamData
import ui.boxes.SimulatorModule.SimulatorUtils as utils

dict = {}
# def param_dict_init():
dict["head_marker"] = ["rgba", "pos"]
dict["tz_agv"] = ["pos", "quat"]
dict["front_wheel_rolling_joint"] = ["qvel"]
dict["left_wheel_rolling_joint"] = ["qvel"]
dict["right_wheel_rolling_joint"] = ["qvel"]


def param_init(mjModel, mjData):
    for key in dict.keys:
        name = key
        for value in dict[key]:
            print(value)
            # ParamData(f"tz_agv/{key}", f"{value}")


#     head_marker_name = "head_marker"
#     head_marker_rgba = ParamData(f"tz_agv/{head_marker_name}", "rgba")
#     head_marker_rgba.value = utils.get_model_attribute(mjModel, head_marker_name, mujoco.mjtObj.mjOBJ_GEOM, "rgba")

#     tz_agv_name = "tz_agv"
#     tz_agv_pos = ParamData(f"{tz_agv_name}", "pos")
#     tz_agv_pos.value = utils.get_model_attribute(mjModel, tz_agv_name, mujoco.mjtObj.mjOBJ_BODY, "pos")
#     tz_agv_quat = ParamData(f"{tz_agv_name}", "quat")
#     tz_agv_quat.value = utils.get_model_attribute(mjModel, tz_agv_name, mujoco.mjtObj.mjOBJ_BODY, "quat")

#     front_wheel_rolling_joint_name = "front_wheel_rolling_joint"
#     front_wheel_rolling_joint_qvel = ParamData(f"{front_wheel_rolling_joint_name}", "qvel")
#     front_wheel_rolling_joint_qvel.value = utils.get_model_attribute(mjData, front_wheel_rolling_joint_name, mujoco.mjtObj.mjOBJ_JOINT, "qvel")

#     left_wheel_rolling_joint_name = "left_wheel_rolling_joint"
#     left_wheel_rolling_joint_qvel = ParamData(f"{left_wheel_rolling_joint_name}", "qvel")
#     left_wheel_rolling_joint_qvel.value = utils.get_model_attribute(mjData, left_wheel_rolling_joint_name, mujoco.mjtObj.mjOBJ_JOINT, "qvel")

#     right_wheel_rolling_joint_name = "right_wheel_rolling_joint"
#     right_wheel_rolling_joint_qvel = ParamData(f"{right_wheel_rolling_joint_name}", "qvel")
#     right_wheel_rolling_joint_qvel.value = utils.get_model_attribute(mjData, right_wheel_rolling_joint_name, mujoco.mjtObj.mjOBJ_JOINT, "qvel")


# def param_update(mjModel, mjData):
#     head_marker_name = "head_marker"
#     utils.set_model_attribute(mjModel, head_marker_name, mujoco.mjtObj.mjOBJ_GEOM, "rgba", head_marker_rgba.value)

#     tz_agv_name = "tz_agv"
#     utils.set_model_attribute(mjModel, tz_agv_name, mujoco.mjtObj.mjOBJ_BODY, "pos", tz_agv_pos.value)

#     front_wheel_rolling_joint_name = "front_wheel_rolling_joint"
#     utils.set_model_attribute(mjModel, tz_agv_name, mujoco.mjtObj.mjOBJ_BODY, "quat", tz_agv_quat.value)
#     utils.set_model_attribute(mjData, front_wheel_rolling_joint_name, mujoco.mjtObj.mjOBJ_JOINT, "qvel", front_wheel_rolling_joint_qvel.value)
#     left_wheel_rolling_joint_qvel = ParamData(f"{left_wheel_rolling_joint_name}", "qvel")

#     left_wheel_rolling_joint_name = "left_wheel_rolling_joint"
#     utils.set_model_attribute(mjData, left_wheel_rolling_joint_name, mujoco.mjtObj.mjOBJ_JOINT, "qvel", left_wheel_rolling_joint_qvel.value)

#     right_wheel_rolling_joint_name = "right_wheel_rolling_joint"
#     utils.set_model_attribute(mjData, right_wheel_rolling_joint_name, mujoco.mjtObj.mjOBJ_JOINT, "qvel", right_wheel_rolling_joint_qvel.value)
