import mujoco

from ...components.TBKManager.ParamData import ParamData
from .SimulatorUtils import *


class SimulatorParam:
    def __init__(self, mjModel, mjData):
        self.mjModel = mjModel
        self.mjData = mjData

        self.dict = {}
        self.param_dict_init(self.mjModel, self.mjData)

    def param_dict_init(self, mjModel, mjData):
        self.dict["head_marker"] = [mjModel, mujoco.mjtObj.mjOBJ_GEOM, "geom_rgba"]
        self.dict["head_marker"] = [mjData, mujoco.mjtObj.mjOBJ_GEOM, "qpos"]
        self.dict["tz_agv"] = [mjModel, mujoco.mjtObj.mjOBJ_BODY, "quat"]
        self.dict["tz_agv"] = [mjData, mujoco.mjtObj.mjOBJ_BODY, "qpos"]
        self.dict["front_wheel_rolling_joint"] = [mjData, mujoco.mjtObj.mjOBJ_JOINT, "qvel"]
        self.dict["left_wheel_rolling_joint"] = [mjData, mujoco.mjtObj.mjOBJ_JOINT, "qvel"]
        self.dict["right_wheel_rolling_joint"] = [mjData, mujoco.mjtObj.mjOBJ_JOINT, "qvel"]

    def param_init(self, mjModel, mjData):
        for model_name in self.dict.keys():
            value = self.dict[model_name]
            source = value[0]
            type = value[1]
            for i in range(2, len(value)):
                param_name = f"{model_name}_{value[i]}"
                prefix = f"tz_agv/{model_name}"
                name = value[i]  # 变量
                deafult_value = get_model_attribute(
                    model=mjModel,
                    source=source,
                    model_name=model_name,
                    type=type,
                    attribute=name,
                )
                setattr(self, param_name, ParamData(prefix=prefix, name=name, default_value=str(deafult_value)))

    def param_update(self, mjModel, mjData):
        for model_name in self.dict.keys():
            value = self.dict[model_name]
            source = value[0]
            type = value[1]
            for i in range(2, len(value)):
                param_name = f"{model_name}_{value[i]}"
                prefix = f"tz_agv/{model_name}"
                name = value[i]
                set_model_attribute(
                    model=mjModel,
                    source=source,
                    model_name=model_name,
                    type=type,
                    attribute=name,
                    value=getattr(self, param_name).value,
                )
