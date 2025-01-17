# pip install mujoco-python-viewer

from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
from ui.components.TBKManager.TBKManager import tbk_manager

import pickle

# proto include
# from .proto.python import actor_info_pb2
# from .proto.python import imu_info_pb2
# from .proto.python import jointstate_info_pb2


class TBKDemo(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (1200, 900)

        # tbk
        # tbk_manager.load_module(actor_info_pb2)
        # tbk_manager.load_module(imu_info_pb2)
        # tbk_manager.load_module(jointstate_info_pb2)

        # agv status subscriber
        # self.__suber_status_imu = tbk_manager.subscriber(name="tz_agv", msg_name="tz_agv_status_imu", tag="IMUInfo", callback_func=print)
        # self.__suber_status_actor = tbk_manager.subscriber(name="tz_agv", msg_name="tz_agv_status_actor", tag="ActorInfo", callback_func=print)
        # self.__suber_status_jointstate = tbk_manager.subscriber(name="tz_agv", msg_name="tz_agv_status_jointstate", tag="JointStateInfo", callback_func=print)

        # agv run command
        self.__puber_command = tbk_manager.publisher(name="tz_agv", msg_name="tz_agz_command", msg_type="list")

    def create(self):
        pass

    def update(self):
        self.__puber_command.publish(pickle.dumps([5, 0, 0]))

    def destroy(self):
        super().destroy()
