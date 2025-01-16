# pip install mujoco-python-viewer

from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
import mujoco

from ui.components.TBKManager.TBKManager import tbk_manager


class TBKDemo(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (1200, 900)

        self.puber = tbk_manager.publisher(name="tz_agv", msg_name="tz_agv_status")

        # tbk_manager.subscriber(name="test", msg_name="test2", tag="123", callback_func=print)
        # tbk_manager.subscriber(name="test", msg_name="test3", tag="456", callback_func=print)
        tbk_manager.subscriber(name="test", msg_name="test4", tag="456", callback_func=lambda msg: print(msg.decode()))
        # tbk_manager.subscriber(name="test", msg_name="test4", tag="456", callback_func=print)

    def create(self):
        pass

    def update(self):
        msg = tbk_manager.all_types.State()
        msg.uuid = "testuuid"

        msg2 = tbk_manager.all_types.Int64()
        msg2.data = 123

        # self.puber.publish(msg.SerializeToString())
        # self.puber2.publish(msg2.SerializeToString())
        self.puber3.publish("123")


    def destroy(self):
        super().destroy()
