# pip install mujoco-python-viewer

from ui.boxes.BaseBox import BaseBox
from ui.components.CanvasMuJoCo import CanvasMuJoCo
import mujoco
from tzcp.tbk import tbk_pb2
from tzcp.ros import std_pb2

from ui.components.TBKManager.TBKManager import tbk_manager


class TBKDemo(BaseBox):
    only = True

    def __init__(self, ui, **kwargs):
        super().__init__(ui, **kwargs)
        self.size = (1200, 900)
        self.mj_model = mujoco.MjModel.from_xml_path('ui/boxes/SimulatorModule/test.xml')
        self.mj_data = mujoco.MjData(self.mj_model)


        self.puber = tbk_manager.publisher(name="test", msg_name="test2", msg_type=tbk_pb2.State)
        self.puber2 = tbk_manager.publisher(name="test", msg_name="test3", msg_type=std_pb2.Int64)
        self.puber3 = tbk_manager.publisher(name="test", msg_name="test4", msg_type="int")


        tbk_manager.subscriber(name="test", msg_name="test2", tag="123", callback_func=print)
        tbk_manager.subscriber(name="test", msg_name="test3", tag="456", callback_func=print)
        tbk_manager.subscriber(name="test", msg_name="test4", tag="456", callback_func=print)


    def create(self):
        self.canvas = CanvasMuJoCo(parent=self.tag, size=self.size, mj_model=self.mj_model, mj_data=self.mj_data)

    def update(self):
        self.canvas.update()

        msg = tbk_pb2.State()
        msg.uuid = "testuuid"

        msg2 = std_pb2.Int64()
        msg2.data = 123

        self.puber.publish(msg.SerializeToString())
        self.puber2.publish(msg2.SerializeToString())
        self.puber3.publish("zidingyi")

    def destroy(self):
        super().destroy()
