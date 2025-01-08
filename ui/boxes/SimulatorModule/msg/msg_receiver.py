import tbkpy._core as tbkpy
class MsgReceiver:
    def __init__(self):
        pass
    def init(self):
        tbkpy.init("TZ-Simulator")
    def register_pub(self, msg_name,msg_type,cbk):
        info = tbkpy.EPInfo()
        info.msg_name = msg_name
        info.msg_type = msg_type
        pub = tbkpy.Subscriber(info)
    def f_decode(self, msg):
        msg_type = exec(f"import {msg_type}")
        msg.ParseFromString(msg)
