import tbkpy._core as tbkpy

class MsgSender:
    def __init__(self,msg_name,msg_type):
        info = tbkpy.EPInfo()
        info.msg_name = msg_name
        info.msg_type = msg_type
        self.pub = tbkpy.Publisher(info)
    def init(self):
        tbkpy.init("TZ-Simulator")
    def publish(self, pub, msg):
        msg.SerializeToString()
        pub.publish(msg)
