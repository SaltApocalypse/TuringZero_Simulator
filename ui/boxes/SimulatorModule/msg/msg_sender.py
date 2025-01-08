import tbkpy._core as tbkpy
class MsgType:
    pass
class MsgSender:
    def __init__(self):
        pass
    def init(self):
        tbkpy.init("TZ-Simulator")
    def register_pub(self, msg_name,msg_type):
        info = tbkpy.EPInfo()
        info.msg_name = msg_name
        info.msg_type = msg_type
        pub = tbkpy.Publisher(info)
        return pub
    def publish(self, pub, type,msg):
        
        pub.publish()
