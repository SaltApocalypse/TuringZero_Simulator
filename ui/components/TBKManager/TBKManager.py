import inspect
import os
import sys
import time
import types

from google.protobuf.internal import builder

from utils.ClientLogManager import client_logger
from utils.ModuleLazyLoader import ModuleLazyLoader
from .EtcdClient import etcd_client


# def build_param_tree(flat_dict):
#     tree = {}
#
#     for key, value in flat_dict.items():
#         parts = key.split("/")
#         current_level = tree
#
#         for part in parts[:-1]:  # 遍历层级中的所有部分（除了最后一个）
#             if part not in current_level:
#                 current_level[part] = {}
#             current_level = current_level[part]
#
#         # 处理最后一个部分，可能包含冒号
#         last_part = parts[-1]
#         sub_parts = last_part.split(":")
#
#         for sub_part in sub_parts[:-1]:
#             if sub_part not in current_level:
#                 current_level[sub_part] = {}
#             current_level = current_level[sub_part]
#
#         # 最后的部分是叶节点
#         current_level[sub_parts[-1]] = value
#     return tree


class TBKManager:
    def __init__(self, name="TBKNode"):
        self.name = name
        # 动态导入库
        self.all_modules = []
        self._all_types = None
        self.tbkpy = ModuleLazyLoader("tbkpy._core", self.tbkpy_init)

        self.etcd = etcd_client
        # self.param_tree = None
        # self._param_data = None
        # self._message_data = None

        self.callback_dict = {}
        self.subscriber_dict = {}
        self.publisher_dict = {}

    def tbkpy_init(self):
        self.tbkpy.init(self.name)

    @property
    def all_types(self):
        if self._all_types is None:
            self.all_types_init()
        return self._all_types

    def all_types_init(self):
        from tzcp.tbk import tbk_pb2
        from tzcp.ros import std_pb2

        # 加载 protobuf type 类型
        self._all_types = types.ModuleType("all_types")
        sys.modules["all_types"] = self._all_types
        self.load_module(tbk_pb2)
        self.load_module(std_pb2)

    def load_module(self, module):
        descriptor = module.DESCRIPTOR
        package_name = descriptor.package
        builder.BuildTopDescriptorsAndMessages(descriptor, package_name, self.all_types.__dict__)
        self.all_modules.append(module)

    def unsubscribe(self, name, msg_name, tag, **kwargs):
        print(self.subscriber_dict)
        del self.callback_dict[name][msg_name][tag]
        if len(self.callback_dict[name][msg_name]) < 1:
            del self.subscriber_dict[name][msg_name]

    def is_subscribed(self, name, msg_name, **kwargs) -> bool:
        return self.subscriber_dict.get(name, {}).get(msg_name, None) is not None

    def callback_manager(self, name, msg_name, msg, msg_type, **kwargs):
        try:
            raw_msg = getattr(self.all_types, msg_type)()
            raw_msg.ParseFromString(msg)
        except:
            raw_msg = msg
            client_logger.log("WARNING", f"name: {name}, msg_name: {msg_name}, Automatic parsing failed!")

        for tag, callback_func in self.callback_dict[name][msg_name].items():
            try:
                callback_func(raw_msg)
            except Exception as e:
                client_logger.log("ERROR", f"Run callback_func {callback_func.__name__} error!", e)

    def subscriber(self, name, msg_name, tag, **kwargs):
        msg_type = self.etcd.get_pub_msg_type(name, msg_name)
        callback = kwargs.get("callback", None)
        self.callback_dict.setdefault(name, {}).setdefault(msg_name, {})[tag] = callback

        if self.subscriber_dict.get(name, {}).get(msg_name) is not None:
            # 如果已经订阅则退出
            return
        client_logger.log("INFO", f"Add new subscriber({name, msg_name}).")
        self.subscriber_dict.setdefault(name, {})[msg_name] = self.tbkpy.Subscriber(
            name,
            msg_name,
            lambda msg: self.callback_manager(name, msg_name, msg, msg_type, **kwargs),
        )

    def publisher(self, name, msg_name, msg_type, **kwargs):
        if inspect.isclass(msg_type):
            try:
                msg_type = msg_type.__name__
            except:
                msg_type = "Unknown"
        ep_info = self.tbkpy.EPInfo()
        ep_info.name = name
        ep_info.msg_name = msg_name
        ep_info.msg_type = msg_type
        self.publisher_dict.setdefault(name, {})[msg_name] = self.tbkpy.Publisher(ep_info)
        return self.publisher_dict[name][msg_name]

    def list(self, **kwargs):
        return self.etcd.list(**kwargs)

    def get(self, **kwargs):
        r = self.etcd.get(**kwargs)[0]
        r = r.decode() if isinstance(r, bytes) else r.__str__()
        return r

    def put(self, **kwargs):
        return self.etcd.put(**kwargs)

    def delete(self, **kwargs):
        return self.etcd.delete(**kwargs)[1]


tbk_manager = TBKManager("simulator")

if __name__ == "__main__":
    # from Utils.ProtobufManager import ProtobufManager
    #
    # pm = ProtobufManager()
    #
    # file = "/home/mark/anaconda3/envs/platform_war/lib/python3.11/site-packages/tzcp/ros/std_pb2.py"
    #
    # descriptor = b'\n\x12tzcp/ros/std.proto\x12\ntz.ros.std\"\x14\n\x04\x42ool\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x08\"\x14\n\x04\x42yte\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x14\n\x04\x43har\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x17\n\x07\x46loat32\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x02\"\x17\n\x07\x46loat64\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x01\"\x14\n\x04Int8\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x05\"\x15\n\x05Int16\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x05\"\x15\n\x05Int32\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x05\"\x15\n\x05Int64\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x03\"\x15\n\x05UInt8\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x16\n\x06UInt16\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x16\n\x06UInt32\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x16\n\x06UInt64\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x04\"\x16\n\x06String\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\t\"7\n\tColorRGBA\x12\t\n\x01r\x18\x01 \x01(\x02\x12\t\n\x01g\x18\x02 \x01(\x02\x12\t\n\x01\x62\x18\x03 \x01(\x02\x12\t\n\x01\x61\x18\x04 \x01(\x02\"\'\n\x08\x44uration\x12\x0c\n\x04secs\x18\x01 \x01(\x05\x12\r\n\x05nsecs\x18\x02 \x01(\x05\"\x07\n\x05\x45mpty\"#\n\x04Time\x12\x0c\n\x04secs\x18\x01 \x01(\x05\x12\r\n\x05nsecs\x18\x02 \x01(\x05\"6\n\x06Header\x12\x0b\n\x03seq\x18\x01 \x01(\r\x12\r\n\x05stamp\x18\x02 \x01(\x02\x12\x10\n\x08\x66rame_id\x18\x03 \x01(\t\"B\n\x13MultiArrayDimension\x12\r\n\x05label\x18\x01 \x01(\t\x12\x0c\n\x04size\x18\x02 \x01(\r\x12\x0e\n\x06stride\x18\x03 \x01(\r\"U\n\x10MultiArrayLayout\x12,\n\x03\x64im\x18\x01 \x03(\x0b\x32\x1f.tz.ros.std.MultiArrayDimension\x12\x13\n\x0b\x64\x61ta_offset\x18\x02 \x01(\r\"L\n\x0e\x42yteMultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x01(\x0c\"N\n\x10UInt32MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\r\"N\n\x10UInt64MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x04\"M\n\x0fInt32MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x05\"M\n\x0fInt64MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x03\"O\n\x11\x46loat32MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x02\"O\n\x11\x46loat64MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x01\x62\x06proto3'
    #
    # pm.load_from_descriptor(descriptor)
    # descriptor2 = b'\n\x18tzcp/ros/ackermann.proto\x12\x10tz.ros.achermann\x1a\x12tzcp/ros/std.proto\"|\n\x0e\x41\x63kermannDrive\x12\x16\n\x0esteering_angle\x18\x01 \x01(\x02\x12\x1f\n\x17steering_angle_velocity\x18\x02 \x01(\x02\x12\r\n\x05speed\x18\x03 \x01(\x02\x12\x14\n\x0c\x61\x63\x63\x65leration\x18\x04 \x01(\x02\x12\x0c\n\x04jerk\x18\x05 \x01(\x02\"l\n\x15\x41\x63kermannDriveStamped\x12\"\n\x06header\x18\x01 \x01(\x0b\x32\x12.tz.ros.std.Header\x12/\n\x05\x64rive\x18\x02 \x01(\x0b\x32 .tz.ros.achermann.AckermannDriveb\x06proto3'
    #
    # pm.load_from_descriptor(descriptor2)
    #
    # # for k, v in tbk_manager.api_list().items():
    # #     print(k, v)
    #
    # # 创建 AckermannDrive 消息
    # drive_msg = pm.AckermannDrive()
    # drive_msg.steering_angle = 0.5  # 转向角度（弧度）
    # drive_msg.steering_angle_velocity = 0.1  # 转向角速度
    # drive_msg.speed = 2.0  # 速度（米/秒）
    # drive_msg.acceleration = 1.0  # 加速度
    # drive_msg.jerk = 0.5  # 加加速度
    # # 创建带时间戳的 AckermannDriveStamped 消息
    # stamped_msg = pm.AckermannDriveStamped()
    # # 设置头部信息
    # header = pm.Header()
    # header.seq = 123  # 秒
    # header.stamp = 456000000  # 纳秒
    # header.frame_id = "base_link"
    # stamped_msg.header.CopyFrom(header)
    # # 设置drive信息
    # stamped_msg.drive.CopyFrom(drive_msg)
    # # 序列化消息（如果需要）
    # serialized_msg = stamped_msg.SerializeToString()
    # print(serialized_msg)
    # # 反序列化消息（如果需要）
    # recovered_msg = pm.AckermannDriveStamped()
    # recovered_msg.ParseFromString(serialized_msg)
    #
    #
    # def f(msg):
    #     recovered_msg = pm.AckermannDriveStamped()
    #     try:
    #         recovered_msg.ParseFromString(msg)
    #     except Exception as e:
    #         client_logger.log("ERROR", "error", e)
    #     print(recovered_msg)
    from tzcp.tbk import tbk_pb2 as tbkpb

    puber = tbk_manager.publisher(name="test", msg_name="test2", msg_type=tbkpb.State)
    tbk_manager.subscriber(name="test", msg_name="test2", tag="123", callback_func=print)

    i = 0
    while True:
        if i > 5:
            break
        i += 1
        msg = tbkpb.State()
        msg.uuid = "testuuid"

        puber.publisher()
        # tbk_manager.pub(name="test", msg_name="test2", msg="test123123")
        time.sleep(0.01)
