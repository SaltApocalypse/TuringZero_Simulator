import importlib
import os
import time

from utils.ClientLogManager import client_logger
import functools


def build_param_tree(flat_dict):
    tree = {}

    for key, value in flat_dict.items():
        parts = key.split("/")
        current_level = tree

        for part in parts[:-1]:  # 遍历层级中的所有部分（除了最后一个）
            if part not in current_level:
                current_level[part] = {}
            current_level = current_level[part]

        # 处理最后一个部分，可能包含冒号
        last_part = parts[-1]
        sub_parts = last_part.split(":")

        for sub_part in sub_parts[:-1]:
            if sub_part not in current_level:
                current_level[sub_part] = {}
            current_level = current_level[sub_part]

        # 最后的部分是叶节点
        current_level[sub_parts[-1]] = value

    return tree


class ModuleLazyLoader:
    def __init__(self, module_path, callback=None):
        self._module_path = module_path
        self._module = None
        self.callback = callback

    def __getattr__(self, name):
        if self._module is None:
            try:
                self._module = importlib.import_module(self._module_path)
                if self.callback is not None:
                    self.callback()
            except ImportError:
                raise ImportError(f"Could not import module {self._module_path}")
        return getattr(self._module, name)

class TBKManager:
    def __init__(self):
        # 动态导入库
        self.etcd3 = ModuleLazyLoader('etcd3', self.etcd_init)
        self.tbkpb = ModuleLazyLoader('tzcp.tbk.tbk_pb2')
        self.tbkpy = ModuleLazyLoader('tbkpy._core', self.tbkpy_init)

        self.param_tree = None
        self._param_data = None
        self._message_data = None
        self.etcd = None
        self.MESSAGE_PREFIX = "/tbk/ps"
        self.PARAM_PREFIX = "/tbk/params"

        self.callback_dict = {}
        self.subscriber_dict = {}
        self.publisher_dict = {}

    def etcd_init(self):
        self.etcd = self._client()

    def tbkpy_init(self):
        self.tbkpy.init("simulator")

    def _client(self):
        pki_path = os.path.join(os.path.expanduser("~"), ".tbk/etcdadm/pki")
        return self.etcd3.client(
            host="127.0.0.1",
            port=2379,
            ca_cert=os.path.join(pki_path, "ca.crt"),
            cert_key=os.path.join(pki_path, "etcdctl-etcd-client.key"),
            cert_cert=os.path.join(pki_path, "etcdctl-etcd-client.crt"),
        )

    def get_original_param(self, _prefix=None):
        prefix = self.PARAM_PREFIX + (_prefix if _prefix else "")
        raw_data = self.etcd.get_prefix(prefix)
        data = dict([(r[1].key.decode("utf-8", errors="ignore")[12:], r[0].decode("utf-8", errors="ignore")) for r in raw_data])
        return data

    def get_param(self, _prefix=None):
        data = self.get_original_param(_prefix)
        result = {}
        # 遍历原始字典并分类存储
        for key, value in data.items():
            base_key = key.rsplit("/", 1)[0]
            suffix = key.rsplit("/", 1)[1]
            if base_key not in result:
                result[base_key] = {"info": None, "type": None, "value": None}
            if suffix == "__i__":
                result[base_key]["info"] = value
            elif suffix == "__t__":
                result[base_key]["type"] = value
            elif suffix == "__v__":
                result[base_key]["value"] = value
        self.param_tree = build_param_tree(result)
        return result

    @property
    def param_data(self):
        self._param_data = self.get_param()
        return self._param_data

    def get_message(self):
        processes = {}
        publishers = {}
        subscribers = {}
        res = self.etcd.get_prefix(self.MESSAGE_PREFIX)
        for r in res:
            key, value = r[1].key.decode(), r[0]
            keys = key[len(self.MESSAGE_PREFIX) :].split("/")[1:]
            info = None
            if len(keys) == 1:
                info = self.tbkpb.State()
                info.ParseFromString(value)
                processes[info.uuid] = info
            elif len(keys) == 3:
                if keys[1] == "pubs":
                    info = self.tbkpb.Publisher()
                    info.ParseFromString(value)
                    publishers[info.uuid] = info
                elif keys[1] == "subs":
                    info = self.tbkpb.Subscriber()
                    info.ParseFromString(value)
                    subscribers[info.uuid] = info
            else:
                client_logger.log("ERROR", f"TBKApi: Error: key error:{key}")
        res = {"ps": processes, "pubs": publishers, "subs": subscribers}
        return res

    @property
    def message_data(self):
        self._message_data = self.get_message()
        return self._message_data

    @property
    def message_tree(self):
        message_tree = {}
        for node_type in self.message_data:
            tree = {}
            if node_type == "ps":
                continue
            elif node_type == "subs":
                continue
            elif node_type == "pubs":
                data = self.message_data[node_type]
                for uuid in data:
                    node_name = data[uuid].ep_info.node_name
                    if node_name == "simulator":
                        puuid = node_name
                    else:
                        puuid = f"{node_name}_{data[uuid].puuid}"
                    if puuid not in tree:
                        tree[puuid] = {}
                    tree[puuid][uuid] = data[uuid]
            else:
                client_logger.log("ERROR", f"{self.__class__} build message_tree type error!")
            message_tree[node_type] = tree
        return message_tree

    def unsubscribe(self, **kwargs):
        name = kwargs.get("name", None)
        msg_name = kwargs.get("msg_name", None)
        msg_type = kwargs.get("msg_type", None)
        tag = kwargs.get("tag", None)
        info = (name, msg_name)
        del self.callback_dict[info][tag]
        if len(self.callback_dict[info]) < 1:
            del self.subscriber_dict[info]

    def is_subscribed(self, **kwargs) -> bool:
        name = kwargs.get("name", None)
        msg_name = kwargs.get("msg_name", None)
        info = (name, msg_name)
        return info in self.subscriber_dict

    def callback_manager(self, msg, **kwargs):
        name = kwargs.get("name", None)
        msg_name = kwargs.get("msg_name", None)
        info = (name, msg_name)
        for tag, callback_func in self.callback_dict[info].items():
            callback_func(msg)

    def subscriber(self, **kwargs):
        name = kwargs.get("name", None)
        msg_name = kwargs.get("msg_name", None)
        msg_type = kwargs.get("msg_type", None)
        tag = kwargs.get("tag", None)
        info = (name, msg_name)
        callback_func = kwargs.get("callback_func", None)

        self.callback_dict.setdefault(info, {})[tag] = callback_func

        if self.subscriber_dict.get(info) is not None:
            # 如果已经订阅则退出
            return
        client_logger.log("INFO", f"Add new subscriber({info})")
        self.subscriber_dict[info] = self.tbkpy.Subscriber(
            name,
            msg_name,
            lambda msg: self.callback_manager(msg, **kwargs),
        )

    def publisher(self, **kwargs):
        name = kwargs.get("name", None)
        msg_name = kwargs.get("msg_name", None)
        msg_type = kwargs.get("msg_type", "Unknown")
        info = (name, msg_name)
        ep_info = self.tbkpy.EPInfo()
        ep_info.name = name
        ep_info.msg_name = msg_name
        ep_info.msg_type = msg_type
        self.publisher_dict[info] = self.tbkpy.Publisher(ep_info)

    def pub(self, **kwargs):
        name = kwargs.get("name", None)
        msg_name = kwargs.get("msg_name", None)
        info = (name, msg_name)
        msg = kwargs.get("msg", None)
        self.publisher_dict[info].publish(msg)


    # def clear(self):
    #     for puuid, msg_names in self.subscriber_dict.items():
    #         for msg_name, names in msg_names.items():
    #             for name, item_tags in names.items():
    #                 for item_tag, subscriber in item_tags.items():
    #                     self.unsubscribe({"puuid": puuid, "msg_name": msg_name, "name": name, "tag": item_tag})






tbk_manager = TBKManager()

if __name__ == '__main__':
    from Utils.ProtobufManager import ProtobufManager

    pm = ProtobufManager()


    file = "/home/mark/anaconda3/envs/platform_war/lib/python3.11/site-packages/tzcp/ros/std_pb2.py"

    descriptor = b'\n\x12tzcp/ros/std.proto\x12\ntz.ros.std\"\x14\n\x04\x42ool\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x08\"\x14\n\x04\x42yte\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x14\n\x04\x43har\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x17\n\x07\x46loat32\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x02\"\x17\n\x07\x46loat64\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x01\"\x14\n\x04Int8\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x05\"\x15\n\x05Int16\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x05\"\x15\n\x05Int32\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x05\"\x15\n\x05Int64\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x03\"\x15\n\x05UInt8\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x16\n\x06UInt16\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x16\n\x06UInt32\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\r\"\x16\n\x06UInt64\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x04\"\x16\n\x06String\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\t\"7\n\tColorRGBA\x12\t\n\x01r\x18\x01 \x01(\x02\x12\t\n\x01g\x18\x02 \x01(\x02\x12\t\n\x01\x62\x18\x03 \x01(\x02\x12\t\n\x01\x61\x18\x04 \x01(\x02\"\'\n\x08\x44uration\x12\x0c\n\x04secs\x18\x01 \x01(\x05\x12\r\n\x05nsecs\x18\x02 \x01(\x05\"\x07\n\x05\x45mpty\"#\n\x04Time\x12\x0c\n\x04secs\x18\x01 \x01(\x05\x12\r\n\x05nsecs\x18\x02 \x01(\x05\"6\n\x06Header\x12\x0b\n\x03seq\x18\x01 \x01(\r\x12\r\n\x05stamp\x18\x02 \x01(\x02\x12\x10\n\x08\x66rame_id\x18\x03 \x01(\t\"B\n\x13MultiArrayDimension\x12\r\n\x05label\x18\x01 \x01(\t\x12\x0c\n\x04size\x18\x02 \x01(\r\x12\x0e\n\x06stride\x18\x03 \x01(\r\"U\n\x10MultiArrayLayout\x12,\n\x03\x64im\x18\x01 \x03(\x0b\x32\x1f.tz.ros.std.MultiArrayDimension\x12\x13\n\x0b\x64\x61ta_offset\x18\x02 \x01(\r\"L\n\x0e\x42yteMultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x01(\x0c\"N\n\x10UInt32MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\r\"N\n\x10UInt64MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x04\"M\n\x0fInt32MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x05\"M\n\x0fInt64MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x03\"O\n\x11\x46loat32MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x02\"O\n\x11\x46loat64MultiArray\x12,\n\x06layout\x18\x01 \x01(\x0b\x32\x1c.tz.ros.std.MultiArrayLayout\x12\x0c\n\x04\x64\x61ta\x18\x02 \x03(\x01\x62\x06proto3'

    pm.load_from_descriptor(descriptor)
    descriptor2 = b'\n\x18tzcp/ros/ackermann.proto\x12\x10tz.ros.achermann\x1a\x12tzcp/ros/std.proto\"|\n\x0e\x41\x63kermannDrive\x12\x16\n\x0esteering_angle\x18\x01 \x01(\x02\x12\x1f\n\x17steering_angle_velocity\x18\x02 \x01(\x02\x12\r\n\x05speed\x18\x03 \x01(\x02\x12\x14\n\x0c\x61\x63\x63\x65leration\x18\x04 \x01(\x02\x12\x0c\n\x04jerk\x18\x05 \x01(\x02\"l\n\x15\x41\x63kermannDriveStamped\x12\"\n\x06header\x18\x01 \x01(\x0b\x32\x12.tz.ros.std.Header\x12/\n\x05\x64rive\x18\x02 \x01(\x0b\x32 .tz.ros.achermann.AckermannDriveb\x06proto3'

    pm.load_from_descriptor(descriptor2)



    # for k, v in tbk_manager.api_list().items():
    #     print(k, v)



    # 创建 AckermannDrive 消息
    drive_msg = pm.AckermannDrive()
    drive_msg.steering_angle = 0.5  # 转向角度（弧度）
    drive_msg.steering_angle_velocity = 0.1  # 转向角速度
    drive_msg.speed = 2.0  # 速度（米/秒）
    drive_msg.acceleration = 1.0  # 加速度
    drive_msg.jerk = 0.5  # 加加速度
    # 创建带时间戳的 AckermannDriveStamped 消息
    stamped_msg = pm.AckermannDriveStamped()
    # 设置头部信息
    header = pm.Header()
    header.seq = 123  # 秒
    header.stamp = 456000000  # 纳秒
    header.frame_id = "base_link"
    stamped_msg.header.CopyFrom(header)
    # 设置drive信息
    stamped_msg.drive.CopyFrom(drive_msg)
    # 序列化消息（如果需要）
    serialized_msg = stamped_msg.SerializeToString()
    print(serialized_msg)
    # 反序列化消息（如果需要）
    recovered_msg = pm.AckermannDriveStamped()
    recovered_msg.ParseFromString(serialized_msg)


    def f(msg):
        recovered_msg = pm.AckermannDriveStamped()
        try:
            recovered_msg.ParseFromString(msg)
        except Exception as e:
            client_logger.log("ERROR", "error", e)
        print(recovered_msg)

    tbk_manager.publisher(name="test", msg_name="test2")
    tbk_manager.subscriber(name="test", msg_name="test2", callback_func=f)

    i = 0
    while True:
        if i>1:
            break
        i+=1
        tbk_manager.pub(name="test", msg_name="test2", msg=serialized_msg)
        time.sleep(0.01)

