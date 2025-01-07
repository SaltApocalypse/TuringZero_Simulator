import os

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


def ensure_import(func):
    """装饰器，用于确保调用函数前已经导入相关库"""

    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        if not self.is_import:
            self.new()
            self.is_import = True
        return func(self, *args, **kwargs)

    return wrapper


class TBKManager:
    def __init__(self):
        # 动态导入库
        self.etcd3 = None
        self.tbkpb = None
        self.tbkpy = None

        self.param_tree = None
        self._param_data = None
        self._message_data = None
        self.etcd = None
        self.MESSAGE_PREFIX = "/tbk/ps"
        self.PARAM_PREFIX = "/tbk/params"

        self.callback_dict = {}
        self.subscriber_dict = {}

        self.is_import = False

    def new(self):
        try:
            import etcd3
            import tzcp.tbk.tbk_pb2 as tbkpb
            from tbkpy import _core as tbkpy

            self.etcd3 = etcd3
            self.tbkpb = tbkpb
            self.tbkpy = tbkpy

            tbkpy.init("simulator")
            self.etcd = self._client()
            self.is_import = True
        except Exception as e:
            client_logger.log("ERROR", "Import tbk error!", e)

    def _client(self):
        pki_path = os.path.join(os.path.expanduser("~"), ".tbk/etcdadm/pki")
        return self.etcd3.client(
            host="127.0.0.1",
            port=2379,
            ca_cert=os.path.join(pki_path, "ca.crt"),
            cert_key=os.path.join(pki_path, "etcdctl-etcd-client.key"),
            cert_cert=os.path.join(pki_path, "etcdctl-etcd-client.crt"),
        )

    @ensure_import
    def get_original_param(self, _prefix=None):
        prefix = self.PARAM_PREFIX + (_prefix if _prefix else "")
        raw_data = self.etcd.get_prefix(prefix)
        data = dict([(r[1].key.decode("utf-8", errors="ignore")[12:], r[0].decode("utf-8", errors="ignore")) for r in raw_data])
        return data

    @ensure_import
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
    @ensure_import
    def param_data(self):
        self._param_data = self.get_param()
        return self._param_data

    @ensure_import
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
    @ensure_import
    def message_data(self):
        self._message_data = self.get_message()
        return self._message_data

    @property
    @ensure_import
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

    @ensure_import
    def unsubscribe(self, info: dict, is_del_msg=False):
        puuid = info["puuid"]
        name = info["name"]
        msg_name = info["msg_name"]
        tag = info["tag"]
        if tag in self.callback_dict.get(puuid, {}).get(msg_name, {}).get(name, {}):
            del self.callback_dict[puuid][msg_name][name][tag]
        if len(self.callback_dict[puuid][msg_name][name]) < 1 or is_del_msg:
            del self.subscriber_dict[puuid][msg_name][name]
            del self.callback_dict[puuid][msg_name][name]

    # def remove_subscriber(self, puuid: str, msg_name: str, name: str, item_tag: str):
    #     """
    #     删除嵌套字典中的订阅者。
    #     """
    #     # 检查是否存在对应的订阅者
    #     if puuid in self.subscriber_data and msg_name in self.subscriber_data[puuid]:
    #         if name in self.subscriber_data[puuid][msg_name] and item_tag in self.subscriber_data[puuid][msg_name][name]:
    #             del self.subscriber_data[puuid][msg_name][name][item_tag]  # 删除具体的 item_tag
    #             # 如果 name 下没有任何订阅者，则删除 name
    #             if not self.subscriber_data[puuid][msg_name][name]:
    #                 del self.subscriber_data[puuid][msg_name][name]
    #             # 如果 msg_name 下没有任何订阅者，则删除 msg_name
    #             if not self.subscriber_data[puuid][msg_name]:
    #                 del self.subscriber_data[puuid][msg_name]
    #             # 如果 puuid 下没有任何消息，则删除 puuid
    #             if not self.subscriber_data[puuid]:
    #                 del self.subscriber_data[puuid]
    #             return True
    #     return False  # 如果订阅者不存在，则返回 False

    @ensure_import
    def is_subscribed(self, info: dict) -> bool:
        return info["name"] in self.subscriber_dict.get(info["puuid"], {}).get(info["msg_name"], {})

    @ensure_import
    def callback_manager(self, msg, info):
        puuid = info["puuid"]
        name = info["name"]
        msg_name = info["msg_name"]
        for tag in self.callback_dict.get(puuid, {}).get(msg_name, {}).get(name, {}):
            callback = self.callback_dict.get(puuid, {}).get(msg_name, {}).get(name, {}).get(tag)
            if callback:
                callback(msg)

    @ensure_import
    def subscriber(self, info: dict, callback):
        puuid = info["puuid"]
        name = info["name"]
        msg_name = info["msg_name"]
        tag = info["tag"]
        # if "user_data" in info:
        #     user_data = info["user_data"]
        self.callback_dict.setdefault(puuid, {}).setdefault(msg_name, {}).setdefault(name, {})[tag] = callback

        if self.subscriber_dict.get(puuid, {}).get(msg_name, {}).get(name) is not None:
            # 如果self.subscriber_dict[puuid][msg_name][name] 如果已经订阅则退出
            return
        client_logger.log("INFO", f"Add new subscriber({puuid}, {msg_name}, {name})")
        self.subscriber_dict.setdefault(puuid, {}).setdefault(msg_name, {})[name] = self.tbkpy.Subscriber(
            # puuid, #这个属性tbk内还没开出接口
            name,
            msg_name,
            lambda msg: self.callback_manager(msg, info),
        )

    @ensure_import
    def publisher(self, info: dict):
        ep_info = self.tbkpy.EPInfo()
        ep_info.name = info["name"]
        ep_info.msg_name = info["msg_name"]
        ep_info.msg_type = info["msg_type"]
        return self.tbkpy.Publisher(ep_info)

    @ensure_import
    def clear(self):
        for puuid, msg_names in self.subscriber_dict.items():
            for msg_name, names in msg_names.items():
                for name, item_tags in names.items():
                    for item_tag, subscriber in item_tags.items():
                        self.unsubscribe({"puuid": puuid, "msg_name": msg_name, "name": name, "tag": item_tag})


tbk_manager = TBKManager()
