import os
import etcd3
from tzcp.tbk import tbk_pb2

from utils.ClientLogManager import client_logger


class EtcdClient:
    def __init__(self):
        self.etcd = self._client()
        self.MESSAGE_PREFIX = "/tbk/ps"
        self.PARAM_PREFIX = "/tbk/params"
        self.pubs = {}

    def _client(self):
        pki_path = os.path.join(os.path.expanduser("~"), ".tbk/etcdadm/pki")
        return etcd3.client(
            host="127.0.0.1",
            port=2379,
            ca_cert=os.path.join(pki_path, "ca.crt"),
            cert_key=os.path.join(pki_path, "etcdctl-etcd-client.key"),
            cert_cert=os.path.join(pki_path, "etcdctl-etcd-client.crt"),
        )

    def get_message_info(self):
        processes = {}
        publishers = {}
        subscribers = {}
        res = self.etcd.get_prefix(self.MESSAGE_PREFIX)
        for r in res:
            key, value = r[1].key.decode(), r[0]
            keys = key[len(self.MESSAGE_PREFIX):].split("/")[1:]
            info = None
            if len(keys) == 1:
                info = tbk_pb2.State()
                info.ParseFromString(value)
                processes[info.uuid] = info
            elif len(keys) == 3:
                if keys[1] == "pubs":
                    info = tbk_pb2.Publisher()
                    info.ParseFromString(value)
                    publishers[info.uuid] = info
                elif keys[1] == "subs":
                    info = tbk_pb2.Subscriber()
                    info.ParseFromString(value)
                    subscribers[info.uuid] = info
            else:
                client_logger.log("ERROR", f"TBKApi: Error: key error:{key}")
        res = {"ps": processes, "pubs": publishers, "subs": subscribers}
        return res

    def get_param_info(self):
        prefix = self.PARAM_PREFIX
        raw_data = self.etcd.get_prefix(prefix)
        data = dict(
            [(r[1].key.decode("utf-8", errors="ignore")[12:], r[0].decode("utf-8", errors="ignore")) for r in raw_data])
        return data

    def list(self, _prefix=None):
        prefix = '/tbk/params/' + (_prefix if _prefix else '')
        res = dict([(r[1].key.decode(), r[0].decode()) for r in self.etcd.get_prefix(prefix)])
        return res

    def get(self, param):
        r = self.etcd.get(f"/tbk/params/{param}")
        return r

    def put(self, param, value):
        r = self.etcd.put(f"/tbk/params/{param}", value)
        # print("r:", r)
        # return True, "OK"

    # def set(self, param, value):
    #     res, v = self.get(param)
    #     if res:
    #         return self.put(param, value)
    #     else:
    #         return False, v

    def delete(self, param):
        r = self.etcd.delete(f"/tbk/params/{param}")
        return r, f"Key \"{param}\" Not found." if r == False else "OK"

    def update_pub_msg_type(self):
        msg_info = self.get_message_info()
        print(msg_info)
        for k, v in msg_info["pubs"].items():
            info = (v.ep_info.name, v.ep_info.msg_name)
            msg_type = v.ep_info.msg_type
            self.pubs[info] = msg_type

    def get_pub_msg_type(self, name, msg_name):
        info = (name, msg_name)
        if info not in self.pubs:
            self.update_pub_msg_type()
        return self.pubs.get(info, "Unknown")




etcd_client = EtcdClient()
