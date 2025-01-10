from ui.components.TBKManager.Utils.RemoteManager import RemoteManager


class TBKParamManager(RemoteManager):
    def __init__(self):
        pass

    # 测试用
    def list(self, _prefix=None, **kwargs):
        prefix = '/tbk/params/' + (_prefix if _prefix else '')
        res = dict([(r[1].key.decode(), r[0].decode()) for r in self.etcd.get_prefix(prefix)])
        return res

    def get(self, key, **kwargs):
        r = self.etcd.get(f"/tbk/params/{key}")
        return (True, r[0])

    def put(self, key, value, **kwargs):
        r = self.etcd.put(f"/tbk/params/{key}", value)
        return True, "OK"

    def set(self, key, value, **kwargs):
        res, v = self.get(key)
        if res:
            return self.put(key, value)
        else:
            return False, v

    def delete(self, key, **kwargs):
        r = self.etcd.delete(f"/tbk/params/{key}")
        return r, f"Key \"{key}\" Not found." if r == False else "OK"