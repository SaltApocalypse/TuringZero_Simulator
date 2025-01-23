import os
from .TBKManager import tbk_manager

# TODO: 缺少删除逻辑

class ParamData:
    def __init__(self, prefix, name, type="Unknown", info="Custom param", default_value="None"):
        self.tbk_manager = tbk_manager
        self.prefix = prefix
        self.name = name
        self.type = type or self.tbk_manager.get(param=os.path.join(self.prefix, self.name, "__t__"))
        self.info = info or self.tbk_manager.get(param=os.path.join(self.prefix, self.name, "__i__"))
        self.value = default_value or self.tbk_manager.get(param=os.path.join(self.prefix, self.name, "__v__"))
        self.init_param()

    def init_param(self):
        self.tbk_manager.put(param=os.path.join(self.prefix, self.name, "__v__"), value=self._value)
        self.tbk_manager.put(param=os.path.join(self.prefix, self.name, "__t__"), value=self.type)
        self.tbk_manager.put(param=os.path.join(self.prefix, self.name, "__i__"), value=self.info)


    def __call__(self):
        return self._value

    @property
    def value(self):
        self._value = self.tbk_manager.get(param=os.path.join(self.prefix, self.name, "__v__"))
        return self._value

    @value.setter
    def value(self, value):
        self._value = value
        self.tbk_manager.put(param=os.path.join(self.prefix, self.name, "__v__"), value=self._value)
