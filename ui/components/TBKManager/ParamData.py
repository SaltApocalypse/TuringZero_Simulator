import os


class ParamData:
    def __init__(self, prefix, name, tbk_manager, type="Unknown", info="Custom param"):
        self.tbk_manager = tbk_manager
        self.prefix = prefix
        self.name = name
        self.type = type
        self.info = info
        self._value = self.tbk_manager.get(param=os.path.join(self.prefix, self.name, "__v__"))
        self.init_param()

    def init_param(self):
        if self._value == "None":
            self.tbk_manager.put(param=os.path.join(self.prefix, self.name, "__v__"), value=self._value)
            self.tbk_manager.put(param=os.path.join(self.prefix, self.name, "__t__"), value=self.type)
            self.tbk_manager.put(param=os.path.join(self.prefix, self.name, "__i__"), value=self.info)

        self.type = self.tbk_manager.get(param=os.path.join(self.prefix, self.name, "__t__"))
        self.info = self.tbk_manager.get(param=os.path.join(self.prefix, self.name, "__i__"))

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
