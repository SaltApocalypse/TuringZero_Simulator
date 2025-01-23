import importlib

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

