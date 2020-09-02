from ctypes import *


class BaseVehicle:
    def __init__(self, config):
        self._config = config
        self._v = 0.0
        self._acc = 0.0
        self._alpha = 0.0
        self._Pb = 0.0
        self._build_c_funcs()

    def _build_c_funcs(self):
        self.dll = CDLL(self._config.project_root + "/Dll1.dll")
        self._bound_control = self.dll.bound_control
        self._bound_control.argtypes = [c_double, c_double,
                                        POINTER(c_double), POINTER(c_double)]

    def control(self, new_alpha, new_Pb):
        # TODO rewrite C++ function
        old_alpha, old_Pb = self.get_control()
        old_alpha, old_Pb = c_double(old_alpha), c_double(old_Pb)
        new_alpha, new_Pb = c_double(new_alpha), c_double(new_Pb)
        self._bound_control(old_alpha, old_Pb, pointer(new_alpha), pointer(new_Pb))
        self._alpha, self._Pb = new_alpha.value, new_Pb.value

    def get_v(self):
        return self._v

    def set_v(self, v):
        self._v = v

    def get_control(self):
        return self._alpha, self._Pb

    def set_control(self, alpha, Pb):
        self._alpha = alpha
        self._Pb = Pb

    def update_acc(self, slope):
        # implemented by child class
        pass

    def step(self, slope):
        # implemented by child class
        pass
