import ctypes


class BaseVehicle:
    def __init__(self, config):
        self.config = config
        self.v = 0.0
        self.acc = 0.0
        self.alpha = 0.0
        self.Pb = 0.0
        self._build_base_c_funcs()

    def _build_base_c_funcs(self):
        self.dll = ctypes.CDLL(self.config.project_root + "/model/vehicle.dll")
        self._control = self.dll.base_vehicle_control
        self.alpha_bounds = (ctypes.c_double*2)(*self.config.vehicle.alpha_bounds)
        self.Pb_bounds = (ctypes.c_double*2)(*self.config.vehicle.Pb_bounds)
        self.d_alpha = ctypes.c_double(self.config.vehicle.d_alpha)
        self.d_Pb = ctypes.c_double(self.config.vehicle.d_Pb)

    def control(self, new_alpha, new_Pb):
        old_alpha = ctypes.c_double(self.alpha)
        old_Pb = ctypes.c_double(self.Pb)
        new_alpha = ctypes.c_double(new_alpha)
        new_Pb = ctypes.c_double(new_Pb)
        self._control(old_alpha, old_Pb, ctypes.byref(new_alpha), ctypes.byref(new_Pb),
                      self.alpha_bounds, self.Pb_bounds,self.d_alpha, self.d_Pb)
        self.alpha = new_alpha.value
        self.Pb = new_Pb.value

    def get_v(self):
        return self.v

    def set_v(self, v):
        self.v = v

    def get_control(self):
        return self.alpha, self.Pb

    def set_control(self, alpha, Pb):
        self.alpha = alpha
        self.Pb = Pb

    def _update_acc(self, slope):
        raise NotImplementedError

    def step(self, slope):
        raise NotImplementedError


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    BV = BaseVehicle(cfg)
    BV.control(1, 0)
    print(BV.get_control())
