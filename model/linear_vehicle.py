import numpy as np
import ctypes
from model.base_vehicle import BaseVehicle


class LinearVehicle(BaseVehicle):
    def __init__(self, config):
        super(LinearVehicle, self).__init__(config)
        self.vehicle_params = np.load(self.config.vehicle.linear_vehicle_params)
        self.veh_alpha_split_points = config.vehicle.veh_alpha_split_points
        self._build_c_funcs()

    def _build_c_funcs(self):
        # update acc
        self._get_acc = self.dll.linear_vehicle_acc
        self._get_acc.restype = ctypes.c_double
        m = self.config.vehicle.m
        Jf = self.config.vehicle.Jf
        Jr = self.config.vehicle.Jr
        r = self.config.vehicle.r
        Kb = self.config.vehicle.Kb
        g = self.config.const.g
        M = m+(Jf+Jr)/r**2
        self.k_Pb = ctypes.c_double(Kb/(r*M))
        self.k_Fi = ctypes.c_double(-m*g/M)

        # step
        self._step = self.dll.vehicle_step
        self._step.restype = ctypes.c_double

    def get_vehicle_params(self, alpha):
        for i, asp in enumerate(self.veh_alpha_split_points):
            if alpha <= asp:
                return self.vehicle_params[i]
        return self.vehicle_params[-1]

    def _update_acc(self, slope):
        k = (ctypes.c_double*3)(*self.get_vehicle_params(self.alpha))
        acc = self._get_acc(k, self.k_Pb, self.k_Fi, ctypes.c_double(self.v),
                            ctypes.c_double(self.alpha), ctypes.c_double(self.Pb),
                            ctypes.c_double(slope))
        self.acc = acc

    def step(self, slope=0):
        self._update_acc(slope)
        v = self._step(ctypes.c_double(self.acc), ctypes.c_double(self.v),
                       ctypes.c_double(self.config.const.dt))
        self.v = v


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    LV = LinearVehicle(cfg)
    t = 0
    while t < 20:
        LV.control(1, 0)
        LV.step()
        t += cfg.const.dt
        print(LV.get_v())
