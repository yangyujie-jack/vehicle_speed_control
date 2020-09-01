import numpy as np
from copy import deepcopy
from ctypes import *


class LinearVehicle:
    def __init__(self, config):
        self.config = config
        self.vehicle_params = np.load(self.config.vehicle.linear_vehicle_params)  # (3,3)
        self.fuel_rate_params = np.load(self.config.vehicle.fuel_rate_params)  # (2,3,6)
        self.m = self.config.vehicle.m
        self.Jf = self.config.vehicle.Jf
        self.Jr = self.config.vehicle.Jr
        self.r = self.config.vehicle.r
        self.Kb = self.config.vehicle.Kb
        self.k_Pb = -self.Kb/self.r/(self.m+(self.Jf+self.Jr)/self.r**2)
        self.v = self.config.vehicle.v  # m/s
        self.alpha = self.config.vehicle.alpha
        self.Pb = self.config.vehicle.Pb
        self._build_c_funcs()

    def _build_c_funcs(self):
        dll = CDLL(self.config.PROJECT_ROOT + "/Dll1.dll")
        self._bound_control = dll.bound_control
        self._bound_control.argtypes = [c_double, c_double,
                                       POINTER(c_double), POINTER(c_double)]
        self._update_v = dll.update_v
        self._update_v.argtypes = [POINTER(c_double), c_double, c_double, c_double]
        self._update_v.restype = c_double

    def control(self, new_alpha, new_Pb):
        old_alpha, old_Pb = self.get_control()
        old_alpha, old_Pb = c_double(old_alpha), c_double(old_Pb)
        new_alpha, new_Pb = c_double(new_alpha), c_double(new_Pb)
        self._bound_control(old_alpha, old_Pb, pointer(new_alpha), pointer(new_Pb))
        self.alpha, self.Pb = new_alpha.value, new_Pb.value

    def update_v(self, ks, alpha, Pb):
        ks = (c_double*4)(*ks)
        old_v = c_double(self.v)
        alpha, Pb = c_double(alpha), c_double(Pb)
        self.v = self._update_v(ks, old_v, alpha, Pb)

    def get_vehicle_param(self, slope, alpha):
        g = self.config.const.g
        Fi = self.m * g * np.sin(slope)  # 坡度阻力
        # 找节气门开度的区间
        alpha_range_i = 0
        for asp in self.config.vehicle.veh_alpha_split_points:
            if alpha > asp:
                alpha_range_i += 1
            else:
                break
        param = deepcopy(self.vehicle_params[alpha_range_i])
        param[2] -= Fi/(self.m+(self.Jf+self.Jr)/self.r**2)
        return param

    def get_fuel_rate_param(self, alpha, v):
        # 找节气门开度的区间
        alpha_range_i = 0
        for i, asp in enumerate(self.config.vehicle.fr_alpha_split_points):
            if alpha > asp:
                alpha_range_i += 1
            else:
                break
        # 找车速的区间
        v_range_i = 0
        for i, vsp in enumerate(self.config.vehicle.fr_v_split_points):
            if v > vsp:
                v_range_i += 1
            else:
                break
        param = self.fuel_rate_params[alpha_range_i, v_range_i]
        return param

    def step(self, slope=0):
        vp = self.get_vehicle_param(slope, self.alpha)
        ks = [vp[0], vp[1], self.k_Pb, vp[2]]
        self.update_v(ks, self.alpha, self.Pb)

    def set_v(self, v):
        self.v = v

    def get_v(self):
        return self.v

    def set_control(self, alpha, Pb):
        self.alpha = alpha
        self.Pb = Pb

    def get_control(self):
        return self.alpha, self.Pb
