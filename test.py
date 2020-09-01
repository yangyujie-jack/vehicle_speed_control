import numpy as np
from ctypes import *
from config import Config
from model.params import *


class LinearVehicle(Structure):
    _fields_ = [('dt', c_double),
                ('v', c_double),
                ('alpha', c_double),
                ('Pb', c_double),
                ('alpha_bounds', c_double*2),
                ('Pb_bounds', c_double*2),
                ('d_alpha', c_double),
                ('d_Pb', c_double),
                ('k_Pb', c_double),
                ('vehicle_params', (c_double*3)*3),
                ('veh_alpha_split_points', c_double*2)]

    def __init__(self, config):
        super(LinearVehicle, self).__init__()
        self.config = config
        self.dt = dt
        self.v = 0
        self.alpha, self.Pb = 0, 0
        self.alpha_bounds = (c_double*2)(*alpha_bounds)
        self.Pb_bounds = (c_double*2)(*Pb_bounds)
        self.d_alpha, self.d_Pb = d_alpha, d_Pb
        self.k_Pb = k_Pb
        self.vehicle_params = np.load(self.config.vehicle.linear_vehicle_params).\
            ctypes.data_as(POINTER((c_double*3)*3)).contents
        self.veh_alpha_split_points = (c_double*2)(*veh_alpha_split_points)
        self._build_c_funcs()

    def _build_c_funcs(self):
        self.dll = CDLL("./Dll1.dll")

    def get_vehicle_params(self, alpha):
        ari = self.dll.LV_get_alpha_range_index(self, c_double(alpha))
        return self.vehicle_params[ari][:]

    def step(self):
        self.dll.LV_step(self)

    def control(self, alpha, Pb):
        self.dll.LV_control(self, c_double(alpha), c_double(Pb))

    def set_v(self, v):
        pass

    def get_v(self):
        return self.v

    def set_control(self, alpha, Pb):
        self.alpha = float(alpha)
        self.Pb = float(Pb)

    def get_control(self):
        return self.alpha, self.Pb


cfg = Config()
lv = LinearVehicle(cfg)
lv.control(1, 0)
