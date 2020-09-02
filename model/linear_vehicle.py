import numpy as np
from ctypes import *
from model.base_vehicle import BaseVehicle


# class LinearVehicle(Structure):
#     _fields_ = [('dt', c_double),
#                 ('v', c_double),
#                 ('alpha', c_double),
#                 ('Pb', c_double),
#                 ('alpha_bounds', c_double*2),
#                 ('Pb_bounds', c_double*2),
#                 ('d_alpha', c_double),
#                 ('d_Pb', c_double),
#                 ('k_Pb', c_double),
#                 ('vehicle_params', (c_double*3)*3),
#                 ('veh_alpha_split_points', c_double*2)]
#
#     def __init__(self, config):
#         super(LinearVehicle, self).__init__()
#         self.config = config
#         self.dt = c_double(dt)
#         self.v = c_double(0.0)
#         self.alpha = c_double(0.0)
#         self.Pb = c_double(0.0)
#         self.alpha_bounds = (c_double*2)(*alpha_bounds)
#         self.Pb_bounds = (c_double*2)(*Pb_bounds)
#         self.d_alpha = c_double(d_alpha)
#         self.d_Pb = c_double(d_Pb)
#         self.k_Pb = c_double(k_Pb)
#         self.vehicle_params = np.load(self.config.vehicle.linear_vehicle_params).\
#             ctypes.data_as(POINTER((c_double*3)*3)).contents
#         self.veh_alpha_split_points = (c_double*2)(*veh_alpha_split_points)
#         self._build_c_funcs()
#
#     def _build_c_funcs(self):
#         self.dll = CDLL(self.config.project_root + "/Dll1.dll")
#
#     def get_vehicle_params(self, alpha):
#         ari = self.dll.LV_get_alpha_range_index(self, c_double(alpha))
#         return self.vehicle_params[ari][:]
#
#     def step(self, slope=0):
#         self.dll.LV_step(self)
#
#     def control(self, alpha, Pb):
#         self.dll.LV_control(self, c_double(alpha), c_double(Pb))

class LinearVehicle(BaseVehicle):
    def __init__(self, config):
        super(LinearVehicle, self).__init__(config)
        self.vehicle_params = np.load(self._config.vehicle.linear_vehicle_params)

    def get_vehicle_params(self, alpha):
        # TODO C++ function
        ari = self.dll.LV_get_alpha_range_index(self, c_double(alpha))
        return self.vehicle_params[ari][:]

    def update_acc(self, slope):
        # TODO C++ function
        pass

    def step(self, slope=0):
        # TODO C++ function
        pass
