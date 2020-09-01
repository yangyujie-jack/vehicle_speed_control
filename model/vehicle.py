from model.engine import Engine
from model.torque_converter import TorqueConverter
from model.transmission import Transmission
from model.fuel_rate import FuelRate
import numpy as np
from ctypes import *
from model.params import *


class Vehicle:
    def __init__(self, config):
        self.config = config
        self.v = 0
        self.alpha = 0
        self.Pb = 0
        self.trans = Transmission(self.config)
        self.engine = Engine(self.config)
        self.fuel_rate = FuelRate(self.config)
        self.torq_conv = TorqueConverter(self.config)
        self._build_bound_control()

    def _build_bound_control(self):
        dll = CDLL(self.config.PROJECT_ROOT + "/Dll1.dll")
        self._bound_control = dll.bound_control
        self._bound_control.argtypes = [c_double, c_double,
                                       POINTER(c_double), POINTER(c_double)]

    def control(self, new_alpha, new_Pb):
        old_alpha, old_Pb = self.get_control()
        old_alpha, old_Pb = c_double(old_alpha), c_double(old_Pb)
        new_alpha, new_Pb = c_double(new_alpha), c_double(new_Pb)
        self._bound_control(old_alpha, old_Pb, pointer(new_alpha), pointer(new_Pb))
        self.alpha, self.Pb = new_alpha.value, new_Pb.value

    def get_fuel_rate(self):
        return self.fuel_rate.get_fuel_rate(self.engine.n, self.alpha)

    def get_acc(self, slope):
        Ff = f * m * g * np.cos(slope)  # 滚动阻力
        Fi = m * g * np.sin(slope)  # 坡度阻力
        Tp, _ = self.torq_conv.get_torque()  # 泵轮转矩
        self.engine.step(self.alpha, Tp)  # 更新发动机转速
        self.torq_conv.np = self.engine.n  # 更新泵轮转速
        _, Tt = self.torq_conv.get_torque()  # 涡轮转矩
        self.trans.change(self.torq_conv.nt, self.alpha)  # 更新档位
        Td = i0 * eff0 * self.trans.get_torque(Tt)  # 驱动力矩
        Tb = Kb * self.Pb  # 制动力矩
        Fw = 0.5 * Cd * rou * A * self.v ** 2  # 空气阻力
        acc = ((Td - Tb) / r - Ff - Fw - Fi) / (m + (Jf + Jr) / r ** 2)
        return acc

    def step(self, slope=0):
        acc = self.get_acc(slope)
        self.v += acc*self.config.const.dt  # 更新车速
        self.v = max(0, self.v)
        wt = self.v/r*i0*self.trans.gears[self.trans.i-1]
        self.torq_conv.nt = wt*60/(2*np.pi)  # 更新涡轮转速

    def get_control(self):
        return self.alpha, self.Pb

    def set_v(self, v):
        self.v = v

    def set_control(self, alpha, Pb):
        self.alpha = alpha
        self.Pb = Pb


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    veh = Vehicle(cfg)
    for _ in range(1000):
        veh.control(1, 0)
        veh.step()
        print(veh.v)