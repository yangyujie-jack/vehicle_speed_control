from model.engine import Engine
from model.torque_converter import TorqueConverter
from model.transmission import Transmission
from model.fuel_rate import FuelRate
import numpy as np
from ctypes import *
from model.params import *
from model.base_vehicle import BaseVehicle


class Vehicle(BaseVehicle):
    def __init__(self, config):
        super(Vehicle, self).__init__(config)
        self._trans = Transmission(config)
        self._engine = Engine(config)
        self._fuel_rate = FuelRate(config)
        self._torq_conv = TorqueConverter(config)

    def get_fuel_rate(self):
        return self._fuel_rate.get_fuel_rate(self._engine.n, self._alpha)

    def update_acc(self, slope):
        # TODO C++ function
        Ff = f * m * g * np.cos(slope)  # 滚动阻力
        Fi = m * g * np.sin(slope)  # 坡度阻力
        Tp, _ = self._torq_conv.get_torque()  # 泵轮转矩
        self._engine.step(self._alpha, Tp)  # 更新发动机转速
        self._torq_conv.np = self._engine.n  # 更新泵轮转速
        _, Tt = self._torq_conv.get_torque()  # 涡轮转矩
        self._trans.change(self._torq_conv.nt, self._alpha)  # 更新档位
        Td = i0 * eff0 * self._trans.get_torque(Tt)  # 驱动力矩
        Tb = Kb * self._Pb  # 制动力矩
        Fw = 0.5 * Cd * rou * A * self.v ** 2  # 空气阻力
        acc = ((Td - Tb) / r - Ff - Fw - Fi) / (m + (Jf + Jr) / r ** 2)
        self._acc = acc

    def step(self, slope=0):
        # TODO C++ function
        self.update_acc(slope)
        self.v += self._acc*self._config.const.dt  # 更新车速
        self.v = max(0, self.v)
        wt = self.v/r*i0*self._trans.gears[self._trans.i-1]
        self._torq_conv.nt = wt*60/(2*np.pi)  # 更新涡轮转速


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    veh = Vehicle(cfg)
    for _ in range(1000):
        veh.control(1, 0)
        veh.step()
        print(veh.v)