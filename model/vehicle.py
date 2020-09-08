from model.engine import Engine
from model.torque_converter import TorqueConverter
from model.transmission import Transmission
from model.fuel_rate import FuelRate
import numpy as np
from model.base_vehicle import BaseVehicle


class Vehicle(BaseVehicle):
    def __init__(self, config):
        super(Vehicle, self).__init__(config)
        self.trans = Transmission(config)
        self.engine = Engine(config)
        self.fuel_rate = FuelRate(config)
        self.torq_conv = TorqueConverter(config)
        self.m = config.vehicle.m
        self.Jf = config.vehicle.Jf
        self.Jr = config.vehicle.Jr
        self.r = config.vehicle.r
        self.i0 = config.vehicle.i0
        self.eff0 = config.vehicle.eff0
        self.Kb = config.vehicle.Kb
        self.Cd = config.vehicle.Cd
        self.rou = config.vehicle.rou
        self.A = config.vehicle.A
        self.f = config.vehicle.f
        self.g = config.const.g

    def get_fuel_rate(self):
        return self.fuel_rate.get_fuel_rate(self.engine.n, self.alpha)

    def _update_acc(self, slope):
        g = self.config.const.g
        Ff = self.f * self.m * g * np.cos(slope)  # 滚动阻力
        Fi = self.m * g * np.sin(slope)  # 坡度阻力
        Tp, _ = self.torq_conv.get_torque()  # 泵轮转矩
        self.engine.step(self.alpha, Tp)  # 更新发动机转速
        self.torq_conv.np = self.engine.n  # 更新泵轮转速
        _, Tt = self.torq_conv.get_torque()  # 涡轮转矩
        self.trans.change(self.torq_conv.nt, self.alpha)  # 更新档位
        Td = self.i0 * self.eff0 * self.trans.get_torque(Tt)  # 驱动力矩
        Tb = self.Kb * self.Pb  # 制动力矩
        Fw = 0.5 * self.Cd * self.rou * self.A * self.v ** 2  # 空气阻力
        acc = ((Td - Tb) / self.r - Ff - Fw - Fi) / \
              (self.m + (self.Jf + self.Jr) / self.r ** 2)
        self._acc = acc

    def step(self, slope=0):
        self._update_acc(slope)
        self.v += self._acc*self.config.const.dt  # 更新车速
        self.v = max(0, self.v)
        wt = self.v / self.r * self.i0 * self.trans.gears[self.trans.i-1]
        self.torq_conv.nt = wt*60/(2*np.pi)  # 更新涡轮转速

    def get_engine_n(self):
        return self.engine.n


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    veh = Vehicle(cfg)
    t = 0
    while t < 20:
        veh.control(1, 0)
        veh.step()
        t += cfg.const.dt
        print(veh.v)