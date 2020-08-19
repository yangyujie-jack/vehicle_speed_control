from model.engine import Engine
from model.torque_converter import TorqueConverter
from model.transmission import Transmission
from model.fuel_rate import FuelRate
import numpy as np


class Vehicle:
    """
    整车模型
    """
    def __init__(self, config):
        self.config = config
        self.v = self.config.vehicle.v  # m/s
        self.alpha = self.config.vehicle.alpha
        self.Pb = self.config.vehicle.Pb
        self.trans = Transmission(self.config)
        self.engine = Engine(self.config)
        self.fuel_rate = FuelRate(self.config)
        self.torq_conv = TorqueConverter(self.config)
        self.f = self.config.vehicle.f
        self.m = self.config.vehicle.m
        self.r = self.config.vehicle.r
        self.Cd = self.config.vehicle.Cd
        self.rou = self.config.vehicle.rou
        self.A = self.config.vehicle.A
        self.Jf = self.config.vehicle.Jf
        self.Jr = self.config.vehicle.Jr
        self.i0 = self.config.vehicle.i0
        self.eff0 = self.config.vehicle.eff0
        self.Kb = self.config.vehicle.Kb

    def control(self, alpha, Pb):
        """
        :param alpha: 节气门开度
        :param Pb: 制动压力
        :return: 限制后的控制量
        """
        alpha_hi = min(self.config.vehicle.alpha_bounds[1],
                       self.alpha+self.config.vehicle.d_alpha)
        alpha_lo = max(self.config.vehicle.alpha_bounds[0],
                       self.alpha-self.config.vehicle.d_alpha)
        Pb_hi = min(self.config.vehicle.Pb_bounds[1],
                    self.Pb+self.config.vehicle.d_Pb)
        Pb_lo = max(self.config.vehicle.Pb_bounds[0],
                    self.Pb-self.config.vehicle.d_Pb)
        self.alpha = np.clip(alpha, alpha_lo, alpha_hi)
        self.Pb = np.clip(Pb, Pb_lo, Pb_hi)

    def get_fuel_rate(self):
        return self.fuel_rate.get_fuel_rate(self.engine.n, self.alpha)

    def get_acc(self, slope):
        """
        计算加速度
        :param slope: 坡度, rad
        :return: 加速度, m/s^2
        """
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
        acc = ((Td - Tb) / self.r - Ff - Fw - Fi)/\
              (self.m + (self.Jf + self.Jr) / self.r ** 2)
        return acc

    def step(self, slope=0):
        """
        更新汽车状态
        :param slope: 坡度，rad
        :return:
        """
        acc = self.get_acc(slope)
        self.v += acc*self.config.const.dt  # 更新车速
        self.v = max(0, self.v)
        wt = self.v/self.r*self.i0*self.trans.gears[self.trans.i-1]
        self.torq_conv.nt = wt*60/(2*np.pi)  # 更新涡轮转速

    def get_control(self):
        return self.alpha, self.Pb


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    veh = Vehicle(cfg)
    for _ in range(1000):
        veh.control(1, 0)
        veh.step()
        print(veh.v)