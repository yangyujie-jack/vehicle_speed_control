import numpy as np
from model import Vehicle
from copy import deepcopy

class LinearVehicle(Vehicle):
    def __init__(self, config):
        super(LinearVehicle, self).__init__(config)
        self.vehicle_params = np.load(self.config.vehicle.linear_vehicle_params)
        self.fuel_rate_params = np.load(self.config.vehicle.fuel_rate_params)

    def get_vehicle_param(self, slope, alpha):
        """
        获取当前的线性模型参数
        :param slope: 坡度, rad
        :param alpha: 节气门开度
        :return: 线性模型参数
        """
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
        """
        获取当前的线性模型参数
        :param alpha: 节气门开度
        :return:
        """
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
        Kvs = self.get_vehicle_param(slope, self.alpha)
        acc = Kvs[0]*self.v+Kvs[1]*self.alpha+Kvs[2]-self.Kb*self.Pb/self.r/\
                (self.m+(self.Jf+self.Jr)/self.r**2)
        self.v += acc*self.config.const.dt
        self.v = max(0, self.v)


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    veh = LinearVehicle(cfg)
    for _ in range(1000):
        veh.control(1, 0)
        veh.step()
        print(veh.v)