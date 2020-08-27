from model import Vehicle, LinearVehicle
import numpy as np
import matplotlib.pyplot as plt
from config import Config
from model_compare.control import get_control
from model_compare.utils import *


if __name__ == "__main__":
    cfg = Config()
    for data_file in cfg.carsim.files:
        test_name = data_file[:-4]
        v0, T = get_config(test_name)
        vehicle = Vehicle(cfg)
        linear_vehicle = LinearVehicle(cfg)
        vehicle.set_v(v0)
        linear_vehicle.set_v(v0)
        slope = 0
        t = 0
        vs, lvs = [], []
        while t < T:
            alpha, Pb = get_control(test_name, t)
            vehicle.control(alpha, Pb)
            vehicle.step(slope)
            linear_vehicle.control(alpha, Pb)
            linear_vehicle.step(slope)
            vs.append(vehicle.v)
            lvs.append(linear_vehicle.v)
            t += cfg.const.dt
        vs = np.array(vs)*3.6
        lvs = np.array(lvs)*3.6

        # 读取CarSim仿真数据
        cs_vs = load_carsim_data(cfg.carsim.path+data_file)

        # 画图
        # 车速
        plt.figure()
        plt.plot(np.arange(len(vs))*cfg.const.dt, vs, label="nonlinear")
        plt.plot(np.arange(len(lvs))*cfg.const.dt, lvs, label="linear")
        plt.plot(np.arange(len(cs_vs))*cfg.const.dt, cs_vs, label="CarSim")
        plt.legend()
        plt.xlabel("t/s")
        plt.ylabel("v/(km/h)")
        plt.title(test_name)
        plt.show()

