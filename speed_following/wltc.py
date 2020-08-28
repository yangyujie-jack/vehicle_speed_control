from model import Vehicle, MonitorVehicle, get_slope, WLTC
from config import Config
from utils import *
import numpy as np
import matplotlib.pyplot as plt


def get_v_des(t):
    k = int(t)
    v_des = ((t - k) * (WLTC[k + 1] - WLTC[k]) + WLTC[k])
    return v_des/3.6


if __name__ == "__main__":
    cfg = Config()
    T = 1800
    slope = 0
    vehicle = MonitorVehicle(Vehicle(cfg))
    controller = get_controller(cfg)
    t = 0
    v_dess = []
    while t < T:
        print(f"\r{format(t, '.2f')}/{format(T, '.2f')}", end='')
        v_des = get_v_des(t)
        v_dess.append(v_des)
        alpha, Pb = vehicle.get_control()
        v = vehicle.get_v()
        mode = controller.get_mode(v, v_des, alpha, Pb)
        alpha, Pb = controller.step(mode, v, v_des, alpha, slope)
        vehicle.control(alpha, Pb)
        vehicle.step(slope)
        t += cfg.const.dt

    v_dess = np.array(v_dess)
    vs = np.array(vehicle.get_vs())
    v_error = v_dess - vs
    v_err_2_int = np.sum(v_error*v_error*cfg.const.dt)
    fuel_csp = vehicle.get_fuel_scp()

    print("")
    print(f"velocity error squared integral {format(v_err_2_int, '.4f')} m^2/s")
    print(f"max velocity error {format(max(abs(v_error)*3.6), '.4f')} km/h")
    print(f"fuel consumption {format(fuel_csp, '.4f')} kg")

    # 画图
    # 期望车速和实际车速
    plt.figure()
    ts = np.arange(len(vs))
    plt.plot(ts, v_dess*3.6, label="v_des")
    plt.plot(ts, vs*3.6, label="v")
    plt.xlabel("t/s")
    plt.ylabel("v/(km/h)")
    plt.legend()
    plt.show()



