from model import Vehicle, MonitorVehicle, WLTC
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
        alpha, Pb = controller.step(mode=mode,
                                    v=v,
                                    v_des=v_des,
                                    alpha=alpha,
                                    Pb=Pb,
                                    n=vehicle.vehicle.engine.n)
        vehicle.control(alpha, Pb)
        vehicle.step(slope)
        t += cfg.const.dt

    v_dess = np.array(v_dess)
    vs = np.array(vehicle.get_vs())
    v_error = v_dess - vs
    fuel_csp = vehicle.get_fuel_scp()

    print_res(v_error, fuel_csp)

    # 画图
    # 期望车速和实际车速
    plt.figure()
    ts = np.arange(len(vs))*cfg.const.dt
    plt.plot(ts, v_dess*3.6, label="v_des")
    plt.plot(ts, vs*3.6, label="v")
    plt.xlabel("t/s")
    plt.ylabel("v/(km/h)")
    plt.legend()
    plt.show()

    # 节气门开度和制动压力
    alphas, Pbs = vehicle.get_alphas_Pbs()
    plt.subplot(2, 1, 1)
    plt.plot(ts, alphas)
    plt.xlabel("t/s")
    plt.ylabel("alpha")
    plt.subplot(2, 1, 2)
    plt.plot(ts, Pbs)
    plt.xlabel("t/s")
    plt.ylabel("Pb")
    plt.show()




