from model import Vehicle, LinearVehicle, MonitorVehicle, get_slope
from config import Config
from utils import *
import numpy as np
import matplotlib.pyplot as plt


def get_v_des(t, const_v):
    const_v /= 3.6
    a = 1  # 加速度，单位：m/s^2
    t_a = const_v / a  # 加速时间
    if t < t_a:
        v_des = const_v / t_a * t
    else:
        v_des = const_v
    return v_des  # m/s


if __name__ == "__main__":
    # speed
    CONST_V = [20, 40, 100]  # km/h
    const_v = CONST_V[2]

    # slope
    max_slope = 0

    cfg = Config()
    T = 1e3/(const_v/3.6)
    # vehicle = MonitorVehicle(Vehicle(cfg))
    vehicle = MonitorVehicle(LinearVehicle(cfg))
    controller = get_controller("MPC", cfg)
    t = 0
    v_dess = []
    while t < T:
        # print(f"\r{format(t, '.2f')}/{format(T, '.2f')}", end='')

        next_v_dess = []
        for i in range(controller.n_pred):
            v_des = get_v_des(t+i*cfg.const.dt, const_v)
            next_v_dess.append(v_des)

        v_dess.append(get_v_des(t, const_v))
        v = vehicle.get_v()
        alpha, Pb = vehicle.get_control()
        alpha, Pb = controller.step(mode=1, v=v, v_des=next_v_dess, alpha=alpha, Pb=Pb)
        vehicle.control(alpha, Pb)
        slope = get_slope(max_slope, vehicle.get_s())
        vehicle.step(slope)
        t += cfg.const.dt

    v_dess = np.array(v_dess)
    vs = np.array(vehicle.get_vs())
    v_error = v_dess - vs
    fuel_csp = vehicle.get_fuel_scp()
    ts = np.arange(len(vs))*cfg.const.dt

    print_res(v_error, fuel_csp)

    # 画图
    # 期望车速和实际车速
    plt.figure()
    plt.plot(ts, v_dess*3.6, label="v_des")
    plt.plot(ts, vs*3.6, label="v")
    plt.xlabel("t/s")
    plt.ylabel("v/(km/h)")
    # plt.ylim([const_v-20, const_v+20])
    plt.legend()
    plt.show()

    # plt.figure()
    # plt.plot(np.arange(len(fr)), fr, label="real fr")
    # plt.plot(np.arange(len(efr)), efr, label="estimated fr")
    # plt.legend()
    # plt.show()

    # 节气门开度和制动压力
    # alphas, Pbs = vehicle.get_alphas_Pbs()
    # plt.subplot(2, 1, 1)
    # plt.plot(ts, alphas)
    # plt.xlabel("t/s")
    # plt.ylabel("alpha")
    # plt.subplot(2, 1, 2)
    # plt.plot(ts, Pbs)
    # plt.xlabel("t/s")
    # plt.ylabel("Pb")
    # plt.show()


