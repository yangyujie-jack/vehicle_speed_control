from model import Vehicle, MonitorVehicle, get_slope
from config import Config
from utils import *
import numpy as np
import matplotlib.pyplot as plt


def get_v_des(t, const_v=0):
    a = 1  # 加速度，单位：m/s^2
    t_a = const_v / (3.6 * a)  # 加速时间
    if t < t_a:
        v_des = const_v / t_a * t
    else:
        v_des = const_v
    return v_des/3.6  # m/s


if __name__ == "__main__":
    # speed
    CONST_V = [20, 40, 100]
    const_v = CONST_V[2]

    # slope
    max_slope = 2

    # controller name
    CTRL_NAME = ["PID", "LQR"]
    ctrl_name = CTRL_NAME[0]

    cfg = Config()
    T = 1e4/(const_v/3.6)
    vehicle = MonitorVehicle(Vehicle(cfg))
    controller = get_controller(ctrl_name, cfg)
    t = 0
    v_dess = []
    fr = []
    efr = []
    while t < T:
        print(f"\r{format(t, '.2f')}/{format(T, '.2f')}", end='')
        v_des = get_v_des(t, const_v)
        v_dess.append(v_des)
        alpha, Pb = vehicle.get_control()
        v = vehicle.get_v()
        mode = controller.get_mode(v, v_des, alpha, Pb)
        slope = get_slope(max_slope, vehicle.get_s())
        alpha, Pb = controller.step(mode=mode,
                                    v=v,
                                    v_des=v_des,
                                    alpha=alpha,
                                    Pb=Pb,
                                    n=vehicle.vehicle.engine.n)
        vehicle.control(alpha, Pb)
        vehicle.step(slope)
        t += cfg.const.dt

        # fr.append(vehicle.get_fuel_rate())
        # efr.append(controller.get_estimate_fr(vehicle.get_v(),
        #                                       vehicle.get_control()[0]))

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
    plt.ylim([const_v-20, const_v+20])
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


