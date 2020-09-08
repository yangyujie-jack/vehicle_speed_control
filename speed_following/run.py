from model import Vehicle, MonitorVehicle, get_slope, WLTC
from config import Config
from utils import *
import numpy as np
import matplotlib.pyplot as plt
from speed_following.target_v import get_v_dess


if __name__ == "__main__":
    # test name
    TEST = ['const_v', 'wltc']
    test = TEST[0]

    # speed
    CONST_V = [20, 40, 100]  # km/h
    const_v = CONST_V[2]

    # controller name
    CTRL_NAME = ["PID", "LQR", "MPC"]
    ctrl_name = CTRL_NAME[2]

    # slope
    max_slope = 0

    cfg = Config()
    if test == 'const_v':
        T = 1e4/(const_v/3.6)
    else:
        T = 1800
    vehicle = MonitorVehicle(Vehicle(cfg))
    controller = get_controller(ctrl_name, cfg)
    t = 0
    v_dess = []
    while t < T:
        print(f"\r{format(t, '.2f')}/{format(T, '.2f')}", end='')

        n_v_des = controller.n_pred if ctrl_name == "MPC" else 1
        next_v_dess = get_v_dess(test, t, cfg.const.dt, n_v_des, const_v=const_v)
        v_dess.append(next_v_dess[0])
        v = vehicle.get_v()
        alpha, Pb = vehicle.get_control()
        alpha, Pb = controller.step(v=v, v_dess=next_v_dess, alpha=alpha,
                                    Pb=Pb, n=vehicle.vehicle.get_engine_n())
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
    # plt.figure()
    # plt.plot(ts, v_dess*3.6, label="v_des")
    # plt.plot(ts, vs*3.6, label="v")
    # plt.xlabel("t/s")
    # plt.ylabel("v/(km/h)")
    # if test == 'const_v':
    #     plt.ylim([const_v-20, const_v+20])
    # plt.legend()
    # plt.show()

    # 节气门开度和制动压力
    alphas, Pbs = vehicle.get_controls()
    plt.subplot(2, 1, 1)
    plt.plot(ts, alphas)
    plt.xlabel("t/s")
    plt.ylabel("alpha")
    plt.subplot(2, 1, 2)
    plt.plot(ts, Pbs)
    plt.xlabel("t/s")
    plt.ylabel("Pb")
    plt.show()


