from model import Vehicle, MonitorVehicle, get_slope, WLTC
from config import Config
from controller import get_controller
import numpy as np
import matplotlib.pyplot as plt


def get_v_des(t, sim_name, const_v=0):
    """
    获取当前期望车速
    :param t: 当前时刻
    :return:
    """
    if sim_name == "wltc":
        k = int(t)
        v_des = ((t - k) * (WLTC[k + 1] - WLTC[k]) + WLTC[k])
    elif sim_name == "const_v":
        a = 1  # 加速度，单位：m/s^2
        t_a = const_v/(3.6*a)  # 加速时间
        if t < t_a:
            v_des = const_v/t_a*t
        else:
            v_des = const_v
    else:
        raise ValueError(f"Invalid simulation name '{sim_name}'")
    return v_des/3.6  # m/s


if __name__ == "__main__":
    print("Initializing...")
    cfg = Config()
    test = cfg.simulation.simulation_name
    save = False
    vehicle = MonitorVehicle(Vehicle(cfg))
    controller = get_controller(cfg)
    alpha, Pb = 0, 0
    t = 0
    v_dess = []
    while t < cfg.simulation.simulation_time:
        print(f"\r{format(t, '.2f')}/"
              f"{format(cfg.simulation.simulation_time, '.2f')}", end='')
        v_des = get_v_des(t, cfg.simulation.simulation_name,
                          cfg.simulation.const_v)
        v = vehicle.get_v()
        v_dess.append(v_des)
        mode = controller.get_mode(v, v_des, alpha, Pb)
        slope = get_slope(vehicle.get_s())
        alpha, Pb = controller.step(mode, v, v_des, alpha, slope)
        vehicle.control(alpha, Pb)
        alpha, Pb = vehicle.get_control()
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

    # 保存控制量
    # if save:
    #     alphas = np.array(alphas)
    #     Pbs = np.array(Pbs)
    #     np.save(control_data_dir+"alpha_v100", alphas)
    #     np.save(control_data_dir+"Pb_v100", Pbs)

    # 画图
    # 期望车速和实际车速
    # plt.figure()
    # plt.plot(ts, v_dess, label="v_des")
    # plt.plot(ts, vs, label="v")
    # plt.xlabel("t/s")
    # plt.ylabel("v/(km/h)")
    # plt.ylim([const_v-20, const_v+20])
    # plt.legend()
    # plt.show()

    # 车速误差
    # plt.figure()
    # plt.plot(ts, v_error)
    # plt.xlabel("t/s")
    # plt.ylabel("velocity error/(km/h)")

    # 节气门开度和制动压力
    # plt.figure()
    # plt.subplot(2,1,1)
    # plt.plot(ts, alphas)
    # plt.xlabel("t/s")
    # plt.ylabel("alpha")
    # plt.subplot(2,1,2)
    # plt.plot(ts, Pbs)
    # plt.xlabel("t/s")
    # plt.ylabel("Pb/MPa")
    # plt.show()



