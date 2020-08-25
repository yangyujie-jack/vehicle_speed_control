from model import Vehicle, LinearVehicle
import numpy as np
import matplotlib.pyplot as plt
from config import Config


def get_u_accel_brake(t):
    if t < 10:
        alpha = 0.065*t
    elif t < 10.5:
        alpha = -1.3*t+13.65
    else:
        alpha = 0
    if t < 15:
        Pb = 0
    elif t < 15.5:
        Pb = 1.6*t-24
    else:
        Pb = 0.8
    return alpha, Pb


def get_u_const_alpha(t):
    return 0.5, 0


def get_CarSim_v(path):
    cs_vs = []
    with open(path, 'r') as f:
        collecting = False
        for line in f.readlines():
            if not collecting:
                if "Time" in line and "Vx" in line:
                    collecting = True
            else:
                if "Time" in line and "VxTarget" in line:
                    break
                if line == '\n':
                    continue
                [t, v] = line.split('\t')
                v = float(v)
                cs_vs.append(v)
    return cs_vs


if __name__ == "__main__":
    tests = ["const_alpha", "accel_brake"]
    test = tests[1]
    cfg = Config()
    vehicle = Vehicle(cfg)
    linear_vehicle = LinearVehicle(cfg)
    slope = 0
    T = 30
    t = 0
    vs, lvs = [], []
    while t < T:
        print(f"\r{format(t, '.2f')}/{format(T, '.2f')}", end='')
        if test == "const_alpha":
            alpha, Pb = get_u_const_alpha(t)
        elif test == "accel_brake":
            alpha, Pb = get_u_accel_brake(t)
        vehicle.control(alpha, Pb)
        vehicle.step(slope)
        linear_vehicle.control(alpha, Pb)
        linear_vehicle.step(slope)
        vs.append(vehicle.v)
        lvs.append(linear_vehicle.v)
        t += cfg.const.dt
    vs = np.array(vs)*3.6
    lvs = np.array(lvs)*3.6
    ts = np.arange(len(vs))*cfg.const.dt

    # 读取CarSim仿真数据
    if test == "const_alpha":
        cs_vs = get_CarSim_v("data/CarSim_const_alpha_v.txt")
    elif test == "accel_brake":
        cs_vs = get_CarSim_v("data/CarSim_accel_brake_v.txt")

    # 画图
    # 车速
    plt.figure()
    plt.plot(ts, vs, label="nonlinear")
    plt.plot(ts, lvs, label="linear")
    plt.plot(ts, cs_vs, label="CarSim")
    plt.legend()
    plt.xlabel("t/s")
    plt.ylabel("v/(km/h)")
    plt.show()

