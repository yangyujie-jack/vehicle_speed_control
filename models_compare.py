from models import Vehicle
from linear_models import LinearVehicle
import numpy as np
from params import *
import matplotlib.pyplot as plt


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
    test = tests[0]
    vehicle = Vehicle(i=5, trans_en=False)
    linear_vehicle = LinearVehicle()
    slope = -0*np.pi/180
    T = 30
    t = 0
    vs, lvs = [], []
    Tes = []  # 发动机转矩
    wes = []  # 发动机转速
    wts = []  # 涡轮转速
    Tps = []  # 泵轮转矩
    while t < T:
        vs.append(vehicle.v)
        lvs.append(linear_vehicle.v)

        if test == "const_alpha":
            alpha, Pb = get_u_const_alpha(t)
        elif test == "accel_brake":
            alpha, Pb = get_u_accel_brake(t)

        Tes.append(vehicle.engine.get_torque(alpha))
        wes.append(vehicle.engine.we)
        wts.append(vehicle.torq_conv.wt)
        Tp, _ = vehicle.torq_conv.get_torque()
        Tps.append(Tp)

        vehicle.control(alpha, Pb)
        vehicle.update(slope)
        linear_vehicle.control(alpha, Pb)
        linear_vehicle.update(slope)

        t += dt

    ts = np.arange(len(vs))*dt
    vs = np.array(vs)*3.6
    lvs = np.array(lvs)*3.6

    # 读取CarSim仿真数据
    if test == "const_alpha":
        cs_vs = get_CarSim_v("CarSim_const_alpha_v.txt")
    elif test == "accel_brake":
        cs_vs = get_CarSim_v("CarSim_accel_brake_v.txt")

    # 画图
    # 车速
    plt.figure()
    plt.plot(ts, vs, label="nonlinear")
    plt.plot(ts, lvs, label="linear")
    plt.plot(ts, cs_vs, label="CarSim")
    plt.legend()
    plt.xlabel("t/s")
    plt.ylabel("v/(km/h)")
    plt.title(f"slope = {format(slope*180/np.pi,'.1f')}°")

    # 转矩
    # plt.figure()
    # plt.plot(ts, Tes, label="engine output")
    # # plt.plot(ts, Tps, label="engine input")
    # plt.plot(ts, lTes, label="linear engine output")
    # plt.legend()
    # plt.ylabel("T/Nm")

    # 转速
    # plt.figure()
    # wes = np.array(wes) * 60 / (2 * np.pi)
    # wts = np.array(wts) * 60 / (2 * np.pi)
    # plt.plot(ts, wes, label="engine")
    # plt.plot(ts, wts, label="torque converter output")
    # plt.legend()
    # plt.ylabel("n/(r/min)")

    plt.show()

