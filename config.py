from easydict import EasyDict as edict
import os


class Config:
    def __init__(self):
        PROJECT_ROOT = os.path.dirname(os.path.realpath(__file__))

        # constants
        self.const = edict()
        self.const.dt = 0.025  # 仿真步长
        # self.const.dt = 0.1
        self.const.g = 9.8

        # engine
        self.engine = edict()
        self.engine.data_path = os.path.join(PROJECT_ROOT, "model/data/engine.csv")
        self.engine.n_idle = 750  # 怠速转速
        self.engine.Je = 0.2  # 飞轮转动惯量

        # fuel rate
        self.fuel_rate = edict()
        self.fuel_rate.data_path = os.path.join(PROJECT_ROOT, "model/data/fuel_rate.csv")

        # torque converter
        self.torque_converter = edict()
        self.torque_converter.K_data_path = os.path.join(PROJECT_ROOT,
                                                         "model/data/torque_converter_K.csv")
        self.torque_converter.Kp_data_path = os.path.join(PROJECT_ROOT,
                                                          "model/data/torque_converter_Kp.csv")
        self.torque_converter.np = self.engine.n_idle  # 泵轮转速
        self.torque_converter.nt = 0  # 涡轮转速

        # transmission
        self.transmission = edict()
        self.transmission.enable = False  # 是否允许换挡
        self.transmission.i = 5  # 档位，1~7
        self.transmission.gears = [4.38, 2.86, 1.92, 1.37, 1.00, 0.82, 0.70]  # 传动比
        self.transmission.effs = [0.92, 0.92, 0.95, 0.95, 0.98, 0.99, 0.99]  # 效率
        self.transmission.up_shift_data_path = os.path.join(PROJECT_ROOT,
                                                "model/data/transmission_up_shift.csv")  # 向上换挡规律
        self.transmission.down_shift_data_path = os.path.join(PROJECT_ROOT,
                                                "model/data/transmission_down_shift.csv")  # 向下换挡规律

        # vehicle
        self.vehicle = edict()
        self.vehicle.v = 0
        self.vehicle.alpha = 0
        self.vehicle.Pb = 0
        self.vehicle.alpha_bounds = [0.0, 1.0]
        self.vehicle.Pb_bounds = [0.0, 3.0]  # MPa
        self.vehicle.d_alpha = (self.vehicle.alpha_bounds[1] -
                                self.vehicle.alpha_bounds[0]) / 0.5 * self.const.dt  # dt内的最大变化量
        self.vehicle.d_Pb = (self.vehicle.Pb_bounds[1] -
                             self.vehicle.Pb_bounds[0]) / 0.5 * self.const.dt
        # self.vehicle.alpha_thresh = 0.1 * (self.vehicle.alpha_bounds[1] -
        #                                    self.vehicle.alpha_bounds[0])  # 油门/制动切换阈值
        # self.vehicle.Pb_thresh = 0.1 * (self.vehicle.Pb_bounds[1] -
        #                                self.vehicle.Pb_bounds[0])
        self.vehicle.m = 1416  # 整车质量
        self.vehicle.Jf = 1.8  # 前轴转动惯量
        self.vehicle.Jr = 1.8  # 后轴转动惯量
        self.vehicle.r = 0.31  # 车轮半径
        self.vehicle.i0 = 4.10  # 主减速比
        self.vehicle.eff0 = 0.99  # 主减速器效率
        self.vehicle.Kb = 1000  # 制动力系数
        self.vehicle.Cd = 0.35  # 空气阻力系数
        self.vehicle.rou = 1.206  # 空气密度
        self.vehicle.A = 1.6  # 迎风面积
        self.vehicle.f = 0.005  # 滚动阻力系数
        self.vehicle.linear_vehicle_params = os.path.join(PROJECT_ROOT,
                                                    "model/data/linear_vehicle_params.npy")
        self.vehicle.fuel_rate_params = os.path.join(PROJECT_ROOT,
                                                     "model/data/fuel_rate_params.npy")
        self.vehicle.veh_alpha_split_points = [0.15, 0.55]  # 用于汽车模型分段线性化的节气门开度的分段点
        self.vehicle.fr_alpha_split_points = [0.6]  # 用于油耗模型分段线性化的节气门开度的分段点
        self.vehicle.fr_v_split_points = [20, 27]  # 用于油耗模型分段线性化的车速的分段点

        # controller
        CTRL_NAMES = ["PID", "LQR", "MPC"]
        self.controller = edict()
        self.controller.name = CTRL_NAMES[2]
        self.controller.v_tol = 0.1  # 节气门/制动切换允许的误差, m/s

        # lqr
        self.controller.lqr = edict()
        self.controller.lqr.Q1 = 1  # 速度误差系数
        self.controller.lqr.Q2 = 0  # 油耗系数, 600,600,100
        self.controller.lqr.R_alpha = 0.1
        self.controller.lqr.R_Pb = self.controller.lqr.R_alpha*\
                                   self.vehicle.alpha_bounds[1]/self.vehicle.Pb_bounds[1]

        # mpc
        self.controller.mpc = edict()
        self.controller.mpc.n_pred = 10

        # carsim data
        self.carsim = edict()
        self.carsim.path = os.path.join(PROJECT_ROOT, "data/carsim/")
        self.carsim.files = os.listdir(self.carsim.path)


if __name__ == '__main__':
    cfg = Config()
