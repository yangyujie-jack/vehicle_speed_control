from model import LinearVehicle
import numpy as np
from controller.base import BaseController


class LQRController(BaseController):
    def __init__(self, config):
        super(LQRController, self).__init__(config)
        self.config = config
        Kb = self.config.vehicle.Kb
        r = self.config.vehicle.r
        m = self.config.vehicle.m
        Jf = self.config.vehicle.Jf
        Jr = self.config.vehicle.Jr
        self.B_Pb = -Kb * r / (m * r ** 2 + Jf + Jr)
        self.Q1 = self.config.controller.lqr.Q1
        self.Q2 = self.config.controller.lqr.Q2
        self.R_alpha = self.config.controller.lqr.R_alpha
        self.R_Pb = self.config.controller.lqr.R_Pb
        self.linear_vehicle = LinearVehicle(self.config)

    def step(self, mode, x, eta, curr_alpha):
        """
        计算控制量
        :param slope: 当前坡度
        :param curr_alpha: 当前节气门开度
        :param mode: -1,0,1, 制动，不动，驱动
        :param x: 当前车速
        :param eta: 期望速度
        :return: 节气门开度, 制动压力
        """
        slope = 0  # 路面坡度未知，用0计算
        if mode == 0:
            alpha, Pb = 0, 0
        else:
            Kv = self.linear_vehicle.get_vehicle_param(slope, curr_alpha)
            A = Kv[0]
            B = Kv[1] if mode == 1 else self.B_Pb
            B1 = Kv[2]
            Q1 = self.Q1
            Q2 = self.Q2
            [_, C5, C4, C2, C3, C1] = \
                self.linear_vehicle.get_fuel_rate_param(curr_alpha, x)  # (1,u,x,u^2,ux,x^2)
            if mode == -1:
                C2, C3, C5 = 0, 0, 0
            R = max(0, -Q2*C2)+self.R_alpha if mode == 1 else self.R_Pb

            # 求解P，一元二次方程求根
            temp = Q2*C2+R
            _A = B**2/temp
            _B = -2*(A-Q2*C3*B/temp)
            _C = -Q1+Q2**2*C3**2/temp-Q2*C1
            delta = _B**2-4*_A*_C
            if _A == 0:
                raise ValueError("_A = 0!")
            if delta < 0:
                raise ValueError("delta < 0!")
            P = (-_B+np.sqrt(delta))/(2*_A)

            # 求解Xi，一元一次方程
            _a = (P*B**2+Q2*C3*B)/temp-A
            _b = P*B1-Q1*eta-Q2*C5*(P*B+Q2*C3)/temp+Q2*C4
            Xi = -_b/_a

            u = -(Q2*C3*x+Q2*C5+B*(P*x-Xi))/temp

            if mode == 1:
                alpha, Pb = u, 0
            else:
                alpha, Pb = 0, u

            alpha = np.clip(alpha, self.config.vehicle.alpha_bounds[0],
                            self.config.vehicle.alpha_bounds[1])
            Pb = np.clip(Pb, self.config.vehicle.Pb_bounds[0],
                         self.config.vehicle.Pb_bounds[1])
        return alpha, Pb