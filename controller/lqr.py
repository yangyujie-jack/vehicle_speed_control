from model import LinearVehicle, FuelRate
import numpy as np
from controller.base import BaseController


class LQRController(BaseController):
    def __init__(self, config):
        super(LQRController, self).__init__(config)
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
        self.fr = FuelRate(self.config)

    # def step(self, mode, v, v_des, **kwargs):
    #     slope = 0  # 路面坡度未知，用0计算
    #     alpha0 = kwargs['alpha']
    #     if mode == 0:
    #         alpha, Pb = 0, 0
    #     else:
    #         Kv = self.linear_vehicle.get_vehicle_param(slope, alpha0)
    #         A = Kv[0]
    #         B = Kv[1] if mode == 1 else self.B_Pb
    #         B1 = Kv[2]
    #         Q1 = self.Q1
    #         Q2 = self.Q2
    #         [C6, C5, C4, C2, C3, C1] = \
    #             self.linear_vehicle.get_fuel_rate_param(alpha0, v)  # (1,u,v,u^2,uv,v^2)
    #         if mode == -1:
    #             C2, C3, C5 = 0, 0, 0
    #         R = max(0, -Q2*C2)+self.R_alpha if mode == 1 else self.R_Pb
    #
    #         # 求解P，一元二次方程求根
    #         temp = Q2*C2+R
    #         _A = B**2/temp
    #         _B = -2*(A-Q2*C3*B/temp)
    #         _C = -Q1+Q2**2*C3**2/temp-Q2*C1
    #         delta = _B**2-4*_A*_C
    #         if _A == 0:
    #             raise ValueError("_A = 0!")
    #         if delta < 0:
    #             raise ValueError("delta < 0!")
    #         P = (-_B+np.sqrt(delta))/(2*_A)
    #
    #         # 求解Xi，一元一次方程
    #         _a = (P*B**2+Q2*C3*B)/temp-A
    #         _b = P*B1-Q1*v_des-Q2*C5*(P*B+Q2*C3)/temp+Q2*C4
    #         Xi = -_b/_a
    #
    #         u = -(Q2*C3*v+Q2*C5+B*(P*v-Xi))/temp
    #
    #         if mode == 1:
    #             alpha, Pb = u, 0
    #         else:
    #             alpha, Pb = 0, u
    #
    #         alpha = np.clip(alpha, self.config.vehicle.alpha_bounds[0],
    #                         self.config.vehicle.alpha_bounds[1])
    #         Pb = np.clip(Pb, self.config.vehicle.Pb_bounds[0],
    #                      self.config.vehicle.Pb_bounds[1])
    #     return alpha, Pb

    def step(self, mode, v, v_des, **kwargs):
        slope = 0  # 路面坡度未知，用0计算
        alpha0 = kwargs['alpha']
        n = kwargs['n']
        if mode == 0:
            alpha, Pb = 0, 0
        else:
            Kv = self.linear_vehicle.get_vehicle_param(slope, alpha0)
            A = Kv[0]
            B = Kv[1] if mode == 1 else self.B_Pb
            B1 = Kv[2]
            Q1 = self.Q1
            Q2 = self.Q2
            R = self.R_alpha if mode == 1 else self.R_Pb

            # 求解P，一元二次方程求根
            _A = B**2/R
            _B = -2*A
            _C = -Q1
            delta = _B**2-4*_A*_C
            if _A == 0:
                raise ValueError("_A = 0!")
            if delta < 0:
                raise ValueError("delta < 0!")
            P = (-_B+np.sqrt(delta))/(2*_A)

            # 求解Xi
            _a = P * B ** 2/R-A
            _b = P * B1 - Q1 * v_des
            Xi = -_b / _a

            u = -(P*v-Xi)*B/R

            if mode == -1:
                alpha, Pb = 0, u
                Pb = np.clip(Pb, self.config.vehicle.Pb_bounds[0],
                             self.config.vehicle.Pb_bounds[1])
            else:
                Pb = 0
                alpha0 = np.clip(u, self.config.vehicle.alpha_bounds[0],
                                self.config.vehicle.alpha_bounds[1])
                alpha = alpha0
                max_delta_alpha = 0.5
                ah = min(self.config.vehicle.alpha_bounds[1], alpha0+max_delta_alpha)
                al = max(self.config.vehicle.alpha_bounds[0], alpha0-max_delta_alpha)
                max_iter = 10
                tol = 1e-3
                for _ in range(max_iter):
                    dfr = self.fr.get_dfr(n, alpha)
                    # 求解Xi
                    _a = P * B ** 2 / R - A
                    _b = P * B1 - Q1 * v_des - P*B*Q2*dfr/R
                    Xi = -_b / _a
                    _alpha = -((P*v-Xi)*B+Q2*dfr)/R
                    _alpha = np.clip(_alpha, al, ah)
                    if abs(alpha-_alpha) < tol:
                        alpha = _alpha
                        break
                    alpha = _alpha
        return alpha, Pb

    def get_estimate_fr(self, v, alpha):
        [C6, C5, C4, C2, C3, C1] = \
            self.linear_vehicle.get_fuel_rate_param(alpha, v)  # (1,u,v,u^2,uv,v^2)
        fr = C6 + C5 * alpha + C4 * v + C3 * alpha ** 2 + C2 * alpha * v + C1 * v ** 2
        return fr