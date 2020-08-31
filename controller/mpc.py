from scipy import optimize
from model import LinearVehicle
import numpy as np
import time
from controller.base import BaseController
from controller.pid import PIDController


class MPCController(BaseController):
    def __init__(self, config):
        super(MPCController, self).__init__(config)
        self.dt = self.config.const.dt
        self.n_pred = self.config.controller.mpc.n_pred
        self.gamma = 1.0
        self.vehicle = LinearVehicle(self.config)
        self.pid = PIDController(self.config)

    def target_func(self, x, *args):
        # t1 = time.time()
        alphas = x[:self.n_pred]
        Pbs = x[self.n_pred:]
        v, v_dess, alpha0, Pb0 = args
        self.vehicle.set_v(v)
        self.vehicle.set_control(alpha0, Pb0)
        targ_f = 0
        for alpha, Pb, v_des in zip(alphas, Pbs, v_dess):
            self.vehicle.control(alpha, Pb)
            self.vehicle.step()
            targ_f = (self.vehicle.v-v_des)**2*self.dt + self.gamma*targ_f
        # t2 = time.time()
        # self.t += t2-t1
        # self.call_num += 1
        return targ_f

    def get_init_control(self, v0, v_dess, alpha0, Pb0):
        alphas = []
        Pbs = []
        self.vehicle.set_v(v0)
        self.vehicle.set_control(alpha0, Pb0)
        for i in range(self.n_pred):
            v = self.vehicle.get_v()
            alpha, Pb = self.vehicle.get_control()
            mode = self.pid.get_mode(v, v_dess[i], alpha, Pb)
            alpha, Pb = self.pid.step(mode, v, v_dess[i])
            alphas.append(alpha)
            Pbs.append(Pb)
            self.vehicle.control(alpha, Pb)
            self.vehicle.step(0)
        return np.array(alphas+Pbs)

    def pred_control(self, mode, v, v_dess, alpha, Pb):
        control0 = self.get_init_control(v, v_dess, alpha, Pb)
        lb = np.zeros(self.n_pred*2)
        ub = np.concatenate([np.ones(self.n_pred)*self.alpha_bounds[1],
                             np.ones(self.n_pred)*self.Pb_bounds[1]])
        A = np.identity(self.n_pred*2)
        cons = optimize.LinearConstraint(A, lb, ub)
        # self.t = 0
        # self.call_num = 0
        # t1 = time.time()
        METHODS = ['L-BFGS-B', 'TNC', 'SLSQP', 'trust-constr']
        res = optimize.minimize(self.target_func,
                                x0=control0,
                                args=(v, v_dess, alpha, Pb),
                                method=METHODS[2],
                                constraints=cons,
                                options={'gtol': 1e-4})
        # t2 = time.time()
        # print(f"total time {format(t2-t1, '.2f')}, "
        #       f"func time {format(self.t, '.2f')}, "
        #       f"call num {self.call_num}")
        if res.success:
            alpha, Pb = res.x[0], res.x[self.n_pred]
            return alpha, Pb
        else:
            raise RuntimeError("MPC optimize solver can't find solution!")

    def step(self, mode, v, v_des, **kwargs):
        alpha = kwargs['alpha']
        Pb = kwargs['Pb']
        return self.pred_control(mode, v, v_des, alpha, Pb)
