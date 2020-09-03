from scipy import optimize
from model import LinearVehicle
import numpy as np
from controller.base import BaseController
from controller.lqr import LQRController
from controller.pid import PIDController


class MPCController(BaseController):
    def __init__(self, config):
        super(MPCController, self).__init__(config)
        self.dt = self.config.const.dt
        self.n_pred = self.config.controller.mpc.n_pred
        self.gamma = 1.0
        self.vehicle = LinearVehicle(self.config)
        self.controller = LQRController(self.config)
        METHODS = ['SLSQP', 'trust-constr']
        self.method = METHODS[0]
        self._build_constraints()

    def _build_constraints(self):
        lb = np.zeros(self.n_pred*2)
        ub = np.concatenate([np.ones(self.n_pred)*self.alpha_bounds[1],
                             np.ones(self.n_pred)*self.Pb_bounds[1]])
        A = np.identity(self.n_pred*2)
        self.cons = optimize.LinearConstraint(A, lb, ub)
        self.bounds = np.concatenate((lb[:, np.newaxis], ub[:, np.newaxis]), 1)

    def _target_func(self, x, *args):
        alphas = x[:self.n_pred]
        Pbs = x[self.n_pred:]
        v0, v_dess, alpha0, Pb0 = args
        self.vehicle.set_v(v0)
        self.vehicle.set_control(alpha0, Pb0)
        targ_f = 0
        for alpha, Pb, v_des in zip(alphas, Pbs, v_dess):
            self.vehicle.control(alpha, Pb)
            self.vehicle.step()
            targ_f = (self.vehicle.get_v()-v_des)**2*self.config.const.dt + \
                     self.gamma*targ_f
        # print(f"target func value: {targ_f}")
        return targ_f

    def _get_init_control(self, v0, v_dess, alpha0, Pb0):
        alphas = []
        Pbs = []
        self.vehicle.set_v(v0)
        self.vehicle.set_control(alpha0, Pb0)
        for i in range(self.n_pred):
            v = self.vehicle.get_v()
            alpha, Pb = self.vehicle.get_control()
            mode = self.controller.get_mode(v, v_dess[i], alpha, Pb)
            alpha, Pb = self.controller.step(mode, v, v_dess[i], alpha=alpha)
            alphas.append(alpha)
            Pbs.append(Pb)
            self.vehicle.control(alpha, Pb)
            self.vehicle.step()
        return np.array(alphas+Pbs)

    def _pred_control(self, mode, v, v_dess, alpha, Pb):
        control0 = self._get_init_control(v, v_dess, alpha, Pb)
        # control0 = np.zeros(self.n_pred*2)
        # return control0[0], control0[self.n_pred]
        self.t = 0
        self.step_t = 0
        # print("\n--optimize--\n")
        res = optimize.minimize(fun=self._target_func,
                                x0=control0,
                                args=(v, v_dess, alpha, Pb),
                                method=self.method,
                                bounds=self.bounds,
                                options={'ftol': 1e-8})
        # res = optimize.dual_annealing(func=self._target_func,
        #                               x0=control0,
        #                               args=(v, v_dess, alpha, Pb),
        #                               bounds=self.bounds)
        if res.success:
            alpha, Pb = res.x[0], res.x[self.n_pred]
            return alpha, Pb
        else:
            raise RuntimeError("MPC optimize solver can't find solution!")

    def step(self, mode, v, v_des, **kwargs):
        alpha = kwargs['alpha']
        Pb = kwargs['Pb']
        return self._pred_control(mode, v, v_des, alpha, Pb)
