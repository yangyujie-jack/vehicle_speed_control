from scipy import optimize
from model import LinearVehicle, Vehicle
import numpy as np
import time
from controller.base import BaseController


class MPCController(BaseController):
    def __init__(self, config):
        super(MPCController, self).__init__(config)
        self.dt = self.config.const.dt
        self.t_pred = 10
        self.n_pred = int(self.t_pred/self.dt)
        self.gamma = 0.9
        self._alphas_pred = []
        self._Pbs_pred = []

    def target_func(self, controls, v, v_des, alpha0, Pb0):
        t1 = time.time()
        alphas = controls[:len(controls)//2]
        Pbs = controls[len(controls)//2:]
        self.vehicle = LinearVehicle(self.config)
        self.vehicle.set_v(v)
        self.vehicle.set_control(alpha0, Pb0)
        targ_f = 0
        for alpha, Pb in zip(alphas, Pbs):
            self.vehicle.control(alpha, Pb)
            self.vehicle.step()
            targ_f = (self.vehicle.v-v_des)**2*self.dt + self.gamma*targ_f
        t2 = time.time()
        self.t += t2-t1
        self.call_num += 1
        return targ_f

    def pred_control(self, mode, v, v_des, alpha, Pb):
        control0 = np.zeros(self.n_pred*2)
        lb = np.concatenate([np.ones(self.n_pred)*self.config.vehicle.alpha_bounds[0],
                             np.ones(self.n_pred)*self.config.vehicle.Pb_bounds[0]])
        ub = np.concatenate([np.ones(self.n_pred)*self.config.vehicle.alpha_bounds[1],
                             np.ones(self.n_pred)*self.config.vehicle.Pb_bounds[1]])
        bounds = optimize.Bounds(lb, ub)
        self.t = 0
        self.call_num = 0
        t1 = time.time()
        res = optimize.minimize(self.target_func,
                                x0=control0,
                                args=(v, v_des, alpha, Pb),
                                bounds=bounds,
                                tol=0.1,
                                options={"maxiter": 100})
        t2 = time.time()
        # print(f"total time {format(t2-t1, '.2f')}, "
        #       f"func time {format(self.t, '.2f')}, "
        #       f"call num {self.call_num}")
        if res.success:
            control = res.x
            self._alphas_pred.extend(control[:self.n_pred])
            self._Pbs_pred.extend(control[self.n_pred:])
            return control
        else:
            raise RuntimeError("MPC optimize solver can't find solution!")

    def step(self, mode, v, v_des, alpha, Pb):
        if not self._alphas_pred or not self._Pbs_pred:
            self.pred_control(mode, v, v_des, alpha, Pb)
        return self._alphas_pred.pop(0), self._Pbs_pred.pop(0)


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    veh = Vehicle(cfg)
    mpc = MPCController(cfg)
    t_use = 5
    n_use = int(t_use/cfg.const.dt)
    v_des = 20
    for _ in range(1000):
        alpha, Pb = veh.get_control()
        control = mpc.pred_control(1, veh.v, v_des, alpha, Pb)
        alphas, Pbs = control[:n_use], control[mpc.n_pred:mpc.n_pred+n_use]
        for alpha, Pb in zip(alphas, Pbs):
            print(f"v={format(veh.v, '.3f')}, "
                  f"alpha={format(alpha, '.2f')}, "
                  f"Pb={format(Pb, '.2f')}")
            veh.control(alpha, Pb)
            veh.step(0)





