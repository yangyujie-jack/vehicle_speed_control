from scipy import optimize
from model import LinearVehicle
import numpy as np
from controller.base import BaseController
from controller.lqr import LQRController
import ctypes


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
        self._build_mpc_c_funcs()

    def _build_constraints(self):
        lb = np.zeros(self.n_pred*2)
        ub = np.concatenate([np.ones(self.n_pred)*self.alpha_bounds[1],
                             np.ones(self.n_pred)*self.Pb_bounds[1]])
        self.bounds = np.concatenate((lb[:, np.newaxis], ub[:, np.newaxis]), 1)

    def _build_mpc_c_funcs(self):
        self.dll = ctypes.CDLL(self.config.project_root + "/model/vehicle.dll")
        self._c_target_func = self.dll.mpc_target_func
        self._c_target_func.restype = ctypes.c_double
        self._veh_params = self.vehicle.vehicle_params.ctypes
        self._veh_alpha_sp = (ctypes.c_double*2)(*self.vehicle.veh_alpha_split_points)
        self.c_n_pred = ctypes.c_int(self.n_pred)
        self.c_dt = ctypes.c_double(self.dt)


    def _target_func(self, x, *args):
        v0, v_dess, alpha0, Pb0 = args
        x = x.ctypes
        v_dess = (ctypes.c_double*self.n_pred)(*v_dess)
        v0 = ctypes.c_double(v0)
        alpha0 = ctypes.c_double(alpha0)
        Pb0 = ctypes.c_double(Pb0)
        target_func = self._c_target_func(x, v_dess, self.c_n_pred, v0, alpha0, Pb0,
            self.vehicle.alpha_bounds, self.vehicle.Pb_bounds, self.vehicle.d_alpha,
            self.vehicle.d_Pb, self._veh_params, self._veh_alpha_sp,
            self.vehicle.k_Pb, self.vehicle.k_Fi, self.c_dt)

        return target_func

    def _get_init_control(self, v0, v_dess, alpha0, Pb0):
        alphas = []
        Pbs = []
        self.vehicle.set_v(v0)
        self.vehicle.set_control(alpha0, Pb0)
        for i in range(self.n_pred):
            v = self.vehicle.get_v()
            alpha, Pb = self.vehicle.get_control()
            alpha, Pb = self.controller.step(v, v_dess[i:i+1], alpha=alpha, Pb=Pb)
            alphas.append(alpha)
            Pbs.append(Pb)
            self.vehicle.control(alpha, Pb)
            self.vehicle.step()
        return np.array(alphas+Pbs)

    def _pred_control(self, v, v_dess, alpha, Pb):
        control0 = self._get_init_control(v, v_dess, alpha, Pb)
        # control0 = np.zeros(self.n_pred*2)
        # return control0[0], control0[self.n_pred]
        res = optimize.minimize(fun=self._target_func,
                                x0=control0,
                                args=(v, v_dess, alpha, Pb),
                                method=self.method,
                                bounds=self.bounds,
                                options={'ftol': 1e-8})
        if res.success:
            alpha, Pb = res.x[0], res.x[self.n_pred]
            return alpha, Pb
        else:
            raise RuntimeError("MPC optimize solver can't find solution!")

    def step(self, v, v_dess, **kwargs):
        alpha = kwargs['alpha']
        Pb = kwargs['Pb']
        return self._pred_control(v, v_dess, alpha, Pb)
