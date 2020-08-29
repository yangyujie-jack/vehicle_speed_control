

class MonitorVehicle:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self._dt = self.vehicle.config.const.dt
        self._vs = []
        self._alphas = []
        self._Pbs = []
        self._s = 0
        self._fuel_csp = 0

    def control(self, alpha, Pb):
        self.vehicle.control(alpha, Pb)

    def step(self, slope=0):
        self.vehicle.step(slope)
        self._vs.append(self.vehicle.v)
        self._alphas.append(self.vehicle.alpha)
        self._Pbs.append(self.vehicle.Pb)
        self._s += self.vehicle.v*self._dt
        self._fuel_csp += self.vehicle.get_fuel_rate()*self._dt

    def get_v(self):
        return self.vehicle.v

    def get_vs(self):
        return self._vs

    def get_control(self):
        return self.vehicle.get_control()

    def get_controls(self):
        return self._alphas, self._Pbs

    def get_s(self):
        return self._s

    def get_fuel_scp(self):
        return self._fuel_csp

    def get_alphas_Pbs(self):
        return self._alphas, self._Pbs
