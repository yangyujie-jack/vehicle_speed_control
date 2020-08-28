

class BaseController:
    def __init__(self, config):
        self.config = config
        self.alpha_thresh = self.config.vehicle.d_alpha
        self.Pb_thresh = self.config.vehicle.d_Pb
        self.v_tol = self.config.controller.v_tol

    def get_mode(self, v, v_des, alpha, Pb):
        """
        油门/制动切换逻辑
        :param v: 车速
        :param v_des: 期望车速
        :param alpha: 节气门开度
        :param Pb: 制动压力
        :return: -1,0,1, 分别代表制动，不控制，油门
        """
        delta_v = v_des - v
        if delta_v > self.v_tol:
            if Pb > self.Pb_thresh:
                return -1
            else:
                return 1
        elif delta_v < -self.v_tol:
            if alpha > self.alpha_thresh:
                return 1
            else:
                return -1
        else:
            if Pb > self.Pb_thresh:
                return -1
            elif alpha > self.alpha_thresh:
                return 1
            else:
                return 0
