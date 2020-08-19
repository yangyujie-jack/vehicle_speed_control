

class BaseController:
    def __init__(self, config):
        self.config = config
        self.alpha_thresh = self.config.vehicle.alpha_thresh
        self.P_thresh = self.config.vehicle.P_thresh

    def get_mode(self, v, v_des, alpha, P):
        """
        油门/制动切换逻辑
        :param v: 车速
        :param v_des: 期望车速
        :param alpha: 节气门开度
        :param P: 制动压力
        :return: -1,0,1, 分别代表制动，不控制，油门
        """
        delta_v = v_des - v
        if delta_v > 0.1:
            if P > self.P_thresh:
                return -1
            else:
                return 1
        elif delta_v < -0.1:
            if alpha > self.alpha_thresh:
                return 1
            else:
                return -1
        else:
            if P > self.P_thresh:
                return -1
            elif alpha > self.alpha_thresh:
                return 1
            else:
                return 0
