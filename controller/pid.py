from controller.base import BaseController


class PIDController(BaseController):
    """
    位置式PID控制器
    """
    def __init__(self, config):
        super(PIDController, self).__init__(config)
        # 节气门
        self.KP_a = 1
        self.KI_a = 0
        self.KD_a = 0

        # 制动压力
        self.KP_p = 1
        self.KI_p = 0
        self.KD_p = 0

        self.ei = 0  # e的积分
        self.e_last = 0  # 上一时刻的e

    def step(self, mode, v, v_des, alpha, slope):
        e = v_des - v
        if mode == -1:
            # 制动
            Pb = self.KP_p*e + self.KI_p*self.ei + self.KD_p*(e-self.e_last)
            alpha = 0
        elif mode == 0:
            # 无动作
            alpha, Pb = 0, 0
        else:
            # 节气门开度
            alpha = self.KP_a*e + self.KI_a*self.ei + self.KD_a*(e-self.e_last)
            Pb = 0
        self.ei = self.ei + e
        self.e_last = e
        return alpha, -Pb