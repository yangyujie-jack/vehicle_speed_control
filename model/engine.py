import numpy as np
import csv


class Engine:
    """
    发动机模型
    """
    def __init__(self, config):
        self.config = config
        self.n = self.config.engine.n_idle
        with open(self.config.engine.data_path,'r') as f:
            reader = csv.reader(f)
            self.MAP = np.array(list(reader),dtype=float)  # 转矩MAP，第一行是节气门开度，第一列是转速

    def get_torque(self, alpha):
        """
        :param alpha: 节气门开度
        :return: 转矩
        """
        if alpha < 0 or alpha > self.MAP[0, -1]:
            raise ValueError(f"Invalid alpha {alpha}!")
        if self.n < 0 or self.n > self.MAP[-1, 0]:
            raise ValueError(f"Invalid n {self.n}!")
        i = np.where(self.MAP[:, 0] >= self.n)[0][0]  # 转速在i-1和i之间
        j = np.where(self.MAP[0, :] >= alpha)[0][0]  # 节气门在j-1和j之间
        if i == 0:
            Te = 0
        else:
            n1 = self.MAP[i-1, 0]
            n2 = self.MAP[i, 0]
            if j == 0:
                T1 = self.MAP[i-1, 1]
                T2 = self.MAP[i, 1]
            else:
                alpha1 = self.MAP[0, j-1]
                alpha2 = self.MAP[0, j]
                [[T11,T12],[T21,T22]] = self.MAP[i-1:i+1, j-1:j+1]
                T1 = (alpha-alpha1)/(alpha2-alpha1)*(T12-T11)+T11
                T2 = (alpha-alpha1)/(alpha2-alpha1)*(T22-T21)+T21
            Te = (self.n-n1)/(n2-n1)*(T2-T1)+T1
        return Te

    def step(self, alpha, Tp):
        """
        :param alpha: 节气门开度
        :param Tp: 液力变矩器泵轮转矩
        :return:
        """
        dw = (self.get_torque(alpha)-Tp)/self.config.engine.Je * self.config.const.dt
        self.n += dw*60/(2*np.pi)


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    engine = Engine(cfg)
    for _ in range(100):
        engine.step(0.5, 0)
        print(engine.get_torque(0.5))
