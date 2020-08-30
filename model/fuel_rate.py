import numpy as np
import csv
import matplotlib.pyplot as plt


class FuelRate:
    """
    油耗模型
    """
    def __init__(self, config):
        self.config = config.fuel_rate
        with open(self.config.data_path, 'r') as f:
            reader = csv.reader(f)
            self.MAP = np.array(list(reader), dtype=float)
            # 油耗MAP，第一行是节气门开度，第一列是转速

    def get_fuel_rate(self, n, alpha):
        """
        :param n: 发动机转速，r/min
        :param alpha: 节气门开度
        :return: 油耗，kg/s
        """
        if alpha < 0 or alpha > self.MAP[0, -1]:
            raise ValueError(f"Invalid alpha {alpha}!")
        if n < self.MAP[1, 0]:
            raise ValueError(f"Invalid n {n}!")

        _i = np.where(self.MAP[1:, 0] <= n)[0][-1]+1
        _j = np.where(self.MAP[0, 1:] <= alpha)[0][-1]+1
        if _i == self.MAP.shape[0] - 1:
            i = _i - 1
        else:
            i = _i
        if _j == self.MAP.shape[1] - 1:
            j = _j - 1
        else:
            j = _j
        n1 = self.MAP[i, 0]
        n2 = self.MAP[i+1, 0]
        a1 = self.MAP[0, j]
        a2 = self.MAP[0, j+1]
        [[f11, f12], [f21, f22]] = self.MAP[i:i+2, j:j+2]
        f1 = (alpha - a1) / (a2 - a1) * (f12 - f11) + f11
        f2 = (alpha - a1) / (a2 - a1) * (f22 - f21) + f21
        f = (n - n1) / (n2 - n1) * (f2 - f1) + f1
        return f

    def plot_fr(self):
        """
        画不同发动机转速下，油耗关于节气门开度的曲线
        """
        alphas = self.MAP[0, 1:]
        plt.figure()
        for i in range(1, self.MAP.shape[0]):
            frs = self.MAP[i, 1:]
            plt.plot(alphas, frs)
        plt.xlabel("alpha")
        plt.ylabel("fuel rate")
        plt.show()

    def get_dfr(self, n, alpha):
        """
        油耗对节气门开度的导数
        """
        dalpha = 0.01
        alpha1 = max(0, alpha-dalpha)
        alpha2 = min(1, alpha+dalpha)
        fr1 = self.get_fuel_rate(n, alpha1)
        fr2 = self.get_fuel_rate(n, alpha2)
        dfr = (fr2-fr1)/(alpha2-alpha1)
        return dfr


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    fr = FuelRate(cfg)