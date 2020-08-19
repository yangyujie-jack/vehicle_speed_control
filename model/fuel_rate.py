import numpy as np
import csv


class FuelRate:
    """
    油耗模型
    """
    def __init__(self, config):
        self.config = config.fuel_rate
        with open(self.config.data_path, 'r') as f:
            reader = csv.reader(f)
            self.MAP = np.array(list(reader), dtype=float)  # 油耗MAP，第一行是节气门开度，第一列是转速

    def get_fuel_rate(self, n, alpha):
        """
        :param n: 发动机转速，r/min
        :param alpha: 节气门开度
        :return: 油耗，kg/s
        """
        if alpha < 0 or alpha > self.MAP[0, -1]:
            raise ValueError(f"Invalid alpha {alpha}!")
        if n < self.MAP[1, 0] or n > self.MAP[-1, 0]:
            raise ValueError(f"Invalid n {n}!")
        i = np.where(self.MAP[1:, 0]>=n)[0][0]+1  # 转速在i-1和i之间
        j = np.where(self.MAP[0, 1:]>=alpha)[0][0]+1  # 节气门在j-1和j之间
        if i == 1:
            if j == 1:
                fr = self.MAP[i,j]
            else:
                alpha1 = self.MAP[0, j - 1]
                alpha2 = self.MAP[0, j]
                fr1 = self.MAP[i, j-1]
                fr2 = self.MAP[i, j]
                fr = (alpha-alpha1)/(alpha2-alpha1)*(fr2-fr1)+fr1
        else:
            n1 = self.MAP[i-1, 0]
            n2 = self.MAP[i, 0]
            if j == 1:
                fr1 = self.MAP[i-1, 1]
                fr2 = self.MAP[i, 1]
            else:
                alpha1 = self.MAP[0,j-1]
                alpha2 = self.MAP[0,j]
                [[T11,T12],[T21,T22]] = self.MAP[i-1:i+1,j-1:j+1]
                fr1 = (alpha-alpha1)/(alpha2-alpha1)*(T12-T11)+T11
                fr2 = (alpha-alpha1)/(alpha2-alpha1)*(T22-T21)+T21
            fr = (n-n1)/(n2-n1)*(fr2-fr1)+fr1
        return fr


if __name__ == '__main__':
    from config import Config

    cfg = Config()
    fr = FuelRate(cfg)
    for a in np.arange(0,1,0.1):
        print(fr.get_fuel_rate(1000, a))