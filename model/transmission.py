import numpy as np
import csv


class Transmission:
    """
    变速器模型
    """
    def __init__(self, config):
        self.config = config.transmission
        self.gears = self.config.gears
        self.effs = self.config.effs
        self.i = self.config.i
        with open(self.config.up_shift_data_path, 'r') as f:
            reader = csv.reader(f)
            self.up_shift = np.array(list(reader), dtype=float)
        with open(self.config.down_shift_data_path, 'r') as f:
            reader = csv.reader(f)
            self.down_shift = np.array(list(reader), dtype=float)

    def get_up_i(self, nt, alpha):
        """
        按升档规律计算应在的档位
        :param nt: 涡轮转速
        :param alpha: 节气门开度
        :return: 档位
        """
        if alpha <= self.up_shift[0,0]:
            n_refs = self.up_shift[1:,0]
        elif alpha >= self.up_shift[0,1]:
            n_refs = self.up_shift[1:,1]
        else:
            n_refs = (alpha-self.up_shift[0,0])/(self.up_shift[0,1]-self.up_shift[0,0])\
                     *(self.up_shift[1:,1]-self.up_shift[1:,0])+self.up_shift[1:,0]
        up_i = 1
        for n in n_refs:
            if n < nt:
                up_i += 1
            else:
                break
        return up_i

    def get_down_i(self, nt, alpha):
        """
        按降档规律计算应在的档位
        :param nt: 涡轮转速
        :param alpha: 节气门开度
        :return: 档位
        """
        if alpha <= self.down_shift[0,0]:
            n_refs = self.down_shift[1:,0]
        elif alpha >= self.down_shift[0,1]:
            n_refs = self.down_shift[1:,1]
        else:
            n_refs = (alpha-self.down_shift[0,0])/(self.down_shift[0,1]-self.down_shift[0,0])\
                     *(self.down_shift[1:,1]-self.down_shift[1:,0])+self.down_shift[1:,0]
        down_i = 1
        for n in n_refs:
            if n <= nt:
                down_i += 1
            else:
                break
        return down_i

    def change(self, nt, alpha):
        """
        换挡
        :param nt: 涡轮转速
        :param alpha: 节气门开度
        :return:
        """
        if self.config.enable:
            up_i = self.get_up_i(nt, alpha)
            down_i = self.get_down_i(nt, alpha)
            if up_i > self.i:
                self.i = up_i
            elif down_i < self.i:
                self.i = down_i

    def get_ig(self):
        return self.gears[self.i-1]

    def get_torque(self, Tt):
        """
        :param Tt: 涡轮转矩
        :return: 变速器输出转矩
        """
        return Tt*self.gears[self.i-1]*self.effs[self.i-1]