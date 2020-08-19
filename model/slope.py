import numpy as np
import matplotlib.pyplot as plt


def get_slope(s):
    """
    获取当前坡度
    :param s: 路程
    :return: 坡度, rad
    """
    # 假设坡度为若干个正弦波的叠加
    lmds = np.arange(10, 101, 10)
    max_slope = 2  # 最大坡度，单位：度
    slope = np.sum(max_slope*np.sin(2*np.pi/lmds*s))
    return slope*np.pi/180  # rad


if __name__ == '__main__':
    ss = np.arange(0, 2000, 1)
    slopes = np.zeros(ss.shape[0])
    for i, s in enumerate(ss):
        slopes[i] = get_slope(s)
    plt.plot(ss, slopes)
    plt.xlabel("x/m")
    plt.ylabel("i/rad")
    plt.show()