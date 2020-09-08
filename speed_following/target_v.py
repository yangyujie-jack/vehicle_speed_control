from model import WLTC


def get_wltc_v(t):
    k = min(int(t), 1799)
    v_des = ((t - k) * (WLTC[k + 1] - WLTC[k]) + WLTC[k])
    return v_des/3.6


def get_const_v(t, const_v):
    const_v /= 3.6
    a = 1  # 加速度，单位：m/s^2
    t_a = const_v / a  # 加速时间
    if t < t_a:
        v_des = const_v / t_a * t
    else:
        v_des = const_v
    return v_des  # m/s


def get_v_dess(test, t, dt, n, **kwargs):
    """
    返回未来n个时刻的期望速度
    """
    if test == "const_v":
        const_v = kwargs['const_v']
        v_dess = []
        for i in range(n):
            v_des = get_const_v(t + i * dt, const_v)
            v_dess.append(v_des)
    else:
        v_dess = []
        for i in range(n):
            v_des = get_wltc_v(t + i * dt)
            v_dess.append(v_des)
    return v_dess
