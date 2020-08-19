import numpy as np
import matplotlib.pyplot as plt
from model import Vehicle
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from config import Config


cfg = Config()


def get_vehicle_model(plot=False):
    """
    整车模型分段线性化
    :return: 线性化后的参数
    """
    vs = np.arange(10, 120, 10) / 3.6  # 车速，单位：m/s
    T_max = 50  # 最长仿真时间
    delta_alpha = 0.05
    param_num = 3
    params = np.zeros((len(cfg.vehicle.veh_alpha_split_points)+1, param_num))  # 记录回归得到的参数
    if plot:
        plt.figure()
        plt.xlabel("v/(m/s)")
        plt.ylabel("acc/(m/s^2)")
        plt.title("sampled data")
    for i in range(params.shape[0]):
        if i == 0:
            alphas = np.arange(0, cfg.vehicle.veh_alpha_split_points[0], delta_alpha)
        elif i == len(cfg.vehicle.veh_alpha_split_points):
            alphas = np.arange(cfg.vehicle.veh_alpha_split_points[-1], 1, delta_alpha)
        else:
            alphas = np.arange(cfg.vehicle.veh_alpha_split_points[i-1], cfg.vehicle.veh_alpha_split_points[i], delta_alpha)
        X, y = [], []
        for alpha in alphas:
            x_plt, y_plt = [], []
            vehicle = Vehicle(cfg)
            t = 0
            k = 0
            while t < T_max:
                if k == vs.shape[0]:
                    break
                if vehicle.v >= vs[k]:
                    acc = vehicle.get_acc()
                    X.append([vehicle.v, alpha])
                    y.append(acc)
                    x_plt.append(vehicle.v)
                    y_plt.append(acc)
                    k += 1
                vehicle.control(alpha, 0)
                vehicle.step()
                t += cfg.const.dt
            if vehicle.v < vs[-1]:
                # 速度上不去，补充后面的数据
                # 加速
                while vehicle.v < vs[-1]:
                    vehicle.control(1.0, 0)
                    vehicle.step()
                # 调油门
                while vehicle.alpha != alpha:
                    vehicle.control(alpha, 0)
                t = 0
                k = vs.shape[0]-1
                while t < T_max:
                    if vehicle.v <= vs[k]:
                        acc = vehicle.get_acc()
                        X.append([vehicle.v, alpha])
                        y.append(acc)
                        x_plt.append(vehicle.v)
                        y_plt.append(acc)
                        k -= 1
                    vehicle.step()
                    t += cfg.const.dt
                    if k < 0:
                        break
            x_plt, y_plt = (list(t) for t in zip(*sorted(zip(x_plt, y_plt))))
            if plot:
                plt.plot(x_plt, y_plt)
        X = np.array(X)
        y = np.array(y)
        model = LinearRegression()
        model.fit(X, y)
        params[i] = np.append(model.coef_, model.intercept_)
    if plot:
        plt.figure()
        plt.xlabel("v/(m/s)")
        plt.ylabel("acc/(m/s^2)")
        plt.title("fitted data")
        for alpha in np.arange(0, 1, delta_alpha):
            # 找节气门开度的区间
            if len(cfg.vehicle.veh_alpha_split_points) == 0:
                i = 0
            else:
                for i, asp in enumerate(cfg.vehicle.veh_alpha_split_points):
                    if asp > alpha:
                        break
                if alpha >= cfg.vehicle.veh_alpha_split_points[-1]:
                    i += 1
            accs = params[i,0]*vs+params[i,1]*alpha+params[i,2]
            plt.plot(vs, accs)
        plt.show()
    return params


# linear_vehicle_params = get_vehicle_model()
# np.save("linear_vehicle_params", linear_vehicle_params)


def get_fuel_rate_model(plot=False):
    """
    油耗模型分段线性化
    :param plot: 是否画图
    :return:
    """
    T_max = 50  # 最长仿真时间
    delta_alpha = 0.05  # 节气门开度间隔
    delta_v = 2  # 车速间隔，单位：km/h
    alphas = np.arange(0, 1, delta_alpha)
    vs = np.arange(10, 120, delta_v)/3.6
    frs = np.zeros((alphas.shape[0], vs.shape[0]))  # 记录采样得到的油耗数据
    if plot:
        plt.figure()
        plt.xlabel("v/(m/s)")
        plt.ylabel("fuel rate/(kg/s)")
        plt.title("sampled data")

    # 采样
    for i, alpha in enumerate(alphas):
        x_plt, y_plt = [], []
        vehicle = Vehicle(cfg)
        t = 0
        j = 0
        while t < T_max:
            if j == vs.shape[0]:
                break
            if vehicle.v >= vs[j]:
                v = vehicle.v
                fr = vehicle.get_fuel_rate()
                frs[i,j] = fr
                x_plt.append(v)
                y_plt.append(fr)
                j += 1
            vehicle.control(alpha, 0)
            vehicle.step()
            t += cfg.const.dt
        if vehicle.v < vs[-1]:
            # 速度上不去，补充后面的数据
            # 加速到最高车速
            while vehicle.v < vs[-1]:
                vehicle.control(1.0, 0)
                vehicle.step()
            # 油门调到目标值
            while vehicle.alpha != alpha:
                vehicle.control(alpha, 0)
            # 车速逐渐下降，记录油耗
            t = 0
            j = vs.shape[0]-1
            while t < T_max:
                if vehicle.v <= vs[j]:
                    v = vehicle.v
                    fr = vehicle.get_fuel_rate()
                    frs[i,j] = fr
                    x_plt.append(v)
                    y_plt.append(fr)
                    j -= 1
                vehicle.step()
                t += cfg.const.dt
                if j < 0:
                    break
        x_plt, y_plt = (list(t) for t in zip(*sorted(zip(x_plt, y_plt))))
        if plot:
            plt.plot(x_plt, y_plt)

    # 线性回归
    param_num = 6  # 回归的系数个数
    params = np.zeros((len(cfg.vehicle.fr_alpha_split_points)+1, len(cfg.vehicle.fr_v_split_points)+1, param_num))  # 记录回归得到的参数
    for i in range(params.shape[0]):
        if params.shape[0] == 1:
            i_range = np.arange(alphas.shape[0])
        elif i == 0:
            i_range = np.where(alphas < cfg.vehicle.fr_alpha_split_points[i])[0]
        elif i == len(cfg.vehicle.fr_alpha_split_points):
            i_range = np.where(alphas >= cfg.vehicle.fr_alpha_split_points[i-1])[0]
        else:
            i_range = np.where((alphas < cfg.vehicle.fr_alpha_split_points[i]) & (alphas >= cfg.vehicle.fr_alpha_split_points[i-1]))[0]
        for j in range(params.shape[1]):
            if params.shape[1] == 1:
                j_range = np.arange(vs.shape[0])
            elif j == 0:
                j_range = np.where(vs < cfg.vehicle.fr_v_split_points[j])[0]
            elif j == len(cfg.vehicle.fr_v_split_points):
                j_range = np.where(vs >= cfg.vehicle.fr_v_split_points[j-1])[0]
            else:
                j_range = np.where((vs < cfg.vehicle.fr_v_split_points[j]) & (vs >= cfg.vehicle.fr_v_split_points[j-1]))[0]
            # 构造特征
            X_alphas = np.tile(alphas[i_range], (j_range.shape[0], 1)).T
            X_vs = np.tile(vs[j_range], (i_range.shape[0], 1))
            X_alphas = np.expand_dims(X_alphas, axis=2)
            X_vs = np.expand_dims(X_vs, axis=2)
            X = np.concatenate((X_alphas, X_vs), axis=2).reshape((-1, 2))
            poly = PolynomialFeatures(degree=2)
            X = poly.fit_transform(X)  # (a,b)->(1,a,b,a^2,ab,b^2)
            y = np.reshape(frs[i_range,:][:,j_range], (-1,))
            model = LinearRegression(fit_intercept=False)
            model.fit(X, y)
            params[i,j] = model.coef_
    if plot:
        # 拟合结果
        plt.figure()
        plt.xlabel("v/(m/s)")
        plt.ylabel("fuel rate/(kg/s)")
        plt.title("fitted data")
        for alpha in alphas:
            x_plt, y_plt = [], []
            # 找节气门开度的区间
            if len(cfg.vehicle.fr_alpha_split_points) == 0:
                i = 0
            else:
                for i, asp in enumerate(cfg.vehicle.fr_alpha_split_points):
                    if asp > alpha:
                        break
                if alpha >= cfg.vehicle.fr_alpha_split_points[-1]:
                    i += 1
            for v in vs:
                # 找车速的区间
                if len(cfg.vehicle.fr_v_split_points) == 0:
                    j = 0
                else:
                    for j, vsp in enumerate(cfg.vehicle.fr_v_split_points):
                        if vsp > v:
                            break
                    if v >= cfg.vehicle.fr_v_split_points[-1]:
                        j += 1
                x_plt.append(v)
                y_plt.append(np.dot(params[i,j], np.array([1,alpha,v,alpha**2,alpha*v,v**2])))
            plt.plot(x_plt, y_plt)
        plt.show()
    return params


# 线性油耗参数
# fuel_rate_params = get_fuel_rate_model()
# np.save("fuel_rate_params", fuel_rate_params)