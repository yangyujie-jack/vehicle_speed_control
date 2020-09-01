

dt = 0.025
g = 9.8

m = 1416  # 整车质量
Jf = 1.8  # 前轴转动惯量
Jr = 1.8  # 后轴转动惯量
r = 0.31  # 车轮半径
i0 = 4.10  # 主减速比
eff0 = 0.99  # 主减速器效率
Kb = 1000  # 制动力系数
Cd = 0.35  # 空气阻力系数
rou = 1.206  # 空气密度
A = 1.6  # 迎风面积
f = 0.005  # 滚动阻力系数
k_Pb = -Kb/r/(m+(Jf+Jr)/r**2)  # 线性汽车模型的制动压力系数

alpha_bounds = [0.0, 1.0]
Pb_bounds = [0.0, 3.0]  # MPa
d_alpha = (alpha_bounds[1] - alpha_bounds[0]) / 0.5 * dt  # dt内的最大变化量
d_Pb = (Pb_bounds[1] - Pb_bounds[0]) / 0.5 * dt

veh_alpha_split_points = [0.15, 0.55]  # 用于汽车模型分段线性化的节气门开度的分段点
fr_alpha_split_points = [0.6]  # 用于油耗模型分段线性化的节气门开度的分段点
fr_v_split_points = [20, 27]  # 用于油耗模型分段线性化的车速的分段点

