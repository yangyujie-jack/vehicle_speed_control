dt = 0.025  # 仿真步长，单位：s
g = 9.8  # 重力加速度

m = 1416  # 整车质量
Jf = 1.8  # 前轴转动惯量
Jr = 1.8  # 前轴转动惯量
r = 0.31  # 车轮半径
Je = 0.2  # 发动机转动惯量
n_idle = 750  # 发动机怠速转速, r/min
Cd = 0.35  # 空气阻力系数
rou = 1.206  # 空气密度
A = 1.6  # 汽车迎风面积
f = 0.005  # 滚动阻力系数
Kb = 1000  # 制动力系数
gears = [4.38,2.86,1.92,1.37,1.00,0.82,0.70]  # 变速器传动比，7档
effs = [0.92,0.92,0.95,0.95,0.98,0.99,0.99]  # 变速器效率
i0 = 4.10  # 主减速比
eff0 = 0.99  # 主减速器效率
engine_data_path = "Engine.csv"  # 发动机转矩MAP数据路径
TC_K_path = "TC_K.csv"  # 液力变矩器扭矩特性
TC_Kp_inv_path = "TC_Kp.csv"  # 液力变矩器容量特性
up_shift_path = "up_shift.csv"  # 变速器向上换挡规律
down_shift_path = "down_shift.csv"  # 变速器向下换挡规律
fueal_rate_path = "FuelRate.csv"  # 油耗数据
control_data_dir = "control_data/"

# 控制量
alpha_min, alpha_max = 0.0, 1.0
Pb_min, Pb_max = 0.0, 3.0  # MPa
d_Pb = (Pb_max-Pb_min)/0.5*dt  # 一个仿真步长内，制动压力最大变化量
d_alpha = (alpha_max-alpha_min)/0.5*dt  # 一个仿真步长内，节气门开度最大变化量
P_thresh = 0.1*(Pb_max-Pb_min)  # 油门/制动切换阈值, MPa
alpha_thresh = 0.1*(alpha_max-alpha_min)  # 油门/制动切换阈值
