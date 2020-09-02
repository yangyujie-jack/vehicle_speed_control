from scipy.optimize import minimize
import numpy as np


# demo 1
# 计算 1/x+x 的最小值
def fun(x, *args):
    print(x[0])
    a = args[0]
    v = a/x[0]+x[0]
    return v


if __name__ == "__main__":
    args = (1,)  # a
    x0 = np.array([30])  # 初始猜测值
    res = minimize(fun, x0, args=args, bounds=[[1e-5, 100]], method='SLSQP')
    print(res.fun)
    print(res.success)
    print(res.x)