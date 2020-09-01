from ctypes import *


dll = CDLL("./Dll1.dll")
bound_control = dll.bound_control
bound_control.argtypes = [c_double, c_double, POINTER(c_double), POINTER(c_double)]

_alpha, _Pb = c_double(0.0), c_double(0.0)
alpha, Pb = c_double(1.0), c_double(1.0)

bound_control(_alpha, _Pb, alpha, Pb)

print(alpha.value, Pb.value)
