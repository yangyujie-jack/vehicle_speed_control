import numpy as np
from params import *
import matplotlib.pyplot as plt


alphas = np.load(control_data_dir+"alpha_v100.npy")
alphas_fr = np.load(control_data_dir+"alpha_v100_fr.npy")
Pbs = np.load(control_data_dir+"Pb_v100.npy")
Pbs_fr = np.load(control_data_dir+"Pb_v100_fr.npy")

# 检查控制量的变化速度
for i in range(len(alphas)-1):
    if abs(alphas[i]-alphas[i+1])>(1+1e-3)*d_alpha:
        print(f"alpha change to fast! {abs(alphas[i]-alphas[i+1])}")
    if abs(Pbs[i]-Pbs[i+1])>(1+1e-3)*d_Pb:
        print(f"Pb change to fast! {abs(Pbs[i]-Pbs[i+1])}")
    if abs(alphas_fr[i]-alphas_fr[i+1])>(1+1e-3)*d_alpha:
        print(f"alpha_fr change to fast! {abs(alphas[i]-alphas[i+1])}")
    if abs(Pbs_fr[i]-Pbs_fr[i+1])>(1+1e-3)*d_Pb:
        print(f"Pb_fr change to fast! {abs(Pbs[i]-Pbs[i+1])}")

ts = np.arange(alphas.shape[0])*dt

start = np.where(ts<=110)[0][-1]
end = np.where(ts<=120)[0][-1]

plt.figure()
plt.subplot(2,1,1)
plt.plot(ts[start:end],alphas[start:end])
plt.xlabel("t/s")
plt.ylabel("alpha")
plt.title("not consider fuel rate")

plt.subplot(2,1,2)
plt.plot(ts[start:end],alphas_fr[start:end])
plt.xlabel("t/s")
plt.ylabel("alpha")
plt.title("consider fuel rate")

# plt.figure()
# plt.subplot(2,1,1)
# plt.plot(ts[start:end],Pbs[start:end])
# plt.xlabel("t/s")
# plt.ylabel("Pb/MPa")
# plt.title("not consider fuel rate")
#
# plt.subplot(2,1,2)
# plt.plot(ts[start:end],Pbs_fr[start:end])
# plt.xlabel("t/s")
# plt.ylabel("Pb/MPa")
# plt.title("consider fuel rate")

plt.show()