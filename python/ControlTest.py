
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import control as ctrl
import pandas as pd

# --- Continuous system (ZPK) ---
# MATLAB: zpk([], [-2 (-1 + 0.6i) (-1 - 0.6i)], 2)
zeros = []
poles = [-2, -1 + 0.38j, -1 - 0.38j]
gain = 2

sys = ctrl.zpk(zeros, poles, gain)  
print(sys)

# --- Time setup ---
T = 0.01
t = np.arange(0, 30 + T, T)
u = np.ones_like(t)

# --- Discretization (Tustin) ---
sysd = ctrl.sample_system(sys, T, method='tustin')

# Get transfer function coefficients
num, den = ctrl.tfdata(sysd)
b = np.squeeze(num)
a = np.squeeze(den)
print("Discrete coefficients (b):", b)
print("Discrete coefficients (a):", a)

np.savetxt("./data/dsp_plant.csv", np.column_stack([b, a]), delimiter=",")

# --- Step responses ---
t_cont, sys_y = ctrl.step_response(sys, t)
sysd_y = signal.lfilter(b, a, u)

np.savetxt("./data/iir_test_f64.csv", sysd_y, delimiter=",")

sysd_y_f32 = signal.lfilter(b.astype(np.float32), a.astype(np.float32), u.astype(np.float32)).astype(np.float32)
np.savetxt("./data/iir_test_f32.csv", sysd_y_f32, delimiter=",")
# --- Load external comparison data ---
try:
    data_f32 = pd.read_csv("./data/iir_output_f32.csv", header=None)
except FileNotFoundError:
    print("File not found.")
    data_f32 = None

try:
    data_f64 = pd.read_csv("./data/iir_output_f64.csv", header=None)
except FileNotFoundError:   
    print("File not found.")
    data_f64 = None

plt.figure()
plt.plot(t, sysd_y, "--", label="Reference discrete system")
plt.plot(t_cont, sys_y, ":", label="Continuous system")
if(data_f32 is not None):
    plt.plot(t, data_f32, "-", label="DSP-lib System (Float32)")
if(data_f64 is not None):
    plt.plot(t, data_f64, "-", label="DSP-lib System (Float64)")
plt.xlabel("t (s)")
plt.ylabel("u(t)")
plt.title("System Response Comparison")
plt.legend()
plt.grid()
plt.show()



# Kp = 2.9193
# Ki = 1.6070
# Kd = 1 / 0.7738

# print("Kp, Ki, Kd:", Kp, Ki, Kd)