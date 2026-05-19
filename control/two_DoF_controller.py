import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

# Plant Parameters (Nominal)
K = 0.638
tau = 0.5

# Controller Parameters (From our loop shaping optimization)
Kp = 8.0
Ki = 35.0

# ---------------------------------------------------------
# Transfer Function Definitions
# ---------------------------------------------------------
# Plant G(s) = K / (tau*s + 1)
num_G = [K]
den_G = [tau, 1]

# PI Controller C(s) = (Kp*s + Ki) / s
num_C = [Kp, Ki]
den_C = [1, 0]

# Pre-filter F(s) = Ki / (Kp*s + Ki)
num_F = [Ki]
den_F = [Kp, Ki]

# Open Loop L(s) = G(s)C(s)
num_L = np.convolve(num_G, num_C)
den_L = np.convolve(den_G, den_C)

# ---------------------------------------------------------
# 1-DoF vs 2-DoF Closed-Loop Systems
# ---------------------------------------------------------
# 1-DoF Tracking: T_1dof(s) = L(s) / (1 + L(s))
num_T1 = num_L
den_T1 = np.polyadd(den_L, num_L)
T_1DoF = signal.TransferFunction(num_T1, den_T1)

# 2-DoF Tracking: T_2dof(s) = F(s) * T_1dof(s)
num_T2 = np.convolve(num_F, num_T1)
den_T2 = np.convolve(den_F, den_T1)
T_2DoF = signal.TransferFunction(num_T2, den_T2)

# Disturbance Rejection: Gd(s) = G(s) / (1 + L(s))
num_Gd = np.convolve(num_G, den_C) # G_num * C_den
den_Gd = den_T1 # 1 + G*C denominator is the same
Gd = signal.TransferFunction(num_Gd, den_Gd)

# ---------------------------------------------------------
# Simulations
# ---------------------------------------------------------
t = np.linspace(0, 2, 500)

# Simulate Command Tracking (Step Reference)
_, y_track_1dof = signal.step(T_1DoF, T=t)
_, y_track_2dof = signal.step(T_2DoF, T=t)

# Simulate Disturbance Rejection (Step Disturbance)
_, y_dist = signal.step(Gd, T=t)

# ---------------------------------------------------------
# Plotting
# ---------------------------------------------------------
fig, axs = plt.subplots(1, 2, figsize=(14, 5))

# Plot 1: Command Tracking Objective
axs[0].plot(t, y_track_1dof, label='1-DoF (PI Only)', linestyle='--', color='red', linewidth=2)
axs[0].plot(t, y_track_2dof, label='2-DoF (PI + Pre-filter)', color='blue', linewidth=2.5)
axs[0].axhline(1.0, color='k', linestyle=':', label='Reference Command')
axs[0].set_title('Objective 1: Command Tracking')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Motor Speed / Output')
axs[0].grid(True)
axs[0].legend()

# Plot 2: Disturbance Rejection Objective
axs[1].plot(t, y_dist, label='Disturbance Rejection (Both Architectures)', color='green', linewidth=2.5)
axs[1].axhline(0.0, color='k', linestyle=':')
axs[1].set_title('Objective 2: Input Disturbance Rejection')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Output Deviation')
axs[1].grid(True)
axs[1].legend()

plt.tight_layout()
plt.show()