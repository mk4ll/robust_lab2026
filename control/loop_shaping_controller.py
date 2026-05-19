import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

# Nominal Plant Parameters
K = 0.638
tau = 0.5

# Controller Gains (Adjust these to test different loop shapes)
Kp = 8.0
Ki = 35.0
Kd = 0.2

# ---------------------------------------------------------
# Disturbance Rejection Transfer Functions: Gd(s) = G / (1 + GC)
# ---------------------------------------------------------
# 1. P Controller: C = Kp
num_Gd_P = [K]
den_Gd_P = [tau, 1 + Kp*K]
Gd_P = signal.TransferFunction(num_Gd_P, den_Gd_P)

# 2. PI Controller: C = (Kp*s + Ki)/s
num_Gd_PI = [K, 0]
den_Gd_PI = [tau, 1 + K*Kp, K*Ki]
Gd_PI = signal.TransferFunction(num_Gd_PI, den_Gd_PI)

# 3. PID Controller: C = (Kd*s^2 + Kp*s + Ki)/s
num_Gd_PID = [K, 0]
den_Gd_PID = [tau + K*Kd, 1 + K*Kp, K*Ki]
Gd_PID = signal.TransferFunction(num_Gd_PID, den_Gd_PID)

# Simulate Step Disturbance
t = np.linspace(0, 2, 500)
_, y_P = signal.step(Gd_P, T=t)
_, y_PI = signal.step(Gd_PI, T=t)
_, y_PID = signal.step(Gd_PID, T=t)

# ---------------------------------------------------------
# Loop Shaping Bode Plots: L(s) = G(s)C(s)
# ---------------------------------------------------------
w = np.logspace(-2, 3, 500)
L_P = signal.TransferFunction([K*Kp], [tau, 1])
L_PI = signal.TransferFunction([K*Kp, K*Ki], [tau, 1, 0])
L_PID = signal.TransferFunction([K*Kd, K*Kp, K*Ki], [tau, 1, 0])

w_P, mag_P, _ = signal.bode(L_P, w)
w_PI, mag_PI, _ = signal.bode(L_PI, w)
w_PID, mag_PID, _ = signal.bode(L_PID, w)

# Plotting
fig, axs = plt.subplots(2, 1, figsize=(10, 8))

# Plot 1: Time Domain
axs[0].plot(t, y_P, label=f'P (Kp={Kp})', linewidth=2)
axs[0].plot(t, y_PI, label=f'PI (Kp={Kp}, Ki={Ki})', linewidth=2)
axs[0].plot(t, y_PID, label=f'PID (Kp={Kp}, Ki={Ki}, Kd={Kd})', linewidth=2)
axs[0].axhline(0, color='k', linestyle=':')
axs[0].set_title('Input Disturbance Rejection (Unit Step Disturbance at t=0)')
axs[0].set_ylabel('Output y(t)')
axs[0].grid(True)
axs[0].legend()

# Plot 2: Frequency Domain (Loop Shaping)
axs[1].semilogx(w_P, mag_P, label='P Controller', linewidth=2)
axs[1].semilogx(w_PI, mag_PI, label='PI Controller', linewidth=2)
axs[1].semilogx(w_PID, mag_PID, label='PID Controller', linewidth=2)
axs[1].axhline(0, color='k', linestyle='-', alpha=0.5, label='0 dB (Crossover)')
axs[1].set_title('Loop Shaping: Open-Loop Magnitude |L(jw)|')
axs[1].set_xlabel('Frequency (rad/s)')
axs[1].set_ylabel('Magnitude (dB)')
axs[1].grid(True, which='both', linestyle='--')
axs[1].legend()

plt.tight_layout()
plt.show()