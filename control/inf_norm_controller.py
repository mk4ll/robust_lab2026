import control
import matplotlib.pyplot as plt
import numpy as np

# 1. Define the Laplace variable and Nominal Plant
s = control.tf('s')
G = 0.638 / (0.5 * s + 1)

# 2. Define the Weights
# Uncertainty Weight W3 (Bounds T(s) at high frequencies for Robust Stability)
# Using the Wm(s) we calculated in the previous step
W3 = (0.1 * s + 0.22) / (0.04 * s + 1)

# Performance Weight W1 (Bounds S(s) at low frequencies for Robust Performance)
# Target: Max 1% steady-state error, bandwidth ~2 rad/s, max peak Ms = 2
W1 = (0.5 * s + 2.0) / (s + 0.02)

# Control Effort Weight W2 (Keeps the controller from using infinite voltage)
# We set this to a small constant to keep the math well-conditioned
W2 = control.tf([0.01], [1])

# 3. Synthesize the H-infinity Controller
print("Synthesizing H-infinity controller...")
K, CL, info = control.mixsyn(G, w1=W1, w2=W2, w3=W3)
gamma = info[0]  # Extract gamma from the info tuple

print(f"Optimal Gamma achieved: {gamma:.4f}")
if gamma < 1:
    print("Success! Gamma < 1 means all specifications were strictly met.")
else:
    print("Warning: Gamma > 1. Specifications were relaxed to find a stable controller.")

# 4. Calculate Closed-Loop Transfer Functions
L = G * K
S = 1 / (1 + L)           # Sensitivity
T = L / (1 + L)           # Complementary Sensitivity

# 5. Frequency Response Analysis (Bode Magnitude Plots)
omega = np.logspace(-3, 3, 500)

# Get magnitudes
mag_S, _, _ = control.bode(S, omega, plot=False)
mag_W1_inv, _, _ = control.bode(1/W1, omega, plot=False)

mag_T, _, _ = control.bode(T, omega, plot=False)
mag_W3_inv, _, _ = control.bode(1/W3, omega, plot=False)

# ---------------------------------------------------------
# 8. Controller Discretization for Hardware Implementation
# ---------------------------------------------------------
Ts = 0.02  # 20 ms sampling time

print("\n=== Discretizing Controller for Hardware ===")

# Method 1: Tustin (Bilinear) Transform
# This is generally the safest method for high-performance controllers like H-infinity
K_discrete_tustin = control.sample_system(K, Ts, method='bilinear')
print("\nDiscrete Controller K(z) [Tustin/Bilinear]:")
print(K_discrete_tustin)

# Method 2: Euler Approximation (State Space)
# As requested by the lab manual as an alternative
K_ss = control.ss(K) # Ensure the controller is in state-space form
K_discrete_euler = control.sample_system(K_ss, Ts, method='euler')
print("\nDiscrete Controller K(z) State Space [Euler]:")
print(K_discrete_euler)

# 6. Plotting
fig, axs = plt.subplots(1, 2, figsize=(14, 5))

# Plot Sensitivity S(s) vs 1/W1(s)
axs[0].loglog(omega, mag_S, label='Sensitivity |S|', linewidth=2)
axs[0].loglog(omega, mag_W1_inv, label='Inverse Perf Weight |1/W1| (Upper Bound)', linestyle='--', color='red')
axs[0].set_title('Robust Performance: Sensitivity vs. Bound')
axs[0].set_xlabel('Frequency (rad/s)')
axs[0].set_ylabel('Magnitude')
axs[0].grid(True, which='both', linestyle=':')
axs[0].legend()

# Plot Complementary Sensitivity T(s) vs 1/W3(s)
axs[1].loglog(omega, mag_T, label='Comp. Sensitivity |T|', linewidth=2)
axs[1].loglog(omega, mag_W3_inv, label='Inverse Uncert Weight |1/W3| (Upper Bound)', linestyle='--', color='red')
axs[1].set_title('Robust Stability: Comp. Sensitivity vs. Bound')
axs[1].set_xlabel('Frequency (rad/s)')
axs[1].grid(True, which='both', linestyle=':')
axs[1].legend()

plt.tight_layout()
plt.show()

# 7. Time Domain Verification (Step Response)
t, y = control.step_response(T)
plt.figure(figsize=(7, 4))
plt.plot(t, y, linewidth=2, color='blue')
plt.axhline(1.0, color='k', linestyle=':', label='Reference')
plt.title('H-infinity Controller: Closed-Loop Step Response')
plt.xlabel('Time (s)')
plt.ylabel('Output')
plt.grid(True)
plt.legend()
plt.show()