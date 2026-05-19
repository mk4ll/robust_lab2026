import control
import numpy as np
import matplotlib.pyplot as plt

print("Synthesizing controllers and calculating robustness boundaries...")

# 1. Define the Laplace variable and Nominal Plant
s = control.tf('s')
G = 0.638 / (0.5 * s + 1)

# 2. Define the Weights
W1 = (0.5 * s + 2.0) / (s + 0.02)         # Performance Weight
W2 = control.tf([0.01], [1])              # Control Effort Weight
Wm = (0.1 * s + 0.22) / (0.04 * s + 1)    # Uncertainty Weight (W3)

# 3. Define / Synthesize the Controllers
# A. Classical PI Controller (from previous loop shaping)
C_pi = (15 * s + 30) / s 

# B. Robust H-infinity Controller
C_hinf, _, info = control.mixsyn(G, w1=W1, w2=W2, w3=Wm)
gamma = info[0]
print(f"H-infinity Gamma achieved: {gamma:.4f}")

# 4. Calculate Sensitivity (S) and Complementary Sensitivity (T)
# For PI
L_pi = G * C_pi
S_pi = 1 / (1 + L_pi)
T_pi = L_pi / (1 + L_pi)

# For H-infinity
L_hinf = G * C_hinf
S_hinf = 1 / (1 + L_hinf)
T_hinf = L_hinf / (1 + L_hinf)

# 5. Frequency Response Analysis
omega = np.logspace(-3, 3, 1000)

# Get magnitudes for Weights
mag_W1, _, _ = control.bode(W1, omega, plot=False)
mag_Wm, _, _ = control.bode(Wm, omega, plot=False)

# Get magnitudes for PI
mag_S_pi, _, _ = control.bode(S_pi, omega, plot=False)
mag_T_pi, _, _ = control.bode(T_pi, omega, plot=False)

# Get magnitudes for H-infinity
mag_S_hinf, _, _ = control.bode(S_hinf, omega, plot=False)
mag_T_hinf, _, _ = control.bode(T_hinf, omega, plot=False)

# 6. Calculate the "Big Three" Conditions for both
# PI Metrics
NP_pi = mag_W1 * mag_S_pi
RS_pi = mag_Wm * mag_T_pi
RP_pi = NP_pi + RS_pi

# H-infinity Metrics
NP_hinf = mag_W1 * mag_S_hinf
RS_hinf = mag_Wm * mag_T_hinf
RP_hinf = NP_hinf + RS_hinf

# 7. Plotting the Comparison
fig, axs = plt.subplots(1, 2, figsize=(14, 6), sharey=True)

# Plot 1: Classical PI Controller
axs[0].semilogx(omega, NP_pi, label='Nominal Performance |W1*S|', linestyle='--', linewidth=2)
axs[0].semilogx(omega, RS_pi, label='Robust Stability |Wm*T|', linestyle='-.', linewidth=2)
axs[0].semilogx(omega, RP_pi, label='Robust Performance |W1*S| + |Wm*T|', color='red', linewidth=3)
axs[0].axhline(1.0, color='k', linestyle=':', linewidth=2, label='Failure Limit')
axs[0].set_title('Classical PI Controller (FAILS)')
axs[0].set_xlabel('Frequency (rad/s)')
axs[0].set_ylabel('Magnitude')
axs[0].set_ylim([0, 1.5])
axs[0].grid(True, which='both', linestyle=':')
axs[0].legend()

# Plot 2: Robust H-infinity Controller
axs[1].semilogx(omega, NP_hinf, label='Nominal Performance |W1*S|', linestyle='--', linewidth=2)
axs[1].semilogx(omega, RS_hinf, label='Robust Stability |Wm*T|', linestyle='-.', linewidth=2)
axs[1].semilogx(omega, RP_hinf, label='Robust Performance |W1*S| + |Wm*T|', color='green', linewidth=3)
axs[1].axhline(1.0, color='k', linestyle=':', linewidth=2, label='Failure Limit')
axs[1].set_title('Robust H-infinity Controller (PASSES)')
axs[1].set_xlabel('Frequency (rad/s)')
axs[1].grid(True, which='both', linestyle=':')
axs[1].legend()

plt.suptitle('Robustness Analysis: Classical vs. H-infinity Control', fontsize=16)
plt.tight_layout()
plt.show()