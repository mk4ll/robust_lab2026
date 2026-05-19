import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

# Plant parameters
tau = 0.5
K_nom = 0.638
K_min = 0.513
K_max = 0.614

# Desired closed-loop time constant (Tune this for faster/slower response)
tau_c = 0.1

def get_closed_loop(K, tau):
    # L(s) = G(s)C(s) = (K / (tau*s + 1)) * ((tau*s + 1) / (K_nom * tau_c * s))
    # Assuming the time constants perfectly cancel:
    # L(s) = K / (K_nom * tau_c * s)
    
    # Tracking: T(s) = L(s) / (1 + L(s))
    # T(s) = K / (K_nom * tau_c * s + K)
    num_T = [K]
    den_T = [K_nom * tau_c, K]
    
    # Input Disturbance Rejection: Gd(s) = G(s) / (1 + L(s))
    num_Gd = [K * K_nom * tau_c, 0]
    den_Gd = np.convolve([tau, 1], [K_nom * tau_c, K])
    
    return signal.TransferFunction(num_T, den_T), signal.TransferFunction(num_Gd, den_Gd)

# Build closed-loop systems
T_nom, Gd_nom = get_closed_loop(K_nom, tau)
T_min, Gd_min = get_closed_loop(K_min, tau)
T_max, Gd_max = get_closed_loop(K_max, tau)

# Time vector
t = np.linspace(0, 3, 500)

# 1. Step Tracking
_, y_nom = signal.step(T_nom, T=t)
_, y_min = signal.step(T_min, T=t)
_, y_max = signal.step(T_max, T=t)

# 2. Input Disturbance Rejection (Step input disturbance at t=0)
_, yd_nom = signal.step(Gd_nom, T=t)
_, yd_min = signal.step(Gd_min, T=t)
_, yd_max = signal.step(Gd_max, T=t)

# Plotting
plt.figure(figsize=(10, 8))

plt.subplot(2, 1, 1)
plt.plot(t, y_nom, label='Nominal ($R_{nom}$)', linewidth=2)
plt.plot(t, y_min, label='Min Load ($R_{min}$)', linestyle='--')
plt.plot(t, y_max, label='Max Load ($R_{max}$)', linestyle='-.')
plt.axhline(1.0, color='k', linestyle=':', label='Reference')
plt.title('Command Tracking (Step Response)')
plt.ylabel('Output y(t)')
plt.grid(True)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, yd_nom, label='Nominal ($R_{nom}$)', linewidth=2)
plt.plot(t, yd_min, label='Min Load ($R_{min}$)', linestyle='--')
plt.plot(t, yd_max, label='Max Load ($R_{max}$)', linestyle='-.')
plt.title('Input Disturbance Rejection (Step Disturbance)')
plt.xlabel('Time (s)')
plt.ylabel('Output y(t)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()