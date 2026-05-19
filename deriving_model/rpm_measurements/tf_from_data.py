import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal

# --- OPERATING POINT PARAMETERS ---
PWM_REF = 200
MAX_PWM_DEV = 50.0  # Maximum PWM deviation (250 - 200)

# Fixed time constant (tau) derived from your previous step response analysis
TAU = 0.5  

# Dictionary mapping load names to your processed CSV files
files = {
    'RMin': 'RMin_measurements_analysis.csv',
    'RNom': 'RNom_measurements_analysis.csv',
    'RMax': 'RMax_measurements_analysis.csv'
}

gains = {}
transfer_functions = {}

# --- 1. CALCULATE STATIC GAIN (K) AND TRANSFER FUNCTIONS ---
plt.figure(figsize=(10, 6))

for name, filepath in files.items():
    try:
        # Read the processed CSV file. 
        # The TA's script uses ';' as separator and ',' for decimals
        df = pd.read_csv(filepath, sep=';', decimal=',')
        
        # In the processed files, the 1st column is 'PWM' and 2nd is 'Mean RPM'
        pwms = df.iloc[:, 0].values
        rpms = df.iloc[:, 1].values
        
        # Find reference RPM at the operating point (PWM = 200)
        idx_ref = np.where(pwms == PWM_REF)[0][0]
        rpm_ref = rpms[idx_ref]
        
        # Calculate deviations (Delta PWM and Delta RPM)
        delta_pwm = pwms - PWM_REF
        delta_rpm = rpms - rpm_ref
        
        # Convert to per-unit (pu) based on lab methodology
        u_pu = delta_pwm / MAX_PWM_DEV
        y_pu = delta_rpm / rpm_ref
        
        # Linear fit to find the slope K (y = K*x + b)
        coefs = np.polyfit(u_pu, y_pu, 1)
        K = coefs[0]  # The static gain is the slope of the line
        gains[name] = K
        
        # Plot the static characteristic lines
        plt.plot(u_pu, y_pu, 'o', label=f'{name} Data')
        plt.plot(u_pu, np.polyval(coefs, u_pu), '-', label=f'{name} Fit (K={K:.3f})')
        
        # Create the TransferFunction object using the calculated K and fixed tau
        transfer_functions[name] = signal.TransferFunction([K], [TAU, 1])
        
    except Exception as e:
        print(f"Error reading {filepath}: {e}")

plt.title('Static Gain (K) Calculation for Different Loads', fontsize=14, fontweight='bold')
plt.xlabel('Input Deviation u (pu)', fontsize=12)
plt.ylabel('Output Deviation y (pu)', fontsize=12)
plt.legend()
plt.grid(True, linestyle='--', alpha=0.7)
plt.show()

# --- 2. PRINT EXTRACTED TRANSFER FUNCTIONS ---
print("\n" + "="*50)
print("  TRANSFER FUNCTIONS G(s) EXTRACTION")
print("="*50)
for name, K in gains.items():
    print(f"--- For Load {name} ---")
    print(f"Gain K = {K:.3f}")
    print(f"TF_{name}(s) = {K:.3f} / ({TAU}s + 1)\n")

# --- 3. COMPARATIVE BODE PLOT ---
w = np.logspace(-2, 1, 500)
fig, (ax_mag, ax_phase) = plt.subplots(2, 1, figsize=(8, 8))

colors = {'RMin': 'tab:blue', 'RNom': 'tab:orange', 'RMax': 'tab:green'}

for name, tf in transfer_functions.items():
    w, mag, phase = signal.bode(tf, w)
    
    # Plot Magnitude
    ax_mag.semilogx(w, mag, color=colors[name], label=f'G_{name}(s) (K={gains[name]:.3f})', linewidth=2)
    
    # Phase is identical for all since tau is constant, using transparency to overlay them smoothly
    ax_phase.semilogx(w, phase, color=colors[name], linewidth=3, alpha=0.5)

ax_mag.set_title('Bode Plots for Different Load Resistances (Rv)', fontsize=14, fontweight='bold')
ax_mag.set_ylabel('Magnitude (dB)', fontsize=12)
ax_mag.grid(True, which='both', ls='--')
ax_mag.legend()

ax_phase.set_xlabel('Frequency (rad/s)', fontsize=12)
ax_phase.set_ylabel('Phase (deg)', fontsize=12)
ax_phase.grid(True, which='both', ls='--')

plt.tight_layout()
plt.show()