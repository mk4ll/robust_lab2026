import control

# Define the Laplace variable
s = control.tf('s')

print("=== H-infinity Weight Calculator ===\n")

# ---------------------------------------------------------
# 1. Performance Weight W1(s) -> Bounds Sensitivity S(s)
# ---------------------------------------------------------
Ms = 2.0       # Maximum Peak Sensitivity (Limits overshoot)
wb = 2.0       # Minimum Bandwidth in rad/s (Dictates speed of response)
A  = 0.01      # Max steady-state error (1%)

# Formula: W1 = (s/Ms + wb) / (s + wb*A)
W1 = ((1/Ms)*s + wb) / (s + wb*A)

print("1. Performance Weight W1(s):")
print(W1)

# ---------------------------------------------------------
# 2. Control Effort Weight W2(s) -> Bounds K(s)S(s)
# ---------------------------------------------------------
# Kept as a small constant to ensure the Riccati equations 
# are well-conditioned without choking motor performance.
W2_val = 0.01
W2 = control.tf([W2_val], [1])

print("2. Control Effort Weight W2(s):")
print(W2)

# ---------------------------------------------------------
# 3. Uncertainty Weight W3(s) -> Bounds Comp. Sensitivity T(s)
# ---------------------------------------------------------
r0 = 0.22      # Low-frequency relative error (22% from R_min/R_max variations)
r_inf = 2.5    # High-frequency relative error (250% to cover the unmodeled 2nd pole)
tau_w = 0.1    # Time constant (places the uncertainty crossover around 10 rad/s)

# Formula: W3 = (tau_w*s + r0) / ((tau_w/r_inf)*s + 1)
W3 = (tau_w*s + r0) / ((tau_w/r_inf)*s + 1)

print("3. Uncertainty Weight W3(s):")
print(W3)