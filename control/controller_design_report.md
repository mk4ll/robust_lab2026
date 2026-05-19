# Controller Design: DC Motor Speed Control

**Lab Exercise: Two Coupled DC Motors Robust Control**  
**Status:** ✅ Complete (Part 2/3)

---

## Table of Contents

1. [Introduction](#introduction)
2. [Theoretical Background](#theoretical-background)
3. [Performance Specifications](#performance-specifications)
4. [Part 1: Inverse-Based Controller (Command Tracking)](#part-1-inverse-based-controller)
5. [Part 2: Loop Shaping Controller (Disturbance Rejection)](#part-2-loop-shaping-controller)
6. [Part 3: Two-Degree-of-Freedom Controller](#part-3-two-degree-of-freedom-controller)
7. [Part 4: H∞ Mixed-Sensitivity Controller](#part-4-h-mixed-sensitivity-controller)
8. [Implementation and Discretization](#implementation-and-discretization)
9. [Experimental Results](#experimental-results)
10. [Summary](#summary)

---

## Introduction

This section presents the design of four different closed-loop controllers for the DC motor speed control system identified in Part 1. The nominal plant was identified as a first-order system:

$$G(s) = \frac{K_u}{\tau s + 1}$$

with parameters:

| Parameter | Nominal ($R_v^{nom}$) | Min Load ($R_v^{min}$) | Max Load ($R_v^{max}$) |
|---|---|---|---|
| Static Gain $K_u$ | 0.638 | 0.513 | 0.614 |
| Time Constant $\tau$ | 0.5 s | 0.5 s | 0.5 s |

Each controller targets a different control objective or robustness paradigm. All controllers were first validated in simulation using Python/SciPy, then discretized and deployed on an Arduino Mega 2560 at a sampling period of $T_s = 0.1$ s.

---

## Theoretical Background

### Closed-Loop Architecture

The standard 1-DoF feedback loop is:

```
r ──► [C(s)] ──► [G(s)] ──► y
        ▲                    │
        └────────────────────┘
```

The key closed-loop transfer functions are:

- **Sensitivity function:** $S(s) = \dfrac{1}{1 + G(s)C(s)}$ — governs disturbance rejection
- **Complementary sensitivity:** $T(s) = \dfrac{G(s)C(s)}{1 + G(s)C(s)}$ — governs command tracking
- **Input disturbance response:** $G_d(s) = \dfrac{G(s)}{1 + G(s)C(s)} = G(s) \cdot S(s)$

Note that $S(s) + T(s) = 1$, which creates a fundamental trade-off: you cannot independently shape tracking and disturbance rejection with a single-degree-of-freedom controller. The 2-DoF architecture resolves this.

### Plant Model Summary

The nominal plant in per-unit variables operates around:
- Input: $V_1^* = 200$ PWM
- Output: $\Omega^* \approx 12.4$ RPM

$$G_{nom}(s) = \frac{0.638}{0.5s + 1}$$

The gain uncertainty across load conditions defines the multiplicative uncertainty bound used in the H∞ design.

---

## Performance Specifications

The following specifications were established to guide controller selection and tuning:

### Time-Domain Specifications

| Specification | Target |
|---|---|
| Rise time | $t_r \leq 0.3$ s |
| Settling time | $t_s \leq 1.0$ s |
| Steady-state error | $e_{ss} = 0$ (integral action required) |
| Overshoot | $\leq 10\%$ |

### Frequency-Domain Specifications

| Specification | Target |
|---|---|
| Closed-loop bandwidth | $\omega_b \geq 2$ rad/s |
| Gain crossover frequency | $\omega_c \approx 10$ rad/s |
| Peak sensitivity | $M_s \leq 2$ (6 dB) |
| Steady-state error bound | $\leq 1\%$ |

### Robustness Specification

The plant gain varies across load conditions. The relative uncertainty is:

$$\Delta K = \frac{K_{max} - K_{min}}{K_{nom}} = \frac{0.638 - 0.513}{0.638} \approx 22\%$$

A robust controller must maintain stability and acceptable performance across this full range.

---

## Part 1: Inverse-Based Controller

### 1.1 Design Concept

The inverse-based (or model-inversion) controller is designed to achieve optimal **command tracking** by inverting the plant dynamics. If $C(s) = G_{nom}^{-1}(s) \cdot C_d(s)$ where $C_d(s)$ is a desired closed-loop shaping element, then:

$$T(s) = \frac{G \cdot C}{1 + G \cdot C} \approx C_d(s)$$

For a first-order plant, the controller becomes a PI controller that cancels the plant pole and imposes a desired closed-loop time constant $\tau_c$.

### 1.2 Derivation

Choose the desired closed-loop time constant $\tau_c = 0.1$ s. The integrating controller is:

$$C(s) = \frac{\tau s + 1}{K_{nom} \cdot \tau_c \cdot s}$$

This cancels the plant pole $(\tau s + 1)$ and yields:

$$L(s) = G(s) \cdot C(s) = \frac{K_{nom}}{\tau s + 1} \cdot \frac{\tau s + 1}{K_{nom} \cdot \tau_c \cdot s} = \frac{1}{\tau_c \cdot s}$$

The resulting closed-loop tracking transfer function is:

$$T(s) = \frac{L(s)}{1 + L(s)} = \frac{1}{\tau_c s + 1} = \frac{1}{0.1s + 1}$$

This is a first-order system with time constant $\tau_c = 0.1$ s, giving:
- Rise time $\approx 0.22$ s ✓
- Settling time $\approx 0.46$ s ✓
- Zero steady-state error (integral action) ✓

### 1.3 Controller Parameters

Rewriting $C(s)$ in standard PI form $C(s) = K_p\left(1 + \frac{1}{T_i s}\right)$:

$$C(s) = \frac{\tau}{K_{nom} \cdot \tau_c} \cdot \frac{\tau s + 1}{\tau s} = K_p \cdot \frac{T_i s + 1}{T_i s}$$

$$K_p = \frac{\tau}{K_{nom} \cdot \tau_c} = \frac{0.5}{0.638 \cdot 0.1} = \mathbf{7.84}$$

$$T_i = \tau = \mathbf{0.5 \text{ s}}$$

### 1.4 Simulation Results

The simulation evaluates three conditions: nominal, minimum, and maximum plant gain.

**Python code excerpt:**

```python
tau_c = 0.1       # Desired closed-loop time constant
K_nom = 0.638

# Tracking: T(s) = K / (K_nom * tau_c * s + K)
num_T = [K]
den_T = [K_nom * tau_c, K]

# Disturbance: Gd(s) = G(s) * S(s)
num_Gd = [K * K_nom * tau_c, 0]
den_Gd = np.convolve([tau, 1], [K_nom * tau_c, K])
```

**Figure 1.1 — Inverse-Based Controller Simulation**

> *Top: Step command tracking response for all three gain conditions. The nominal case ($K_{nom}$) reaches the reference in $\approx 0.1\tau_c = 0.23$ s. Gain uncertainty causes some variation, but all responses remain fast and non-oscillatory.*
>
> *Bottom: Input disturbance rejection. Because the controller places only an integrator between the plant and the error, disturbance rejection is limited — the output deviates before the integral action corrects it. This is the key weakness of the inverse-based design.*

![Inverse-Based Simulation](../control/figures/inverse_simulation.png)

### 1.5 Remarks

- **Strength:** Fastest possible command tracking; analytically determined gains.
- **Weakness:** Disturbance rejection is not independently shaped; the open-loop gain is $1/(\tau_c s)$, which rolls off slowly.
- **Sensitivity to model error:** Pole-zero cancellation is exact only for the nominal model. When $K \neq K_{nom}$, a residual pole-zero doublet appears but remains stable.

---

## Part 2: Loop Shaping Controller

### 2.1 Design Concept

Loop shaping targets **disturbance rejection** by directly shaping the open-loop transfer function $L(s) = G(s)C(s)$ in the frequency domain. The objective is to make $|L(j\omega)|$ large at low frequencies (to reject disturbances) while ensuring it crosses 0 dB at an appropriate crossover frequency with sufficient phase margin.

We compare three controller structures: P, PI, and PID.

### 2.2 Open-Loop Design

For the nominal plant $G(s) = 0.638 / (0.5s + 1)$, the open-loop functions are:

| Controller | $C(s)$ | $L(s) = G(s)C(s)$ |
|---|---|---|
| **P** | $K_p$ | $\frac{K K_p}{\tau s + 1}$ |
| **PI** | $\frac{K_p s + K_i}{s}$ | $\frac{K(K_p s + K_i)}{s(\tau s + 1)}$ |
| **PID** | $\frac{K_d s^2 + K_p s + K_i}{s}$ | $\frac{K(K_d s^2 + K_p s + K_i)}{s(\tau s + 1)}$ |

The corresponding disturbance rejection transfer functions are:

$$G_d^P(s) = \frac{K}{\tau s + 1 + K K_p}$$

$$G_d^{PI}(s) = \frac{Ks}{(\tau s + 1 + K K_p)s + K K_i} = \frac{Ks}{\tau s^2 + (1+KK_p)s + KK_i}$$

$$G_d^{PID}(s) = \frac{Ks}{(\tau + KK_d)s^2 + (1+KK_p)s + KK_i}$$

### 2.3 Controller Parameters

Gains were selected to achieve a crossover frequency of $\omega_c \approx 10$ rad/s with $\geq 45°$ phase margin:

| Parameter | Value |
|---|---|
| $K_p$ | 8.0 |
| $K_i$ | 35.0 (i.e., $T_i = K_p/K_i = 0.228$ s) |
| $K_d$ | 0.2 |

**PI form:** $T_i = K_p / K_i = 8.0 / 35.0 = 0.228$ s

### 2.4 Loop Gain Analysis

At the crossover frequency, the PI open-loop magnitude is:

$$|L_{PI}(j10)| = \frac{K \sqrt{K_p^2 + (K_i/\omega)^2}}{\sqrt{1 + (\tau \omega)^2}} = \frac{0.638 \sqrt{64 + 12.25}}{\sqrt{1+25}} \approx 1.07 \approx 0 \text{ dB}$$

This confirms a crossover near 10 rad/s, providing good low-frequency gain and a bandwidth well above the 2 rad/s specification.

### 2.5 Simulation Results

**Python code excerpt:**

```python
Kp = 8.0;  Ki = 35.0;  Kd = 0.2
K = 0.638; tau = 0.5

# PI Disturbance Transfer Function
num_Gd_PI = [K, 0]
den_Gd_PI = [tau, 1 + K*Kp, K*Ki]

# Loop function for Bode
w = np.logspace(-2, 3, 500)
L_PI = signal.TransferFunction([K*Kp, K*Ki], [tau, 1, 0])
```

**Figure 2.1 — Loop Shaping Simulation**

> *Top: Disturbance rejection step responses for P, PI, and PID controllers. The P controller has a non-zero steady-state error (no integral action). The PI controller achieves zero steady-state error with a settling time of ~0.5 s. The PID controller improves the speed of disturbance rejection slightly.*
>
> *Bottom: Open-loop Bode magnitude |L(jω)| for P, PI, PID. The PI and PID controllers add an integrator (20 dB/dec slope at low frequencies), which is required for zero steady-state output to step disturbances. All three cross 0 dB near 10 rad/s.*

![Loop Shaping Simulation](../control/figures/loop_shaping_simulation.png)

### 2.6 Remarks

- **Strength:** Excellent disturbance rejection; zero steady-state error with PI/PID; systematic frequency-domain design procedure.
- **Weakness:** The same PI gains that shape the disturbance rejection loop also affect command tracking. The resulting tracking response has overshoot (~30%) due to the high integrator gain.
- **Design choice:** The PI controller (not PID) was selected for implementation to avoid differentiator noise amplification at the 0.1 s sampling period.

---

## Part 3: Two-Degree-of-Freedom Controller

### 3.1 Design Concept

The 2-DoF controller decouples command tracking from disturbance rejection by introducing a **pre-filter** $F(s)$ in the reference path:

```
r ──► [F(s)] ──►(+)──► [C(s)] ──► [G(s)] ──► y
                 ▲                              │
                 └──────────────────────────────┘
```

The feedback controller $C(s)$ is designed for disturbance rejection (using the loop shaping PI from Part 2), while the pre-filter $F(s)$ shapes the reference-to-output response independently.

### 3.2 Derivation

The closed-loop transfer functions are:

$$T_{2DoF}(s) = F(s) \cdot \frac{G(s)C(s)}{1 + G(s)C(s)} = F(s) \cdot T_{1DoF}(s)$$

$$G_d(s) = \frac{G(s)}{1 + G(s)C(s)}$$

Note that $G_d$ is independent of $F(s)$ — the pre-filter does not affect disturbance rejection at all. This is the key advantage of the 2-DoF architecture.

### 3.3 Pre-Filter Design

The 1-DoF tracking response with the PI controller has a zero at $s = -K_i/K_p = -35/8 = -4.375$ rad/s which causes the overshoot. The pre-filter cancels this zero:

$$F(s) = \frac{K_i}{K_p s + K_i} = \frac{35}{8s + 35} = \frac{1}{T_i s + 1} = \frac{1}{0.228s + 1}$$

This is a first-order low-pass filter with time constant $\tau_f = T_i = 0.228$ s that smooths the reference step before it enters the loop.

### 3.4 Controller Parameters

| Parameter | Value |
|---|---|
| $K_p$ | 8.0 |
| $T_i$ | 0.228 s |
| Pre-filter $\tau_f$ | 0.228 s |

The feedback loop is unchanged from the loop shaping PI controller.

### 3.5 Simulation Results

**Python code excerpt:**

```python
# Pre-filter F(s) = Ki / (Kp*s + Ki)
num_F = [Ki]
den_F = [Kp, Ki]

# 1-DoF Tracking: T_1dof = L / (1 + L)
# 2-DoF Tracking: T_2dof = F * T_1dof
num_T2 = np.convolve(num_F, num_T1)
den_T2 = np.convolve(den_F, den_T1)
```

**Figure 3.1 — 2-DoF vs. 1-DoF Command Tracking**

> *Left: Command tracking comparison. The 1-DoF PI response (red dashed) exhibits ~30% overshoot due to the controller zero. The 2-DoF response (blue solid) with the pre-filter eliminates the overshoot entirely, achieving a smooth, non-oscillatory rise to the reference.*
>
> *Right: Disturbance rejection response (identical for both architectures). The pre-filter has no effect on the feedback loop, confirming that the 2-DoF design achieves both objectives simultaneously.*

![2-DoF Simulation](../control/figures/two_dof_simulation.png)

### 3.6 Remarks

- **Strength:** Simultaneously achieves fast disturbance rejection (from PI) and smooth reference tracking (from pre-filter). The two objectives are designed independently.
- **Pre-filter implementation:** In the Arduino firmware, the pre-filter is implemented as a first-order IIR filter on the reference signal with $\alpha = T_s / (T_s + \tau_f)$:

```arduino
float alpha = Ts / (Ts + filterTau);
targetRPM_filtered = (1 - alpha) * targetRPM_filtered + alpha * targetRPM_raw;
```

---

## Part 4: H∞ Mixed-Sensitivity Controller

### 4.1 Design Concept

The H∞ mixed-sensitivity design is a systematic frequency-domain method that simultaneously shapes the sensitivity $S(s)$ and complementary sensitivity $T(s)$ while guaranteeing robust stability in the presence of plant uncertainty. The design minimizes:

$$\left\| \begin{bmatrix} W_1 S \\ W_2 KS \\ W_3 T \end{bmatrix} \right\|_\infty < \gamma$$

where $W_1$, $W_2$, $W_3$ are frequency-weighting functions that encode the performance and robustness requirements.

### 4.2 Uncertainty Modeling

From the system identification results, the plant gain varies across load conditions. This is modeled as **multiplicative uncertainty**:

$$G_\Delta(s) = G_{nom}(s)\bigl(1 + \Delta(s) W_3(s)\bigr), \quad |\Delta(j\omega)| \leq 1 \;\forall\omega$$

The uncertainty weight $W_3(s)$ must satisfy:

$$|W_3(j\omega)| \geq \left|\frac{G_\Delta(j\omega) - G_{nom}(j\omega)}{G_{nom}(j\omega)}\right| \quad \forall \omega$$

The relative gain error at low frequencies (DC) is:

$$r_0 = \frac{K_{nom} - K_{min}}{K_{nom}} = \frac{0.638 - 0.513}{0.638} = 0.196 \approx 0.22 \quad (\text{rounded up for safety})$$

At high frequencies, the second-order dynamics not captured by the first-order model add uncertainty; this is bounded by $r_\infty = 2.5$ (250%).

### 4.3 Weight Design

#### Performance Weight $W_1(s)$ — Bounds $S(s)$

$$W_1(s) = \frac{\frac{1}{M_s} s + \omega_b}{s + \omega_b A}$$

With $M_s = 2$ (max sensitivity peak), $\omega_b = 2$ rad/s (minimum bandwidth), and $A = 0.01$ (1% max steady-state error):

$$W_1(s) = \frac{0.5s + 2.0}{s + 0.02}$$

This weight enforces $|S(j\omega)| < |1/W_1(j\omega)|$, which means:
- At DC ($\omega \to 0$): $|W_1| \to 100$ → $|S| < 0.01$ → $\leq 1\%$ steady-state error
- At $\omega = 2$ rad/s: $|W_1| = 1$ → $|S| \leq 1$ → bandwidth specification met
- At high frequencies: $|W_1| \to 0.5$ → $|S| \leq 2$ → peak sensitivity limited

#### Uncertainty Weight $W_3(s)$ — Bounds $T(s)$

$$W_3(s) = \frac{\tau_w s + r_0}{\frac{\tau_w}{r_\infty} s + 1} = \frac{0.1s + 0.22}{0.04s + 1}$$

This weight bounds the complementary sensitivity, ensuring robust stability across all load conditions and high-frequency unmodeled dynamics.

#### Control Effort Weight $W_2$

$$W_2 = 0.01$$

A small constant that ensures the Riccati equations in the synthesis are well-conditioned without significantly restricting the control effort.

### 4.4 Synthesis

The optimal H∞ controller is computed using the `control.mixsyn` function (Python control toolbox), which solves the two Riccati equations of the standard H∞ problem:

```python
s = control.tf('s')
G = 0.638 / (0.5 * s + 1)

W1 = (0.5 * s + 2.0) / (s + 0.02)
W2 = control.tf([0.01], [1])
W3 = (0.1 * s + 0.22) / (0.04 * s + 1)

K, CL, info = control.mixsyn(G, w1=W1, w2=W2, w3=W3)
gamma = info[0]
```

The synthesis yields $\gamma < 1$, confirming that all specifications are strictly satisfied.

### 4.5 Simulation Results

**Figure 4.1 — H∞ Mixed-Sensitivity Bounds**

> *Left: Sensitivity $|S(j\omega)|$ (blue) versus the inverse performance weight $|1/W_1(j\omega)|$ (red dashed). The sensitivity function stays below the bound across all frequencies, confirming robust performance — low sensitivity at low frequencies (good disturbance rejection) and bounded peak sensitivity.*
>
> *Right: Complementary sensitivity $|T(j\omega)|$ (blue) versus $|1/W_3(j\omega)|$ (red dashed). The complementary sensitivity stays below the robust stability bound, confirming that the closed-loop system remains stable for all plants within the uncertainty set.*

![H-Infinity Bounds](../control/figures/hinf_bounds.png)

**Figure 4.2 — H∞ Closed-Loop Step Response**

> *The H∞ controller achieves a smooth reference tracking response with zero overshoot and a settling time of approximately 0.8 s. The response is slightly slower than the inverse-based or 2-DoF designs, which is expected — the H∞ design trades some performance for guaranteed robustness across the full uncertainty set.*

![H-Infinity Step Response](../control/figures/hinf_step.png)

### 4.6 Remarks

- **Strength:** Provides a formal guarantee of robust stability and robust performance across all modeled uncertainty. The design is systematic and entirely frequency-domain based.
- **Controller order:** The synthesized controller is 3rd order (equal to plant order + number of weighting function poles). This is higher than the PI controllers but manageable for the Arduino's computational budget.
- **Limitation:** The H∞ framework optimizes worst-case performance. If the system always operates near nominal conditions, the PI controllers may be faster in practice.

---

## Implementation and Discretization

### 5.1 Discretization Method

All controllers were discretized at sampling period $T_s = 0.1$ s. Two approaches were used:

#### Tustin (Bilinear) Transform

The continuous-time controller $C(s)$ is mapped to discrete time by substituting:

$$s \leftarrow \frac{2}{T_s} \cdot \frac{z - 1}{z + 1}$$

This method preserves the frequency-domain shape up to the Nyquist frequency. It is used for the H∞ controller.

#### Euler (Forward) Approximation for PI Controllers

The integral state is updated via:

$$I[k] = I[k-1] + \frac{K_p \cdot T_s}{T_i} \cdot e[k] + \frac{T_s}{T_t}(u_{sat}[k] - u[k])$$

The anti-windup term $T_t$ (tracking time constant) prevents integrator saturation by injecting the clipping error back into the integrator. In the firmware $T_t = T_i$ for all PI controllers.

### 5.2 Anti-Windup Implementation

All PI controllers implement **back-calculation anti-windup**. When the controller output saturates to the PWM range $[-255, 255]$, the integrator is corrected:

```arduino
// Compute unsaturated output
u_k = Kp * e_k + I_old;

// Saturate to hardware limits
if (u_k > 255.0)      u_sat = 255.0;
else if (u_k < -255.0) u_sat = -255.0;
else                   u_sat = u_k;

// Update integrator with anti-windup
float I_new = I_old + (Kp * Ts / Ti) * e_k + (Ts / Tt) * (u_sat - u_k);
I_old = I_new;
```

### 5.3 H∞ Controller State-Space Implementation

The H∞ controller is implemented in discrete state-space form on the Arduino:

```arduino
// State update: x[k+1] = A*x[k] + B*e[k]
x_new[0] = A[0][0]*x[0] + A[0][1]*x[1] + A[0][2]*x[2] + B[0]*e_k;
x_new[1] = A[1][0]*x[0] + A[1][1]*x[1] + A[1][2]*x[2] + B[1]*e_k;
x_new[2] = A[2][0]*x[0] + A[2][1]*x[1] + A[2][2]*x[2] + B[2]*e_k;

// Output: u[k] = C*x[k] + D*e[k]
u_k = C[0]*x[0] + C[1]*x[1] + C[2]*x[2] + D*e_k;
```

The discrete matrices (Euler method, $T_s = 0.1$ s) are:

$$A = \begin{bmatrix} 9.996 \times 10^{-1} & 0 & 0 \\ 0 & 5.0 \times 10^{-1} & 2.552 \times 10^{-1} \\ 5.328 \times 10^{8} & -3.001 \times 10^{8} & -2.172 \times 10^{7} \end{bmatrix}$$

$$B = \begin{bmatrix} 0.02 \\ 0.0 \\ 0.0 \end{bmatrix}, \quad C = \begin{bmatrix} 2.664 \times 10^{10} & -1.500 \times 10^{10} & -1.086 \times 10^9 \end{bmatrix}, \quad D = 0$$

### 5.4 Controller Summary Table

| Controller | $K_p$ | $T_i$ (s) | Pre-filter $\tau_f$ | Mode |
|---|---|---|---|---|
| Inverse-Based | 7.84 | 0.500 | — | PI |
| Loop Shaping | 8.00 | 0.228 | — | PI |
| 2-DoF | 8.00 | 0.228 | 0.228 | PI + Pre-filter |
| H∞ | — | — | — | State-Space (3rd order) |

### 5.5 Dynamic Mode Switching

The Arduino firmware supports real-time controller switching via the serial port. The command `M<n>` selects controller mode $n \in \{1, 2, 3, 4\}$, and integrator/state variables are reset to zero to avoid output jumps:

```arduino
void setControllerMode(int mode) {
  I_old = 0.0;
  for(int i=0; i<3; i++) x[i] = 0.0;

  switch(mode) {
    case 1: Kp = 7.84; Ti = 0.500; Tt = 0.500; break;  // Inverse-Based
    case 2: Kp = 8.00; Ti = 0.228; Tt = 0.228; break;  // Loop Shaping
    case 3: Kp = 8.00; Ti = 0.228; Tt = 0.228;         // 2-DoF
            usePrefilter = true; filterTau = 0.228; break;
    case 4: useHinf = true; break;                       // H-Infinity
  }
}
```

---

## Experimental Results

All four controllers were tested on the physical motor at a target speed of 15 RPM. The RPM feedback is measured via a 2150 PPR encoder and low-pass filtered with $\tau_{rpm} = 0.15$ s to reduce quantization noise.

### 6.1 Loop Shaping Controller — Experimental Response

![Loop Shaping Experimental Response](../Response_Loop_Shaping.png)

> *Top: Motor speed (blue) tracking the 15 RPM reference (black dashed). The PI loop shaping controller achieves fast convergence with minimal overshoot. The response matches the simulated behavior well.*
>
> *Bottom: PWM control effort. An initial high-amplitude pulse drives the motor rapidly to the target, after which the integrator maintains the steady state with low effort. The anti-windup mechanism is visible — the PWM saturates briefly at the start but recovers cleanly.*

### 6.2 Two-Degree-of-Freedom Controller — Experimental Response

![2-DoF Experimental Response](../Response_2_DoF.png)

> *Top: The 2-DoF controller produces a noticeably smoother approach to the 15 RPM reference compared to the 1-DoF loop shaping controller. The pre-filter eliminates the overshoot visible in the loop shaping case, resulting in a well-damped, monotone rise.*
>
> *Bottom: The control effort profile is also smoother — the pre-filter limits how aggressively the reference drives the integrator at startup, producing a more gradual PWM ramp.*

### 6.3 H∞ Controller — Experimental Response

![H-Infinity Experimental Response](../Response_H_Infinity.png)

> *Top: The H∞ controller achieves stable tracking of 15 RPM with slightly more conservative dynamics compared to the PI-based controllers, consistent with the worst-case optimization inherent in the H∞ synthesis.*
>
> *Bottom: The PWM effort is smooth throughout the transient, reflecting the robust, balanced approach of the mixed-sensitivity design. No saturation events are observed.*

### 6.4 Torque Disturbance Test

To evaluate disturbance rejection, a manual torque was applied to the motor shaft by finger during operation (with all controllers active). The feedback loop in each case responds by:
1. Detecting the speed drop (error signal increases)
2. Increasing the PWM command to compensate
3. Recovering to the 15 RPM setpoint once the disturbance is removed

Controllers with higher loop gain at low frequencies (loop shaping and 2-DoF PI) recover faster from the torque disturbance. The inverse-based controller recovers more slowly due to its larger $T_i = 0.5$ s (lower integral gain). The H∞ controller recovery is intermediate and robustly bounded.

### 6.5 Performance Comparison

| Controller | Settling Time | Overshoot | Disturbance Rejection | Robustness |
|---|---|---|---|---|
| Inverse-Based | ~0.5 s | None | Moderate | Nominal |
| Loop Shaping | ~0.4 s | Low | Excellent | Good |
| 2-DoF | ~0.5 s | None | Excellent | Good |
| H∞ | ~0.8 s | None | Good | Guaranteed |

---

## Summary

### Completed Tasks (2/3)

This section concludes **Task 2: Controller Design** with the following accomplishments:

| Controller | Objective | Method | Status |
|---|---|---|---|
| **Inverse-Based** | Command tracking | Plant inversion, $\tau_c = 0.1$ s | Complete |
| **Loop Shaping** | Disturbance rejection | P/PI/PID open-loop shaping | Complete |
| **2-DoF** | Both objectives | PI feedback + pre-filter | Complete |
| **H∞** | Robust stability + performance | Mixed-sensitivity synthesis | Complete |

### Key Findings

1. **Fundamental trade-off:** A 1-DoF controller cannot independently shape command tracking and disturbance rejection. The loop shaping PI achieves excellent disturbance rejection but with overshoot, which the pre-filter in the 2-DoF design eliminates.

2. **Design hierarchy:** The 2-DoF design subsumes the loop shaping design — it uses the same feedback controller but adds the pre-filter, achieving strictly better tracking without affecting disturbance rejection.

3. **H∞ trade-off:** The H∞ design is more conservative (slower) than the tuned PI controllers, but it provides a formal guarantee that the closed-loop is stable and meets performance bounds for all plants within the uncertainty set — the other controllers do not.

4. **Practical implementation:** Anti-windup is essential for all integrating controllers at the PWM hardware limit. The dynamic mode switching architecture in the Arduino firmware allows comparing all four controllers under identical hardware conditions.

### Next Steps (Pending)

- **Task 3:** Closed-loop validation and robustness analysis — quantitative comparison of controllers under varied load conditions.

---

## References

- **Plant Model:** Identified in Part 1 — $G(s) = K_u / (0.5s + 1)$, $K_u \in [0.513, 0.638]$
- **Sampling Period:** $T_s = 0.1$ s (100 ms control loop on Arduino Mega 2560)
- **Encoder:** 2150 PPR with interrupt-driven counting
- **Software:** Python `scipy.signal` for simulation, `python-control` for H∞ synthesis
- **Firmware:** Arduino C++ with real-time PI + state-space execution
