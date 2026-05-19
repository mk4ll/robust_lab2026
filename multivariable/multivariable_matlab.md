# Quadruple-Tank Process: Modeling, Construction, and Control

This document outlines the comprehensive steps to model, build, identify, analyze, and simulate the multivariable quadruple-tank system based on the provided literature.

## 1. Mathematical Modeling

The quadruple-tank process consists of two lower tanks (Tanks 1 and 2) and two upper tanks (Tanks 3 and 4) that drain freely into the lower tanks. Two pumps deliver water from a basin: Pump 1 feeds Tanks 1 and 4, while Pump 2 feeds Tanks 2 and 3.

Using mass balances and Bernoulli's law for flow out of an orifice, the non-linear dynamics of the system are described by the following differential equations:

$$\frac{dh_1}{dt} = -\frac{a_1}{A_1}\sqrt{2gh_1} + \frac{a_3}{A_1}\sqrt{2gh_3} + \frac{\gamma_1 k_1}{A_1}v_1$$

$$\frac{dh_2}{dt} = -\frac{a_2}{A_2}\sqrt{2gh_2} + \frac{a_4}{A_2}\sqrt{2gh_4} + \frac{\gamma_2 k_2}{A_2}v_2$$

$$\frac{dh_3}{dt} = -\frac{a_3}{A_3}\sqrt{2gh_3} + \frac{(1-\gamma_2) k_2}{A_3}v_2$$

$$\frac{dh_4}{dt} = -\frac{a_4}{A_4}\sqrt{2gh_4} + \frac{(1-\gamma_1) k_1}{A_4}v_1$$

**Variables & Parameters:**
* $h_i$: The water level in tank $i$.
* $A_i$: The cross-sectional area of tank $i$.
* $a_i$: The cross-sectional area of the outlet hole for tank $i$.
* $v_j$: The input voltage applied to pump $j$.
* $k_j$: The proportionality constant for pump $j$, correlating voltage to flow.
* $\gamma_j$: The valve ratio in the range $(0, 1)$, determining how the flow from a pump is split between the upper and lower tanks.

## 2. Constructing the Lab Model

To physically construct this system using the specified equipment:

* **Tanks & Weight Sensing:** Use four identical plastic bottles to act as the tanks. Mount each bottle securely on an independent force cell (load cell). Ensure the force cells are tared with the empty bottles.
* **Deriving Level from Weight:** Because the density of water ($\rho$) and the cross-sectional area of the bottles ($A$) are constant, you can convert the weight ($W$) measured by the force cells directly into the water level ($h$) using the formula $W = \rho \cdot A \cdot h \cdot g$.
* **Pumps & Plumbing:** Submerge two water pumps in a lower reservoir basin.
    * Route a tube from Pump 1 to a 3-way manual bypass valve. Send one output to Bottle 1 ($\gamma_1$) and the other to Bottle 4 ($1-\gamma_1$).
    * Route a tube from Pump 2 to a 3-way manual bypass valve. Send one output to Bottle 2 ($\gamma_2$) and the other to Bottle 3 ($1-\gamma_2$).
* **Drainage:** Drill small holes at the bottom of the bottles. Mount Bottle 3 directly above Bottle 1, and Bottle 4 directly above Bottle 2 so they drain via gravity into the lower tanks.

## 3. System Identification (Parameter Estimation)

Before designing the controller, identify the unknown physical parameters ($a_i, k_j, \gamma_j$):

1. **Estimating the Outlet Areas ($a_i$):**
    * Fill all four bottles with water and shut off both pumps.
    * Record the continuous weight (level) decay over time as the tanks drain.
    * Because the drainage rate is $dh_i/dt = -(a_i/A_i)\sqrt{2gh_i}$ (when inputs are zero), use linear regression on the curve of $\sqrt{h_i}$ over time to extract the exact values for each outlet area $a_i$.
2. **Estimating Pump Gains ($k_j$) and Valve Settings ($\gamma_j$):**
    * Use sequences of step changes in the input voltages to appropriately excite the multivariable process.
    * Measure the steady-state inflows to the tanks and use a nonlinear least-squares optimization to fit the pump constants and valve ratios against the mathematical model.

## 4. Analysis of the Quadruple-Tank Dynamics

The quadruple-tank process features a tunable multivariable zero dictated by the bypass valves ($\gamma_1, \gamma_2$).

* **Minimum Phase Regime:** If $1 < \gamma_1 + \gamma_2 < 2$, the system's zero is located in the left half-plane. This occurs when the majority of the water is routed directly to the lower tanks. This regime is relatively easy to control.
* **Non-Minimum Phase Regime:** If $0 < \gamma_1 + \gamma_2 < 1$, the system is non-minimum phase, meaning the zero moves into the right half-plane. This happens when most of the water is routed to the upper tanks, creating an inverse response. A right half-plane zero places severe fundamental limitations on the achievable bandwidth of the feedback controller.
* **Coupling & Pairing:** The Relative Gain Array (RGA), a measure of interaction, is purely a function of the valves: $\lambda = \frac{\gamma_1\gamma_2}{\gamma_1 + \gamma_2 - 1}$. Use this to determine the optimal input-output pairing for decentralized control.

## 5. Controller Design

Because the pumps influence both measured tank levels, the system is strongly coupled. Standard approaches include:

1. **Decentralized PI Control:** Use two single-loop PI controllers. This is simple, but the controllers usually have to be significantly "detuned" to maintain stability due to multivariable interactions and RHP zero limitations.
2. **Internal Model Control (IMC):** Uses a plant model directly in the control law. For the non-minimum phase configuration, an "inner-outer" factorization is required to invert only the minimum-phase portion of the model.
3. **$H_\infty$ Robust Control:** Explictly accounts for plant-model uncertainties (like unmodeled friction or valve errors) to guarantee both robust stability and robust performance.

## 6. MATLAB Simulation

```matlab
% Quadruple Tank Process Simulation with Decentralized PI Control
clear; clc; close all;

% --- Physical Parameters ---
A = [28; 32; 28; 32];      % Cross-sectional areas of tanks 1-4 (cm^2)
a = [0.071; 0.057; 0.071; 0.057]; % Outlet areas of tanks 1-4 (cm^2)
k = [3.33; 3.35];          % Pump constants (cm^3/Vs)
g = 981;                   % Gravity (cm/s^2)

% Valve settings (Minimum Phase setting: gamma1 + gamma2 > 1)
gamma1 = 0.70; 
gamma2 = 0.60;

% --- Simulation Setup ---
Tsim = 1000;               % Simulation time (seconds)
dt = 0.1;                  % Step size
N = Tsim/dt;               
t = 0:dt:Tsim-dt;          

% State initialization: h = [h1; h2; h3; h4]
h = zeros(4, N);
h(:,1) = [12.4; 12.7; 1.8; 1.4]; % Initial steady-state levels (cm)

% Setpoints for tanks 1 and 2
r1 = 15 * ones(1, N);      
r2 = 15 * ones(1, N);      

% Controller Initialization (PI Parameters)
Kp1 = 3.0; Ti1 = 30;
Kp2 = 2.7; Ti2 = 40;

error_int = [0; 0]; % Integral of errors
v = zeros(2, N);    % Pump voltages

% --- Simulation Loop ---
for i = 1:N-1
    
    % Calculate Errors (Setpoint - Current Level)
    e1 = r1(i) - h(1, i);
    e2 = r2(i) - h(2, i);
    
    % Update Integrals
    error_int(1) = error_int(1) + e1 * dt;
    error_int(2) = error_int(2) + e2 * dt;
    
    % PI Control Law (Inputs bounded between 0 and 10 Volts)
    v(1, i) = max(0, min(10, Kp1 * (e1 + (1/Ti1) * error_int(1))));
    v(2, i) = max(0, min(10, Kp2 * (e2 + (1/Ti2) * error_int(2))));
    
    % Non-linear ODEs (Euler Integration)
    dh1 = -(a(1)/A(1))*sqrt(2*g*h(1,i)) + (a(3)/A(1))*sqrt(2*g*h(3,i)) + (gamma1*k(1)/A(1))*v(1,i);
    dh2 = -(a(2)/A(2))*sqrt(2*g*h(2,i)) + (a(4)/A(2))*sqrt(2*g*h(4,i)) + (gamma2*k(2)/A(2))*v(2,i);
    dh3 = -(a(3)/A(3))*sqrt(2*g*h(3,i)) + ((1-gamma2)*k(2)/A(3))*v(2,i);
    dh4 = -(a(4)/A(4))*sqrt(2*g*h(4,i)) + ((1-gamma1)*k(1)/A(4))*v(1,i);
    
    % Update states (ensure levels don't drop below 0)
    h(1, i+1) = max(0, h(1, i) + dh1 * dt);
    h(2, i+1) = max(0, h(2, i) + dh2 * dt);
    h(3, i+1) = max(0, h(3, i) + dh3 * dt);
    h(4, i+1) = max(0, h(4, i) + dh4 * dt);
end

% --- Plotting ---
figure;
subplot(2,1,1);
plot(t, h(1,:), 'b', t, r1, 'b--', 'LineWidth', 1.5); hold on;
plot(t, h(2,:), 'r', t, r2, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Water Level (cm)');
title('Quadruple Tank Process - PI Control');
legend('Tank 1 Level', 'Tank 1 Setpoint', 'Tank 2 Level', 'Tank 2 Setpoint', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, v(1,:), 'b', t, v(2,:), 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Pump Voltage (V)');
legend('Pump 1', 'Pump 2', 'Location', 'best');
grid on;