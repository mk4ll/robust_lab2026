"""
Frequency response analysis from 3 Arduino serial logs (one per Rv).
Each log is parsed into per-frequency blocks, a sinusoid is least-squares
fitted to the steady-state portion of each block, and a 1st-order model
G(s) = K/(τs+1) is optionally fitted to the resulting (|G|, ∠G) points.

All three datasets are overlaid on a single Bode plot for comparison
across operating conditions of the load resistor Rv.

Output is in pu/pu:  u_pu  = δV/PWM_RANGE   (already done in firmware)
                     y_pu  = δΩ/Ω*          (Ω* = steady-state RPM @ PWM=200)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

# ============================================================
#  CONFIG — edit these
# ============================================================
# (label, filename, color, omega_star [RPM])
# omega_star = steady-state RPM at the operating point (PWM=200),
# από τη στατική ταυτοποίηση (κίτρινο κελί στο Excel).
INPUT_FILES = [
    ('R_nominal', 'Omega_RNom.txt', 'tab:blue',  12.6),
    ('R_max',     'Omega_RMax.txt', 'tab:red',   13.1),
    ('R_min',     'Omega_RMin.txt', 'tab:green', 11.7),
]

# Input per-unit amplitude (must match AMP_PU in your Arduino sketch)
A_IN_PU = 0.3

# Drop unreliable high-frequency points
OMEGA_KEEP_BELOW = None    # e.g. 12.0  [rad/s]

# Fit a 1st-order model to each dataset and overlay
FIT_FIRST_ORDER = True

# Show one time-domain figure per file (set False to skip)
SHOW_TIME_DOMAIN = True

GAIN_UNITS = 'pu/pu'
# ============================================================


# ----------------------------------------------------------------
#  Parsing & fitting helpers
# ----------------------------------------------------------------
def parse_blocks(text):
    """Yield (omega, t, u_pu, rpm) numpy arrays for each frequency block."""
    blocks = text.split('Omega = ')
    for block in blocks[1:]:
        lines = block.strip().split('\n')
        try:
            omega = float(lines[0].strip())
        except (ValueError, IndexError):
            continue

        t, u_pu, rpm = [], [], []
        for line in lines[1:]:
            parts = line.split(',')
            if len(parts) != 4:                   # Time_s, u_pwm, u_pu, RPM
                continue
            try:
                t.append(float(parts[0]))
                u_pu.append(float(parts[2]))
                rpm.append(float(parts[3]))
            except ValueError:
                continue

        if len(t) < 10:
            continue
        yield omega, np.asarray(t), np.asarray(u_pu), np.asarray(rpm)


def fit_sinusoid(t, y, omega):
    """LS fit y(t) ≈ a0 + a1·sin(ωt) + a2·cos(ωt). Returns (a0, a1, a2)."""
    M = np.column_stack([np.ones_like(t), np.sin(omega*t), np.cos(omega*t)])
    coeffs, *_ = np.linalg.lstsq(M, y, rcond=None)
    return coeffs


def process_file(filename, omega_star):
    """Read one log file and extract a list of frequency-response points
    expressed in pu/pu (output normalized by omega_star)."""
    try:
        with open(filename, 'r') as f:
            text = f.read()
    except FileNotFoundError:
        print(f"  WARNING: {filename} not found — skipping.")
        return []

    results = []
    for omega, t, u_pu, rpm in parse_blocks(text):
        if OMEGA_KEEP_BELOW is not None and omega > OMEGA_KEEP_BELOW:
            continue

        half = len(t) // 2
        a0, a1, a2 = fit_sinusoid(t[half:], rpm[half:], omega)

        A_out_rpm = float(np.hypot(a1, a2))

        # ---- Per-unit conversion ----
        # y_pu = δΩ / Ω*   ⇒   |G|_pu/pu = (A_out_rpm/Ω*) / A_in_pu
        A_out_pu = A_out_rpm / omega_star
        gain = A_out_pu / A_IN_PU

        mag_db = 20*np.log10(gain) if gain > 0 else np.nan
        phase_deg = float(np.degrees(np.arctan2(a2, a1)))

        results.append(dict(
            omega=omega, t=t, u_pu=u_pu, rpm=rpm,
            a0=a0, a1=a1, a2=a2,
            gain=gain, mag_db=mag_db, phase_deg=phase_deg,
            omega_star_used=omega_star,
        ))
    return results


def fit_first_order(results):
    """Fit G(s) = K/(τs+1) to (|G|, ∠G) measurements. Returns (K, τ)."""
    if len(results) < 3:
        return None, None
    om = np.array([r['omega'] for r in results])
    G_meas = np.array([
        r['gain'] * np.exp(1j*np.radians(r['phase_deg'])) for r in results
    ])

    def residuals(params):
        K, tau = params
        diff = K / (1j*om*tau + 1.0) - G_meas
        return np.concatenate([diff.real, diff.imag])

    K0   = float(np.abs(G_meas[np.argmin(om)]))
    sol  = least_squares(residuals, [K0, 0.3],
                         bounds=([0, 1e-4], [np.inf, 100]))
    return sol.x


# ----------------------------------------------------------------
#  Process all files
# ----------------------------------------------------------------
all_data   = {}    # label -> (results, color)
all_fits   = {}    # label -> (K, tau)

for label, filename, color, omega_star in INPUT_FILES:
    print(f"\n=== {label}  ({filename})   Ω* = {omega_star} RPM ===")
    results = process_file(filename, omega_star)
    if not results:
        continue

    for r in results:
        print(f"  ω = {r['omega']:6.3f} rad/s   "
              f"|G| = {r['gain']:7.4f} {GAIN_UNITS}   "
              f"∠G = {r['phase_deg']:7.1f}°   (Ω_meas ≈ {r['a0']:.2f} RPM)")

    if FIT_FIRST_ORDER:
        K, tau = fit_first_order(results)
        all_fits[label] = (K, tau)
        if K is not None:
            print(f"  → fit:  K = {K:.4f} {GAIN_UNITS},  "
                  f"τ = {tau:.4f} s,  ω_c = {1/tau:.2f} rad/s")

    all_data[label] = (results, color)


# ----------------------------------------------------------------
#  Summary table
# ----------------------------------------------------------------
if FIT_FIRST_ORDER and any(v[0] is not None for v in all_fits.values()):
    print("\n" + "="*60)
    print(f"{'Label':<14}{'K ('+GAIN_UNITS+')':<14}"
          f"{'τ (s)':<10}{'ω_c (rad/s)':<14}")
    print("-"*60)
    for label in all_data:
        K, tau = all_fits.get(label, (None, None))
        if K is not None:
            print(f"{label:<14}{K:<14.4f}{tau:<10.4f}{1/tau:<14.2f}")
    print("="*60)


# ----------------------------------------------------------------
#  Time-domain plots (one figure per file)
# ----------------------------------------------------------------
if SHOW_TIME_DOMAIN:
    for label, (results, color) in all_data.items():
        n = len(results)
        fig, axs = plt.subplots(n, 1, figsize=(10, 2.4*n))
        if n == 1:
            axs = [axs]

        for ax, r in zip(axs, results):
            omega = r['omega']
            fit = (r['a0'] + r['a1']*np.sin(omega*r['t'])
                          + r['a2']*np.cos(omega*r['t']))

            ax_in  = ax
            ax_out = ax.twinx()
            ax_in.plot(r['t'],  r['u_pu'], 'b-',
                       label='u(t) [pu]', alpha=0.8)
            ax_out.plot(r['t'], r['rpm'],  color=color,
                        label='RPM (raw)', alpha=0.6)
            ax_out.plot(r['t'], fit, 'k--',
                        label='sine fit', linewidth=1.5)

            ax_in.set_xlabel('t (s)')
            ax_in.set_ylabel('u (pu)', color='b')
            ax_out.set_ylabel('RPM',   color=color)
            ax_in.set_title(rf'$\omega$ = {omega} rad/s   '
                            rf'($|G|$={r["gain"]:.3f} {GAIN_UNITS}, '
                            rf'$\angle G$={r["phase_deg"]:.1f}°)')
            ax_in.grid(True, alpha=0.3)

            lines = ax_in.get_lines() + ax_out.get_lines()
            ax_in.legend(lines, [l.get_label() for l in lines],
                         loc='upper right', fontsize=8)

        fig.suptitle(f'Time-Domain Responses — {label}   '
                     rf'($\Omega^* = {results[0]["omega_star_used"]}$ RPM)',
                     fontsize=14, fontweight='bold')
        fig.tight_layout(rect=[0, 0, 1, 0.98])


# ----------------------------------------------------------------
#  Combined Bode plot (all Rv overlaid)
# ----------------------------------------------------------------
fig2, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)

# Find global ω range for the fit overlay curves
all_omegas = np.concatenate([
    np.array([r['omega'] for r in results])
    for results, _ in all_data.values()
]) if all_data else np.array([1.0])
om_dense = np.logspace(np.log10(all_omegas.min()*0.3),
                       np.log10(all_omegas.max()*3), 400)

for label, (results, color) in all_data.items():
    om = np.array([r['omega']     for r in results])
    mg = np.array([r['mag_db']    for r in results])
    ph = np.array([r['phase_deg'] for r in results])
    idx = np.argsort(om)
    om, mg, ph = om[idx], mg[idx], ph[idx]

    ax_m.semilogx(om, mg, 'o', color=color, markersize=9,
                  label=f'{label} (measured)', zorder=3)
    ax_p.semilogx(om, ph, 's', color=color, markersize=9,
                  label=f'{label} (measured)', zorder=3)

    K, tau = all_fits.get(label, (None, None))
    if K is not None:
        G = K / (1j*om_dense*tau + 1.0)
        fit_label = (rf'{label} fit: $K$={K:.3f}, '
                     rf'$\tau$={tau:.3f}s')
        ax_m.semilogx(om_dense, 20*np.log10(np.abs(G)),
                      '-', color=color, linewidth=2, alpha=0.6,
                      label=fit_label)
        ax_p.semilogx(om_dense, np.degrees(np.angle(G)),
                      '-', color=color, linewidth=2, alpha=0.6,
                      label=fit_label)

ax_m.set_ylabel(rf'$|G(j\omega)|$  [dB,  {GAIN_UNITS}]')
ax_m.set_title('Bode Diagram — Frequency Response Validation across $R_v$',
               fontsize=14, fontweight='bold')
ax_m.grid(True, which='both', alpha=0.5)
ax_m.legend(fontsize=9)

ax_p.set_xlabel(r'$\omega$  (rad/s)')
ax_p.set_ylabel(r'$\angle G(j\omega)$  (deg)')
ax_p.grid(True, which='both', alpha=0.5)
ax_p.legend(fontsize=9)

plt.tight_layout()
plt.show()