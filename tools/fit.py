#!/usr/bin/env python3
"""
Fit pendulum model parameters (l, c) to measured free-response data.

The cart is assumed stationary (u = 0), so only the pendulum ODE is used:
    (I + m*l²) * θ̈ + c * θ̇ + m*g*l * sin(θ) = 0

The fit starts at the first peak of the oscillation, where ω₀ ≈ 0 and
the initial condition is simply θ₀ = peak amplitude.

Usage:
    python tools/fit.py data/free_response.csv
    python tools/fit.py data/free_response.csv --mass 0.15 --t-start 2.4
    python tools/fit.py data/free_response.csv --l-init 0.20 --c-init 0.005

Requires: numpy, scipy, matplotlib  (source model/.venv/bin/activate)
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize
from scipy.signal import find_peaks

G = 9.81


# =============================================================
# DATA LOADING
# =============================================================

def load_csv(path: str):
    t, angle = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_s"]))
            angle.append(float(row["angle_deg"]))
    t = np.array(t)
    t -= t[0]
    return t, np.deg2rad(np.array(angle))


# =============================================================
# SIMULATION  (fixed cart, u = 0)
# =============================================================

def simulate_free(t_eval, theta0, omega0, l, c, m):
    I = (1 / 12) * m * (2 * l) ** 2
    D = I + m * l ** 2          # effective inertia about pivot = 4/3 * m * l²

    def ode(t, y):
        theta, omega = y
        domega = (-c * omega - m * G * l * np.sin(theta)) / D
        return [omega, domega]

    sol = solve_ivp(ode, (t_eval[0], t_eval[-1]), [theta0, omega0],
                    t_eval=t_eval, method="RK45", rtol=1e-7, atol=1e-9,
                    dense_output=False)
    if sol.y.shape[1] != len(t_eval):
        return np.full(len(t_eval), np.nan)
    return sol.y[0]


# =============================================================
# FITTING
# =============================================================

def cost(params, t, theta_meas, m):
    l, c = params
    if l <= 0 or c <= 0:
        return 1e10
    theta_sim = simulate_free(t, theta_meas[0], 0.0, l, c, m)
    if np.any(np.isnan(theta_sim)):
        return 1e10
    return float(np.mean((theta_sim - theta_meas) ** 2))


def fit(t, theta, m, l_init, c_init):
    result = minimize(cost, [l_init, c_init], args=(t, theta, m),
                      method="Nelder-Mead",
                      options={"xatol": 1e-6, "fatol": 1e-10, "maxiter": 10000})
    return result.x, result.fun


# =============================================================
# MAIN
# =============================================================

def main():
    parser = argparse.ArgumentParser(
        description="Fit pendulum parameters to free-response CSV data")
    parser.add_argument("csv",        help="CSV file from capture.py")
    parser.add_argument("--mass",     type=float, default=0.1,
                        help="Pendulum mass m [kg]  (default: 0.1)")
    parser.add_argument("--t-start",  type=float, default=None,
                        help="Force start time [s] instead of auto-detecting first peak")
    parser.add_argument("--l-init",   type=float, default=0.20,
                        help="Initial guess for l [m]  (default: 0.20)")
    parser.add_argument("--c-init",   type=float, default=0.01,
                        help="Initial guess for c [N·m·s/rad]  (default: 0.01)")
    args = parser.parse_args()

    # ---- Load ----
    print(f"Loading {args.csv} …")
    t_full, theta_full = load_csv(args.csv)
    print(f"  {len(t_full)} samples, {t_full[-1]:.2f} s, "
          f"{len(t_full)/t_full[-1]:.0f} Hz")

    # ---- Find fit start: first positive peak ----
    if args.t_start is not None:
        idx0 = np.searchsorted(t_full, args.t_start)
    else:
        peaks, _ = find_peaks(theta_full, height=np.deg2rad(5))
        if len(peaks) == 0:
            print("ERROR: no peaks found — use --t-start to set the start manually.")
            sys.exit(1)
        idx0 = peaks[0]
        print(f"  First peak at t = {t_full[idx0]:.3f} s, "
              f"θ₀ = {np.rad2deg(theta_full[idx0]):.1f}°")

    # ---- Crop and downsample (every 5th sample → ~200 Hz) ----
    t     = t_full[idx0::5]  - t_full[idx0]
    theta = theta_full[idx0::5]

    print(f"  Fitting on {len(t)} samples over {t[-1]:.2f} s …")

    # ---- Optimise ----
    (l_fit, c_fit), final_cost = fit(t, theta, args.mass, args.l_init, args.c_init)

    # ---- Derived quantities ----
    I   = (1 / 12) * args.mass * (2 * l_fit) ** 2
    D   = I + args.mass * l_fit ** 2
    w_n = np.sqrt(args.mass * G * l_fit / D)
    w_d_meas = 2 * np.pi / np.mean(np.diff(t_full[peaks if args.t_start is None
                                                   else find_peaks(theta_full,
                                                   height=np.deg2rad(5))[0]]))

    print(f"\nFitted parameters:")
    print(f"  l   = {l_fit * 100:.2f} cm")
    print(f"  c   = {c_fit:.5f} N·m·s/rad")
    print(f"  ω_n = {w_n:.3f} rad/s  ({w_n / (2*np.pi):.3f} Hz)")
    print(f"  RMS error = {np.sqrt(final_cost):.4f} rad ({np.rad2deg(np.sqrt(final_cost)):.2f}°)")

    # ---- Simulate with fitted params ----
    theta_fit = simulate_free(t, theta[0], 0.0, l_fit, c_fit, args.mass)

    # ---- Plot ----
    basename = os.path.splitext(os.path.basename(args.csv))[0]
    os.makedirs("figs", exist_ok=True)

    # Figure 1: experimental data only
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, np.rad2deg(theta), lw=2.0, color="steelblue", label="Mesuré")
    ax.set_xlabel("Temps [s]")
    ax.set_ylabel("Angle [deg]")
    ax.set_ylim(-30, 30)
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    png_data = os.path.join("figs", basename + "_data.png")
    fig.savefig(png_data, dpi=150)
    plt.close(fig)
    print(f"Plot saved → {png_data}")

    # Figure 2: experimental data + fit
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, np.rad2deg(theta), lw=2.0, color="steelblue", label="Mesuré")
    ax.plot(t, np.rad2deg(theta_fit), lw=1.0, ls="--", color="tomato",
            label=f"Modèle ajusté  l={2*l_fit*100:.1f} cm, c={c_fit:.5f} N·m·s/rad")
    ax.set_xlabel("Temps [s]")
    ax.set_ylabel("Angle [deg]")
    ax.set_ylim(-30, 30)
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    png_fit = os.path.join("figs", basename + "_fit.png")
    fig.savefig(png_fit, dpi=150)
    plt.close(fig)
    print(f"Plot saved → {png_fit}")


if __name__ == "__main__":
    main()
