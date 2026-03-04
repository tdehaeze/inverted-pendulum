#!/usr/bin/env python3
"""
Trace la dernière séquence de contrôle actif d'un fichier CSV d'expérience.

Usage :
    python tools/plot_experiment.py data/kp_12_kd_1_stable.csv
    python tools/plot_experiment.py data/run.csv --margin 2.0 --upright 180.0
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def last_feedback_segment(df, margin):
    """Retourne le sous-DataFrame couvrant la dernière séquence feedback=1,
    étendue de `margin` secondes avant et après."""
    fb = df["feedback_on"].values
    t  = df["t_s"].values

    # Indices où feedback est actif
    on_idx = np.where(fb == 1)[0]
    if len(on_idx) == 0:
        print("Aucune séquence de contrôle actif trouvée dans le fichier.")
        sys.exit(1)

    # Détection des coupures (saut > 0.1 s entre deux échantillons actifs)
    gaps = np.where(np.diff(t[on_idx]) > 0.1)[0]
    # Dernier segment : commence après le dernier saut (ou dès le début)
    seg_start_idx = on_idx[gaps[-1] + 1] if len(gaps) > 0 else on_idx[0]
    seg_end_idx   = on_idx[-1]

    t_start = t[seg_start_idx] - margin
    t_end   = t[seg_end_idx]   + margin

    mask = (t >= t_start) & (t <= t_end)
    return df[mask].copy(), t[seg_start_idx], t[seg_end_idx]


def main():
    parser = argparse.ArgumentParser(
        description="Trace la dernière séquence de contrôle actif")
    parser.add_argument("csv",        help="Fichier CSV produit par le GUI")
    parser.add_argument("--margin",   type=float, default=1.0,
                        help="Marge avant/après la séquence [s]  (défaut : 1.0)")
    parser.add_argument("--upright",  type=float, default=180.0,
                        help="Angle de référence position haute [deg]  (défaut : 180)")
    parser.add_argument("--ylim-phi", type=float, default=None, metavar="DEG",
                        help="Limite ±DEG degrés sur l'axe φ  (ex: 5)")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    seg, t_on, t_off = last_feedback_segment(df, args.margin)

    # Temps relatif au début du segment affiché
    t0  = seg["t_s"].iloc[0]
    t   = seg["t_s"].values - t0
    t_on_rel  = t_on  - t0
    t_off_rel = t_off - t0

    phi     = seg["angle_deg"].values - args.upright   # déviation par rapport à la verticale
    cart_mm = seg["cart_mm"].values

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # ---- Angle ----
    axes[0].plot(t, phi, lw=1.0, color="steelblue")
    axes[0].axhline(0, color="k", lw=0.6, ls="--")
    axes[0].axvspan(t_on_rel, t_off_rel, alpha=0.08, color="green",
                    label="Contrôle actif")
    axes[0].set_ylabel("Angle du pendule φ [deg]")
    axes[0].legend(loc="upper right", fontsize=8)
    if args.ylim_phi is not None:
        axes[0].set_ylim(-args.ylim_phi, args.ylim_phi)
    axes[0].grid(True)

    # ---- Position chariot ----
    axes[1].plot(t, cart_mm, lw=1.0, color="tomato")
    axes[1].axhline(0, color="k", lw=0.6, ls="--")
    axes[1].axvspan(t_on_rel, t_off_rel, alpha=0.08, color="green")
    axes[1].set_ylabel("Position du chariot [mm]")
    axes[1].set_xlabel("Temps [s]")
    axes[1].grid(True)

    basename = os.path.splitext(os.path.basename(args.csv))[0]
    fig.suptitle(f"Expérience — {basename}", fontsize=11)
    fig.tight_layout()

    os.makedirs("figs", exist_ok=True)
    out = os.path.join("figs", basename + ".png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Figure sauvegardée → {out}")


if __name__ == "__main__":
    main()
