#!/usr/bin/env python3
"""
Trace les dernières N secondes d'un fichier CSV d'expérience.
Conçu pour visualiser la stabilité en boucle fermée.

Usage :
    python tools/plot_stability.py data/run.csv
    python tools/plot_stability.py data/run.csv --duration 5 --ylim-phi 1 --ylim-x 50
    python tools/plot_stability.py data/run.csv --duration 10 --detrend-phi --detrend-x
"""

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import detrend


def main():
    parser = argparse.ArgumentParser(
        description="Trace les dernières N secondes d'une expérience")
    parser.add_argument("csv",            help="Fichier CSV produit par le GUI")
    parser.add_argument("--duration",     type=float, default=None, metavar="S",
                        help="Durée à afficher depuis la fin [s]  (défaut : tout)")
    parser.add_argument("--upright",      type=float, default=180.0, metavar="DEG",
                        help="Angle de référence position haute [deg]  (défaut : 180)")
    parser.add_argument("--ylim-phi",     type=float, default=None, metavar="DEG",
                        help="Limite ±DEG degrés sur l'axe φ")
    parser.add_argument("--ylim-x",       type=float, default=None, metavar="MM",
                        help="Limite ±MM mm sur l'axe position chariot")
    parser.add_argument("--detrend-phi",  action="store_true",
                        help="Supprimer la moyenne de φ")
    parser.add_argument("--detrend-x",    action="store_true",
                        help="Supprimer la moyenne de la position chariot")
    parser.add_argument("--no-shading",   action="store_true",
                        help="Ne pas afficher les zones vertes de contrôle actif")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    # Sélection des dernières N secondes
    t_all = df["t_s"].values
    if args.duration is not None:
        t_end   = t_all[-1]
        t_start = t_end - args.duration
        df = df[t_all >= t_start].copy()

    t0  = df["t_s"].iloc[0]
    t   = df["t_s"].values - t0

    phi     = df["angle_deg"].values - args.upright
    cart_mm = df["cart_mm"].values
    fb      = df["feedback_on"].values

    if args.detrend_phi:
        phi = detrend(phi, 0)
    if args.detrend_x:
        cart_mm = detrend(cart_mm, 0)

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # ---- Angle ----
    axes[0].plot(t, phi, lw=1.0, color="steelblue")
    axes[0].axhline(0, color="k", lw=0.6, ls="--")
    if not args.no_shading:
        _draw_spans(axes[0], t, fb)
    axes[0].set_ylabel("Angle du pendule φ [deg]")
    if args.ylim_phi is not None:
        axes[0].set_ylim(-args.ylim_phi, args.ylim_phi)
    axes[0].grid(True)

    # ---- Position chariot ----
    axes[1].plot(t, cart_mm, lw=1.0, color="tomato")
    axes[1].axhline(0, color="k", lw=0.6, ls="--")
    if not args.no_shading:
        _draw_spans(axes[1], t, fb)
    axes[1].set_ylabel("Position du chariot [mm]")
    axes[1].set_xlabel("Temps [s]")
    if args.ylim_x is not None:
        axes[1].set_ylim(-args.ylim_x, args.ylim_x)
    axes[1].grid(True)

    basename = os.path.splitext(os.path.basename(args.csv))[0]
    fig.tight_layout()

    os.makedirs("figs", exist_ok=True)
    out = os.path.join("figs", basename + "_stability.png")
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Figure sauvegardée → {out}")


def _draw_spans(ax, t, fb):
    """Trace les zones vertes là où feedback_on == 1."""
    in_span = False
    t_start = None
    first   = True
    for i, val in enumerate(fb):
        if val == 1 and not in_span:
            t_start = t[i]
            in_span = True
        elif val != 1 and in_span:
            ax.axvspan(t_start, t[i - 1], alpha=0.08, color="green",
                       label="Contrôle actif" if first else "")
            in_span = False
            first   = False
    if in_span:
        ax.axvspan(t_start, t[-1], alpha=0.08, color="green",
                   label="Contrôle actif" if first else "")

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    if by_label:
        ax.legend(by_label.values(), by_label.keys(),
                  loc="upper right", fontsize=8)


if __name__ == "__main__":
    main()
