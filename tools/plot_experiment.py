#!/usr/bin/env python3
"""
Trace les données d'expérience d'un fichier CSV.

Usage :
    python tools/plot_experiment.py data/run.csv
    python tools/plot_experiment.py data/run.csv --all --ylim-phi 5 --ylim-x 100 --detrend-x
    python tools/plot_experiment.py data/run.csv --margin 2.0 --no-shading
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import detrend


def last_feedback_segment(df, margin_before, margin_after):
    """Retourne le sous-DataFrame couvrant la dernière séquence feedback=1,
    étendue de `margin_before` secondes avant et `margin_after` secondes après."""
    fb = df["feedback_on"].values
    t  = df["t_s"].values

    on_idx = np.where(fb == 1)[0]
    if len(on_idx) == 0:
        print("Aucune séquence de contrôle actif trouvée dans le fichier.")
        sys.exit(1)

    gaps = np.where(np.diff(t[on_idx]) > 0.1)[0]
    seg_start_idx = on_idx[gaps[-1] + 1] if len(gaps) > 0 else on_idx[0]
    seg_end_idx   = on_idx[-1]

    t_start = t[seg_start_idx] - margin_before
    t_end   = t[seg_end_idx]   + margin_after

    mask = (t >= t_start) & (t <= t_end)
    return df[mask].copy()


def feedback_spans(t, fb):
    """Retourne la liste des intervalles (t_start, t_end) où feedback_on == 1."""
    spans = []
    in_span = False
    for i, val in enumerate(fb):
        if val == 1 and not in_span:
            t_start = t[i]
            in_span = True
        elif val != 1 and in_span:
            spans.append((t_start, t[i - 1]))
            in_span = False
    if in_span:
        spans.append((t_start, t[-1]))
    return spans


def main():
    parser = argparse.ArgumentParser(
        description="Trace les données d'expérience")
    parser.add_argument("csv",           help="Fichier CSV produit par le GUI")
    parser.add_argument("--all",         action="store_true",
                        help="Afficher toutes les données (sans extraction du dernier segment)")
    parser.add_argument("--margin",       type=float, default=1.0, metavar="S",
                        help="Marge avant ET après le dernier segment [s]  (défaut : 1.0)")
    parser.add_argument("--margin-after", type=float, default=None, metavar="S",
                        help="Marge après le dernier segment [s]  (écrase --margin)")
    parser.add_argument("--upright",     type=float, default=180.0, metavar="DEG",
                        help="Angle de référence position haute [deg]  (défaut : 180)")
    parser.add_argument("--ylim-phi",    type=float, default=None, metavar="DEG",
                        help="Limite ±DEG degrés sur l'axe φ")
    parser.add_argument("--ylim-x",      type=float, default=None, metavar="MM",
                        help="Limite ±MM mm sur l'axe position chariot")
    parser.add_argument("--detrend-x",   action="store_true",
                        help="Supprimer la moyenne (detrend constant) de la position chariot")
    parser.add_argument("--no-shading",  action="store_true",
                        help="Ne pas afficher les zones vertes de contrôle actif")
    parser.add_argument("--split",       action="store_true",
                        help="Sauvegarder deux figures séparées (_phi.png et _x.png)")
    parser.add_argument("--zero-at-feedback", action="store_true",
                        help="t=0 au démarrage du contrôle (au lieu du début du segment)")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    if args.all:
        seg = df.copy()
    else:
        margin_after = args.margin_after if args.margin_after is not None else args.margin
        seg = last_feedback_segment(df, args.margin, margin_after)

    if args.zero_at_feedback:
        fb_on_idx = np.where(seg["feedback_on"].values == 1)[0]
        t0 = seg["t_s"].iloc[fb_on_idx[0]] if len(fb_on_idx) > 0 else seg["t_s"].iloc[0]
    else:
        t0 = seg["t_s"].iloc[0]
    t = seg["t_s"].values - t0

    phi     = seg["angle_deg"].values - args.upright
    cart_mm = seg["cart_mm"].values

    if args.detrend_x:
        cart_mm = detrend(cart_mm, 0)   # type 0 = soustraction de la moyenne

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    # ---- Angle ----
    axes[0].plot(t, phi, lw=1.0, color="steelblue")
    axes[0].axhline(0, color="k", lw=0.6, ls="--")
    if not args.no_shading:
        for t_a, t_b in feedback_spans(t, seg["feedback_on"].values):
            axes[0].axvspan(t_a, t_b, alpha=0.08, color="green",
                            label="Contrôle actif")
        # Légende sans doublons
        handles, labels = axes[0].get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        axes[0].legend(by_label.values(), by_label.keys(),
                       loc="upper right", fontsize=8)
    axes[0].set_ylabel("Angle du pendule φ [deg]")
    if args.ylim_phi is not None:
        axes[0].set_ylim(-args.ylim_phi, args.ylim_phi)
    axes[0].grid(True)

    # ---- Position chariot ----
    axes[1].plot(t, cart_mm, lw=1.0, color="tomato")
    axes[1].axhline(0, color="k", lw=0.6, ls="--")
    if not args.no_shading:
        for t_a, t_b in feedback_spans(t, seg["feedback_on"].values):
            axes[1].axvspan(t_a, t_b, alpha=0.08, color="green")
    ylabel = "Position du chariot [mm]"
    if args.detrend_x:
        ylabel += " (détendancé)"
    axes[1].set_ylabel(ylabel)
    axes[1].set_xlabel("Temps [s]")
    if args.ylim_x is not None:
        axes[1].set_ylim(-args.ylim_x, args.ylim_x)
    axes[1].grid(True)

    fig.tight_layout()

    basename = os.path.splitext(os.path.basename(args.csv))[0]
    os.makedirs("figs", exist_ok=True)

    if args.split:
        # Save phi panel alone
        fig_phi, ax_phi = plt.subplots(figsize=(5, 2.5), tight_layout=True)
        ax_phi.plot(t, phi, lw=1.0, color="steelblue")
        ax_phi.axhline(0, color="k", lw=0.6, ls="--")
        if not args.no_shading:
            for t_a, t_b in feedback_spans(t, seg["feedback_on"].values):
                ax_phi.axvspan(t_a, t_b, alpha=0.08, color="green", label="Contrôle actif")
            handles, labels = ax_phi.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            ax_phi.legend(by_label.values(), by_label.keys(), loc="upper right", fontsize=8)
        ax_phi.set_ylabel("Angle du pendule φ [deg]")
        ax_phi.set_xlabel("Temps [s]")
        if args.ylim_phi is not None:
            ax_phi.set_ylim(-args.ylim_phi, args.ylim_phi)
        ax_phi.grid(True)
        out_phi = os.path.join("figs", basename + "_phi.png")
        fig_phi.savefig(out_phi, dpi=150)
        plt.close(fig_phi)
        print(f"Figure sauvegardée → {out_phi}")

        # Save x panel alone
        fig_x, ax_x = plt.subplots(figsize=(5, 2.5), tight_layout=True)
        ax_x.plot(t, cart_mm, lw=1.0, color="tomato")
        ax_x.axhline(0, color="k", lw=0.6, ls="--")
        if not args.no_shading:
            for t_a, t_b in feedback_spans(t, seg["feedback_on"].values):
                ax_x.axvspan(t_a, t_b, alpha=0.08, color="green")
        ax_x.set_ylabel("Position du chariot [mm]")
        ax_x.set_xlabel("Temps [s]")
        if args.ylim_x is not None:
            ax_x.set_ylim(-args.ylim_x, args.ylim_x)
        ax_x.grid(True)
        out_x = os.path.join("figs", basename + "_x.png")
        fig_x.savefig(out_x, dpi=150)
        plt.close(fig_x)
        print(f"Figure sauvegardée → {out_x}")

        plt.close(fig)
    else:
        out = os.path.join("figs", basename + ".png")
        fig.savefig(out, dpi=150)
        plt.close(fig)
        print(f"Figure sauvegardée → {out}")


if __name__ == "__main__":
    main()
