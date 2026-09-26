#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Training curves from train_velocity.py (train_log.csv), in the style of the MicroDuck plots.

Smoothed curves, "Iterazione PPO" on the x axis, signature at the bottom, as in
NOESIS EXPERIMENT experiments/run_microduck_cartan_bench.py. Panels follow the
rsl_rl metrics logged for the MicroDuck training (W&B mjlab_microduck).

    python tools/robots/plot_training.py robots/microban/policies/walk_ap_run/train_log.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

SIGNATURE = "Realizzati da Roberto Navoni — DelphyAI LAB"


def smooth(x: np.ndarray, w: int = 11) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    ok = np.isfinite(x)
    if ok.sum() < w:
        return x
    xi = np.interp(np.arange(len(x)), np.flatnonzero(ok), x[ok])
    k = np.ones(w) / w
    pad = np.pad(xi, (w // 2, w - 1 - w // 2), mode="edge")
    return np.convolve(pad, k, mode="valid")


def plot_run(csv_path: Path, out: Path, title: str = "", eval_report: dict | None = None) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = list(csv.DictReader(Path(csv_path).open()))
    cols = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    it = cols["iteration"]
    panels = [
        ("Train/mean_reward", ["Train/mean_reward"], "Return / episodio"),
        ("Train/mean_episode_length", ["Train/mean_episode_length"], "Durata episodio (passi a 50 Hz)"),
        ("Episode_Termination", ["Episode_Termination/fell_over", "Episode_Termination/time_out"],
         "Fine episodio per passo"),
        ("Metrics/twist", ["Metrics/twist/error_vel_xy", "Metrics/twist/error_vel_yaw"], "Errore di tracking"),
        ("Episode_Reward", [k for k in cols if k.startswith("Episode_Reward/")], "Termini di reward"),
        ("Loss", ["Loss/value", "Loss/surrogate"], "Policy / value loss"),
        ("Policy", ["Policy/mean_std", "Loss/learning_rate"], "Rumore di esplorazione e learning rate"),
        ("Curriculum", ["Curriculum/action_rate_weight", "Curriculum/standing_envs"], "Curriculum"),
    ]
    fig, axes = plt.subplots(2, 4, figsize=(18, 8))
    for ax, (_, keys, ttl) in zip(axes.ravel(), panels):
        for k in keys:
            if k not in cols:
                continue
            y = cols[k]
            ax.plot(it, y, alpha=0.2, lw=0.8)
            ax.plot(it, smooth(y), lw=1.8, label=k.split("/", 1)[-1], color=ax.lines[-1].get_color())
        if ttl.startswith("Policy /"):
            ax.set_yscale("symlog", linthresh=1e-3)
        if ttl.startswith("Rumore"):
            ax.set_yscale("log")
        ax.set_title(ttl)
        ax.set_xlabel("Iterazione PPO")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    if eval_report:
        txt = "   ".join(f"{mode}: sopravvivenza {m['survival'] * 100:.0f}%, vx {m['mean_vx']:+.2f} "
                         f"(cmd {m['command_vx']:+.1f})" for mode, m in eval_report.items())
        fig.text(0.5, 0.035, "Valutazione rete int8 distribuita — " + txt, ha="center", fontsize=9)
    fig.suptitle(title, fontsize=12)
    fig.text(0.5, 0.008, SIGNATURE, ha="center", fontsize=8, style="italic", color="#444444")
    fig.tight_layout(rect=(0, 0.05, 1, 0.96))
    out = Path(out)
    fig.savefig(out, dpi=140)
    plt.close(fig)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--title", default="")
    args = ap.parse_args()
    out = args.out or args.csv.parent / "training_curves.png"
    print(plot_run(args.csv, out, args.title))


if __name__ == "__main__":
    main()
