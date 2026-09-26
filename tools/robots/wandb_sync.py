#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Stream a train_velocity.py log (train_log.csv) to Weights & Biases.

For runs started without --wandb. With --follow it keeps reading the CSV
until the run writes <name>_final.nnm next to it, then uploads that file.

    python tools/robots/wandb_sync.py robots/microban/policies/walk_ap_run --robot microban --follow
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path, help="directory with train_log.csv")
    ap.add_argument("--robot", required=True)
    ap.add_argument("--project", default="ardupilot-nnmixer")
    ap.add_argument("--name", default=None)
    ap.add_argument("--rate-hz", type=float, default=50.0)
    ap.add_argument("--envs", type=int, default=0, help="for the env_steps axis")
    ap.add_argument("--steps-per-env", type=int, default=24)
    ap.add_argument("--follow", action="store_true")
    args = ap.parse_args()

    import wandb

    stem = args.run_dir.name.removesuffix("_run")
    run = wandb.init(project=args.project, name=args.name or f"{args.robot}-{stem}",
                     config={"robot": args.robot, "envs": args.envs, "source": str(args.run_dir)})
    csv_path = args.run_dir / "train_log.csv"
    final = args.run_dir / f"{stem}_final.nnm"
    sent = 0
    while True:
        rows = list(csv.DictReader(csv_path.open())) if csv_path.is_file() else []
        for row in rows[sent:]:
            it = int(row["iter"])
            ml = float(row["mean_ep_len"])
            run.log({"mean_ep_return": float(row["mean_ep_return"]), "mean_ep_len": ml,
                     "mean_ep_len_s": ml / args.rate_hz, "kl": float(row["kl"]), "lr": float(row["lr"]),
                     "action_std": float(row["std"]), "seconds": float(row["seconds"]),
                     "env_steps": it * args.envs * args.steps_per_env}, step=it)
        sent = len(rows)
        if not args.follow or final.is_file():
            break
        time.sleep(5)
    if final.is_file():
        art = wandb.Artifact(f"{args.robot}-{stem}", type="nnm-policy")
        art.add_file(str(final))
        run.log_artifact(art)
    print(f"synced {sent} iterations to {run.url}")
    run.finish()


if __name__ == "__main__":
    main()
