#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Entrypoint for MjLab velocity training of a catalog robot.

Full PPO training is a separate GPU job (2048 envs, thousands of iterations).
This script validates the robot profile / MJCF and prints the command line that
matches the MicroDuck mjlab + rsl_rl workflow. It does not download weights.

Usage:
    python tools/robots/train_velocity.py --robot microban
    python tools/robots/train_velocity.py --robot microduck --dry-run
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import expected_obs_dim, load_profile, resolve_mjcf  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--dry-run", action="store_true", help="Only print the planned train recipe")
    ap.add_argument("--envs", type=int, default=2048)
    ap.add_argument("--iters", type=int, default=2000)
    args = ap.parse_args()

    profile = load_profile(args.robot)
    mjcf = resolve_mjcf(args.robot, profile)
    n_joints = int(profile.get("n_joints") or 0)
    obs = int(profile.get("obs_dim") or 0)
    if n_joints and not obs:
        obs = expected_obs_dim(n_joints)
    rate = int(profile.get("rate_hz") or 50)

    print(f"robot_id={args.robot}")
    print(f"status={profile.get('status')}")
    print(f"mjcf={mjcf}")
    print(f"n_joints={n_joints} obs_dim={obs} rate_hz={rate}")
    print(f"upstream={profile.get('upstream')}")
    print()
    print("Planned MjLab velocity recipe (run on a GPU host with mjlab + rsl_rl):")
    print(f"  1. Load MJCF as mjlab env with domain randomization")
    print(f"  2. Observation = gyro, gravity, q-q0, qd, prev_action, twist (+ head/body if any)")
    print(f"  3. Reward = track commanded vx/vy/wz, upright, energy, fall penalty")
    print(f"  4. PPO {args.envs} envs x {args.iters} iters, bake EmpiricalNormalization into ONNX")
    print(f"  5. Write ONNX to robots/{args.robot}/policies/<run>.onnx")
    print(f"  6. python tools/robots/export_nnm.py <onnx> --robot {args.robot}")
    print()

    if profile.get("status") == "needs-mjcf" and mjcf is None:
        raise SystemExit(
            f"refusing to start training: {args.robot} has no local MJCF yet "
            f"(add robots/{args.robot}/robot/scene.xml)"
        )

    if args.dry_run or True:
        # Training is intentionally not launched from this laptop entrypoint.
        print("dry-run complete (GPU train job not started from this script).")
        return


if __name__ == "__main__":
    main()
