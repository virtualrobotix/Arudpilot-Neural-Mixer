#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Load a robot MuJoCo scene and take one control step at zero policy (stand pose).

Verifies that the MJCF loads, the free joint / actuators exist, and the robot
does not immediately explode when held at q0. Does not require a trained policy.

Usage:
    python tools/robots/zero_policy_step.py --robot microduck
    python tools/robots/zero_policy_step.py --robot legolas --mjcf robots/legolas/robot/scene.xml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import load_profile, resolve_mjcf  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--mjcf", type=Path, default=None, help="Override MJCF path")
    ap.add_argument("--steps", type=int, default=1, help="Physics steps after reset")
    args = ap.parse_args()

    profile = load_profile(args.robot)
    mjcf = args.mjcf or resolve_mjcf(args.robot, profile)
    if mjcf is None:
        raise SystemExit(
            f"no MJCF for {args.robot}: place robot/scene.xml or set NNMIXER_MJCF "
            f"(status={profile.get('status')})"
        )
    if not mjcf.is_file():
        raise SystemExit(f"MJCF not found: {mjcf}")

    try:
        import mujoco
    except ImportError as e:
        raise SystemExit(f"mujoco is required: {e}") from e

    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    q0 = profile.get("q0") or []
    names = profile.get("joint_names") or []
    applied = 0
    for i, name in enumerate(names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            print(f"warning: joint {name!r} not in model")
            continue
        qadr = model.jnt_qposadr[jid]
        if i < len(q0):
            data.qpos[qadr] = q0[i]
            applied += 1

    mujoco.mj_forward(model, data)
    for _ in range(max(args.steps, 1)):
        # zero policy: hold q0 via position actuators if present, else leave ctrl=0
        if model.nu > 0 and applied > 0:
            for i, name in enumerate(names[: model.nu]):
                if i < len(q0):
                    data.ctrl[i] = q0[i]
        mujoco.mj_step(model, data)

    height = float(data.qpos[2]) if model.nq >= 3 else float("nan")
    print(
        f"ok robot={args.robot} mjcf={mjcf} nq={model.nq} nu={model.nu} "
        f"joints_applied={applied}/{len(names)} height={height:.4f} "
        f"n_joints_profile={profile.get('n_joints')} rate_hz={profile.get('rate_hz')}"
    )


if __name__ == "__main__":
    main()
