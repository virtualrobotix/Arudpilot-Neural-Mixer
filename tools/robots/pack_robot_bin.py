#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Pack robots/<id>/robot/profile.json into robot.bin for the microSD layout.

Layout (little-endian):
  magic[4] = b'NNMR'
  robot_id[16]
  n_joints u16, obs_dim u16, rate_hz u16, servo_fn0 u16
  q0[n_joints] f32
  joint_names as length-prefixed ASCII strings (u8 len + bytes), n_joints times

Usage:
    python tools/robots/pack_robot_bin.py --robot microduck \\
        --out robots/microduck/robot/robot.bin
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROBOT_INDEX, load_profile, robot_dir  # noqa: E402


def pack(profile: dict) -> bytes:
    robot_id = profile["robot_id"]
    n_joints = int(profile["n_joints"])
    obs_dim = int(profile.get("obs_dim") or 0)
    rate_hz = int(profile.get("rate_hz") or 50)
    servo_fn0 = int(profile.get("servo_fn0") or 94)
    q0 = np.array(profile.get("q0") or [0.0] * n_joints, dtype=np.float32)
    names = list(profile.get("joint_names") or [f"j{i}" for i in range(n_joints)])
    if q0.size != n_joints:
        raise SystemExit(f"q0 length {q0.size} != n_joints {n_joints}")
    if len(names) != n_joints:
        raise SystemExit(f"joint_names length {len(names)} != n_joints {n_joints}")

    rid = robot_id.encode("ascii")[:16]
    rid = rid + b"\0" * (16 - len(rid))
    buf = bytearray()
    buf += b"NNMR"
    buf += rid
    buf += struct.pack("<HHHH", n_joints, obs_dim, rate_hz, servo_fn0)
    buf += q0.tobytes()
    for name in names:
        b = name.encode("ascii")
        if len(b) > 255:
            raise SystemExit(f"joint name too long: {name}")
        buf += bytes([len(b)]) + b
    return bytes(buf)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", required=True, choices=sorted(ROBOT_INDEX))
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    profile = load_profile(args.robot)
    if int(profile.get("n_joints") or 0) <= 0:
        raise SystemExit(f"{args.robot}: n_joints not set in profile yet")
    out = args.out or (robot_dir(args.robot) / "robot" / "robot.bin")
    out.parent.mkdir(parents=True, exist_ok=True)
    blob = pack(profile)
    out.write_bytes(blob)
    print(f"wrote {out} ({len(blob)} bytes) robot_id={args.robot}")


if __name__ == "__main__":
    main()
