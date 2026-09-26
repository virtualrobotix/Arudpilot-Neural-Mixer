# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Shared helpers for the NNMixer multi-robot MuJoCo pipeline."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOTS_DIR = REPO_ROOT / "robots"

BIPEDS = ("microduck", "microban", "zeroth", "bimo", "legolas", "upkie")
QUADRUPEDS = ("rex", "yertle")
ALL_ROBOTS = BIPEDS + QUADRUPEDS

# Integer ids used by NNM_ROBOT (boot). Keep stable.
ROBOT_INDEX = {name: i for i, name in enumerate(ALL_ROBOTS)}


def robot_dir(robot_id: str) -> Path:
    if robot_id not in ROBOT_INDEX:
        raise SystemExit(f"unknown robot_id {robot_id!r}; known: {', '.join(ALL_ROBOTS)}")
    return ROBOTS_DIR / robot_id


def load_profile(robot_id: str) -> dict[str, Any]:
    path = robot_dir(robot_id) / "robot" / "profile.json"
    if not path.is_file():
        raise SystemExit(f"missing profile: {path}")
    data = json.loads(path.read_text())
    if data.get("robot_id") != robot_id:
        raise SystemExit(f"profile robot_id {data.get('robot_id')!r} != directory {robot_id!r}")
    return data


def resolve_mjcf(robot_id: str, profile: dict[str, Any] | None = None) -> Path | None:
    """Return a local MJCF path if present; MicroDuck may use NNMIXER_MJCF env."""
    import os

    if robot_id == "microduck":
        env = os.environ.get("NNMIXER_MJCF") or os.environ.get("MICRODUCK_MJCF")
        if env:
            p = Path(env)
            return p if p.is_file() else None
    profile = profile or load_profile(robot_id)
    local = robot_dir(robot_id) / "robot" / "scene.xml"
    if local.is_file():
        return local
    # optional path recorded in profile
    rel = profile.get("mjcf")
    if rel and not str(rel).startswith("third_party"):
        p = robot_dir(robot_id) / rel
        if p.is_file():
            return p
    return None


def expected_obs_dim(n_joints: int, head: int = 0, body: int = 0) -> int:
    # gyro(3) + gravity(3) + q(n) + qd(n) + a_prev(n) + twist(3) + optional head/body
    return 3 + 3 + 3 * n_joints + 3 + head + body
