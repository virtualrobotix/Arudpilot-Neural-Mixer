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

# Position = NNM_ROBOT value and NNM_RobotId in AP_NNMixer_Policy.h: append only, never reorder.
ALL_ROBOTS = ("microduck", "microban", "zeroth", "bimo", "legolas", "upkie", "rex", "yertle", "albert",
              "openduck")
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


THIRD_PARTY = REPO_ROOT / "third_party"


def upstream(profile: dict[str, Any]) -> dict[str, Any]:
    u = profile.get("upstream", {})
    return {"repo": u} if isinstance(u, str) else dict(u)


def resolve_mjcf(robot_id: str, profile: dict[str, Any] | None = None) -> Path | None:
    """MJCF used for simulation and training, first match wins:

    1. env NNMIXER_MJCF_<ID> (NNMIXER_MJCF also accepted for microduck)
    2. robots/<id>/robot/scene.xml (converted or hand-made scene)
    3. third_party/<id>/<upstream.sim_model> fetched by tools/robots/fetch_upstream.py
    """
    import os

    profile = profile or load_profile(robot_id)
    keys = [f"NNMIXER_MJCF_{robot_id.upper()}"]
    if robot_id == "microduck":
        keys += ["NNMIXER_MJCF", "MICRODUCK_MJCF"]
    for k in keys:
        env = os.environ.get(k)
        if env:
            p = Path(env)
            return p if p.is_file() else None
    local = robot_dir(robot_id) / "robot" / "scene.xml"
    if local.is_file():
        return local
    u = upstream(profile)
    rel = u.get("sim_model")
    if rel and str(rel).endswith(".xml"):
        root = THIRD_PARTY / (f"{robot_id}_model" if u.get("model_repo") else robot_id)
        p = root / rel
        if p.is_file():
            return p
    if robot_id == "microduck":
        noesis = Path("/Users/robertonavoni/Desktop/Lavoro/Progetti-2026/Progetti Software/NOESIS EXPERIMENT/"
                      "third_party/microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml")
        if noesis.is_file():
            return noesis
    return None


def expected_obs_dim(n_joints: int, head: int = 0, body: int = 0) -> int:
    # gyro(3) + gravity(3) + q(n) + qd(n) + a_prev(n) + twist(3) + optional head/body
    return 3 + 3 + 3 * n_joints + 3 + head + body
