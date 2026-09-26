#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Fetch the upstream robot repository and build the MuJoCo scene used for training.

    python tools/robots/fetch_upstream.py --robot microban
    python tools/robots/fetch_upstream.py --robot rex          # URDF -> robots/rex/robot/scene.xml

What it does, from the "upstream" block of robots/<id>/robot/profile.json:
- shallow clone of upstream.repo into third_party/<id> (git, no LFS)
- native MJCF (upstream.sim_model ends with .xml): used in place
- URDF only (upstream.urdf): MuJoCo compiles the URDF, a free joint, a floor,
  IMU sensors and position actuators are added, result written to
  robots/<id>/robot/scene.xml
- no upstream model (sim.mjcf): the hand-made MJCF versioned in
  robots/<id>/robot/ is used; the clone only provides the reference code
- optional published policy (upstream.policy_url): downloaded next to the
  robot's policies/ as <name>.onnx and converted to .nnm when the ONNX follows
  the NNMixer contract
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import THIRD_PARTY, load_profile, robot_dir, upstream  # noqa: E402


def git_clone(repo: str, dest: Path, branch: str | None) -> None:
    if (dest / ".git").exists():
        print(f"already cloned: {dest}")
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["git", "clone", "--depth", "1"]
    if branch:
        cmd += ["--branch", branch]
    cmd += [repo, str(dest)]
    print("+", " ".join(cmd))
    subprocess.check_call(cmd, env={"GIT_LFS_SKIP_SMUDGE": "1", **__import__("os").environ})


def urdf_to_scene(urdf: Path, profile: dict, out: Path) -> None:
    import mujoco

    sim = profile.get("sim", {})
    spec = mujoco.MjSpec.from_file(str(urdf))
    spec.compiler.discardvisual = False
    spec.option.timestep = 0.005
    world = spec.worldbody
    base_name = sim.get("trunk_body")
    base = spec.body(base_name) if base_name else None
    if base is None:
        base = next(b for b in world.bodies)
        print(f"trunk body not set; using first body {base.name!r}")
    # URDF bodies hang from world with a fixed joint; give the trunk a free joint
    if not any(j.type == mujoco.mjtJoint.mjJNT_FREE for j in base.joints):
        base.add_freejoint(name=sim.get("freejoint", "root"))
    base.pos = [0.0, 0.0, float(sim.get("home_z", 0.3))]
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1],
                   rgba=[0.8, 0.8, 0.8, 1.0])
    world.add_light(pos=[0, 0, 3], dir=[0, 0, -1])
    site = base.add_site(name="nnm_imu")
    spec.add_sensor(name=sim.get("gyro_sensor", "nnm_gyro"), type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site.name)
    spec.add_sensor(name=sim.get("accel_sensor", "nnm_accel"), type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site.name)
    kp = float(sim.get("position_kp", 10.0))
    kv = float(sim.get("position_kv", 0.3))
    frc = float(sim.get("force_limit", 3.0))
    for name in profile["joint_names"]:
        act = spec.add_actuator(name=f"{name}_pos", target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=kp, kv=kv)
        act.forcelimited = True
        act.forcerange = [-frc, frc]
    model = spec.compile()
    out.parent.mkdir(parents=True, exist_ok=True)
    # mesh paths in the written XML must resolve from robots/<id>/robot/
    spec.meshdir = str(urdf.parent.resolve())
    out.write_text(spec.to_xml())
    print(f"wrote {out}: nq={model.nq} nu={model.nu}")


def fetch_policy(url: str, robot_id: str, name: str) -> Path:
    dst = robot_dir(robot_id) / "policies" / f"{name}.onnx"
    dst.parent.mkdir(parents=True, exist_ok=True)
    print(f"download {url} -> {dst}")
    urllib.request.urlretrieve(url, dst)
    return dst


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--no-policy", action="store_true")
    args = ap.parse_args()
    profile = load_profile(args.robot)
    u = upstream(profile)
    if not u.get("repo"):
        raise SystemExit(f"{args.robot}: no upstream.repo in profile")
    for key in ("repo", "model_repo"):
        if u.get(key):
            dest = THIRD_PARTY / args.robot if key == "repo" else THIRD_PARTY / f"{args.robot}_model"
            git_clone(u[key], dest, u.get("branch") if key == "repo" else u.get("model_branch"))
    note = u.get("sim_model_note")
    sim_model = u.get("sim_model")
    root = THIRD_PARTY / (f"{args.robot}_model" if u.get("model_repo") else args.robot)
    own = profile.get("sim", {}).get("mjcf")
    if own:
        p = robot_dir(args.robot) / "robot" / own
        print(f"MJCF in this repo: {p} ({'found' if p.is_file() else 'MISSING'})")
    elif sim_model and sim_model.endswith(".xml"):
        p = root / sim_model
        print(f"native MJCF: {p} ({'found' if p.is_file() else 'MISSING'})")
    elif u.get("urdf"):
        urdf_to_scene(root / u["urdf"], profile, robot_dir(args.robot) / "robot" / "scene.xml")
    else:
        print(f"no MuJoCo model can be built automatically for {args.robot}.")
    if note:
        print(f"note: {note}")
    if u.get("policy_url") and not args.no_policy:
        onnx_path = fetch_policy(u["policy_url"], args.robot, u.get("policy_name", "upstream"))
        cmd = [sys.executable, str(Path(__file__).parent / "export_nnm.py"), str(onnx_path),
               "--robot", args.robot, "--parity"]
        print("+", " ".join(cmd))
        subprocess.call(cmd)


if __name__ == "__main__":
    main()
