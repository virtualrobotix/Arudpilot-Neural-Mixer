#!/usr/bin/env python3
"""Reference rollout: ONNX policy driving the MuJoCo plant directly (no ArduPilot).

Validates plant + policy + observation contract before the firmware is in the loop, and
produces the reference trajectory the AP_MicroDuck obs-parity test compares against.

    python tools/policy_rollout_plant.py policies/microduck_mlp_2048x2000_it1999.onnx --vx 0.2 --seconds 10

Observation (61, SI, trunk FLU frame — exactly scripts/infer_policy.py):
  gyro(3) proj_gravity(3) q-q0(14) qd(14) last_action(14) twist(3) head(4)=0 body(6)=0
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "plant"))
from mujoco_json_plant import DEFAULT_MJCF, DEFAULT_POSE, TIMESTEP, Plant  # noqa: E402

DECIMATION = 4  # 50 Hz policy on a 200 Hz plant


def quat_rotate_inverse(q, v):
    w, x, y, z = q
    qv = np.array([x, y, z])
    a = v * (2.0 * w * w - 1.0)
    b = np.cross(qv, v) * w * 2.0
    c = qv * np.dot(qv, v) * 2.0
    return a - b + c


def build_obs(plant: Plant, last_action: np.ndarray, twist: np.ndarray) -> np.ndarray:
    d = plant.data
    gyro = d.sensordata[plant.gyro_adr : plant.gyro_adr + 3]
    q = d.xquat[plant.trunk_id]
    proj_g = quat_rotate_inverse(q, np.array([0.0, 0.0, -1.0]))
    jpos = d.qpos[plant.qpos_idx] - DEFAULT_POSE
    jvel = d.qvel[plant.qvel_idx]
    obs = np.concatenate([gyro, proj_g, jpos, jvel, last_action, twist, np.zeros(4), np.zeros(6)])
    return obs.astype(np.float32)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("onnx_path")
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--wz", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--no-bam", action="store_true")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--realtime", action="store_true")
    ap.add_argument("--save", type=Path, default=None, help="save obs/actions trajectory (.npz)")
    args = ap.parse_args()

    plant = Plant(DEFAULT_MJCF, use_bam=not args.no_bam, viewer=args.viewer, auto_reset_s=0)
    sess = ort.InferenceSession(args.onnx_path, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name
    twist = np.array([args.vx, args.vy, args.wz], dtype=np.float32)
    last_action = np.zeros(14, dtype=np.float32)
    n_ticks = int(args.seconds / (TIMESTEP * DECIMATION))
    obs_log, act_log, z_log, xy0 = [], [], [], plant.data.qpos[plant.free_qpos : plant.free_qpos + 2].copy()
    fell_at = None
    t_wall = time.perf_counter()
    for k in range(n_ticks):
        obs = build_obs(plant, last_action, twist)
        act = sess.run(None, {in_name: obs[None]})[0][0].astype(np.float32)
        q_target = DEFAULT_POSE + act
        for _ in range(DECIMATION):
            plant.step(q_target)
        last_action = act
        obs_log.append(obs)
        act_log.append(act)
        z_log.append(plant.data.qpos[plant.free_qpos + 2])
        if fell_at is None and plant.tilt_deg() > 60:
            fell_at = plant.t
        if args.viewer and k % 2 == 0:
            plant.sync_viewer()
        if args.realtime:
            target = t_wall + (k + 1) * TIMESTEP * DECIMATION
            dt = target - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
    xy1 = plant.data.qpos[plant.free_qpos : plant.free_qpos + 2]
    disp = xy1 - xy0
    # forward direction in world = trunk x axis at start (yaw 0) -> world x
    print(f"policy={Path(args.onnx_path).name} twist={twist.tolist()} sim {plant.t:.1f}s")
    print(f"  fell: {'no' if fell_at is None else f'yes at {fell_at:.2f}s'}   final tilt {plant.tilt_deg():.1f} deg   z mean {np.mean(z_log):.3f}")
    print(f"  displacement x={disp[0]:+.3f} m y={disp[1]:+.3f} m  -> mean vx={disp[0]/plant.t:+.3f} m/s (cmd {args.vx:+.2f})")
    print(f"  |action| max {np.abs(act_log).max():.2f}  wall {time.perf_counter()-t_wall:.1f}s")
    if args.save:
        np.savez(args.save, obs=np.array(obs_log), act=np.array(act_log), z=np.array(z_log), twist=twist)
        print(f"  saved {args.save}")


if __name__ == "__main__":
    main()
