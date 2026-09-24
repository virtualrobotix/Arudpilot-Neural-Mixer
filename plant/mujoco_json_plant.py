#!/usr/bin/env python3
"""MicroDuck MuJoCo plant for ArduPilot SITL (JSON physics backend).

ArduPilot side:   sim_vehicle.py -v Rover --model JSON:127.0.0.1  (SIM_RATE_HZ 200)
This side:        python plant/mujoco_json_plant.py [--no-viewer] [--no-bam]

Protocol (libraries/SITL/examples/JSON/readme.md):
  SITL -> plant  UDP :9002  binary  {u16 magic=18458, u16 frame_rate, u32 frame_count, u16 pwm[16]}
  plant -> SITL  UDP reply  one JSON line: timestamp, imu{gyro, accel_body}, position, velocity,
                 quaternion  (+ our extension: joints{jpos[14], jvel[14]})

Frames: ArduPilot is body FRD / world NED. MicroDuck trunk is FLU / world z-up.
Mapping (both frames):  (x, y, z) -> (x, -y, -z);  quaternion (w,x,y,z) -> (w, x, -y, -z).

Servo encoding (must match AP_MicroDuck): q_target_rad = (pwm - 1500) * 0.003   (1 us = 3 mrad).

Physics step 0.005 s per servo packet (=> SITL runs at 200 Hz, the 50 Hz policy task sees 4 substeps,
same decimation as training). Actuators: BAM M6 XL330 voltage model when `bam` is importable
(what the policies were trained with), else the XML position actuators.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import struct
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

try:
    import mujoco.viewer  # noqa: F401  (optional: needs a display)
except Exception:  # noqa: BLE001
    pass

NOESIS_MJCF = Path(
    "/Users/robertonavoni/Desktop/Lavoro/Progetti-2026/Progetti Software/NOESIS EXPERIMENT/"
    "third_party/microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml"
)
DEFAULT_MJCF = Path(os.environ.get("MICRODUCK_MJCF", str(NOESIS_MJCF)))

SERVO_MAGIC_16 = 18458
SERVO_MAGIC_32 = 29569
PWM_CENTER = 1500
RAD_PER_US = 0.003
TIMESTEP = 0.005
HOME_TRUNK_Z = 0.125

JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
DEFAULT_POSE = np.array([
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
], dtype=np.float64)

# BAM M6 defaults — mirror scripts/infer_policy.py in microduck_rl
BAM_KP_FW = 200.0
BAM_VIN = 7.4
BAM_VIN_MIN = 6.0
BAM_STIFF_SOLREF_FRICTION = (-5.0e4, -2.0e2)
BAM_STIFF_SOLIMP_FRICTION = (0.99, 0.9999, 0.001, 0.5, 2.0)


def load_with_bam(xml_path: Path, vin: float, kp_fw: float):
    from bam.model import load_model
    from bam.mujoco import MujocoController

    bam_model = load_model(motor_name="xl330", model="m6")
    bam_model.actuator.kp = kp_fw
    bam_model.actuator.vin = vin
    bam_model.actuator.max_current = None
    kt, R = bam_model.kt.value, bam_model.R.value
    force_limit = vin * kt / R

    spec = mujoco.MjSpec.from_file(str(xml_path))
    names = []
    for act in spec.actuators:
        tgt = act.target
        tgt_name = tgt.name if hasattr(tgt, "name") else str(tgt)
        if tgt_name.startswith("passive_"):
            continue
        act.set_to_motor()
        act.forcelimited = True
        act.forcerange = (-force_limit, force_limit)
        act.ctrllimited = False
        act.gear = [1.0, 0, 0, 0, 0, 0]
        names.append(act.name)
        for joint in spec.joints:
            if joint.name == tgt_name:
                joint.damping = np.zeros((3, 1))
                joint.frictionloss = 0.0
                joint.solref_friction = BAM_STIFF_SOLREF_FRICTION
                joint.solimp_friction = BAM_STIFF_SOLIMP_FRICTION
                break
    model = spec.compile()
    model.opt.timestep = TIMESTEP
    data = mujoco.MjData(model)
    # bam.mujoco.MujocoController (Rhoban/bam mjlab_frictionloss): targets live in ctrl.q_target,
    # update() writes the voltage-model torque into data.ctrl and refreshes friction/damping.
    ctrl = MujocoController(bam_model, names, model, data, vin_drop_resistance=None, vin_min=BAM_VIN_MIN)
    return model, data, ctrl


class Plant:
    def __init__(self, xml_path: Path, use_bam: bool, viewer: bool, auto_reset_s: float = 3.0):
        self.bam = None
        self.auto_reset_s = auto_reset_s
        self.fallen_since = None
        self.t = 0.0
        if use_bam:
            try:
                self.model, self.data, self.bam = load_with_bam(xml_path, BAM_VIN, BAM_KP_FW)
                print(f"[plant] BAM M6 XL330 actuators (vin={BAM_VIN} V, kp_fw={BAM_KP_FW})")
            except Exception as e:  # noqa: BLE001
                print(f"[plant] BAM unavailable ({e!r}); falling back to XML position actuators")
        if self.bam is None:
            self.model = mujoco.MjModel.from_xml_path(str(xml_path))
            self.model.opt.timestep = TIMESTEP
            self.data = mujoco.MjData(self.model)
            print("[plant] XML position actuators (NOT the actuator the policy was trained with)")
        m = self.model
        assert m.nu == 14, f"expected 14 actuators, got {m.nu}"
        self.act_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
        self.joint_ids = [int(m.actuator_trnid[i, 0]) for i in range(m.nu)]
        jn = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in self.joint_ids]
        assert jn == JOINT_NAMES, f"joint order mismatch: {jn}"
        self.qpos_idx = [int(m.jnt_qposadr[j]) for j in self.joint_ids]
        self.qvel_idx = [int(m.jnt_dofadr[j]) for j in self.joint_ids]
        self.trunk_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "trunk_base")
        fj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
        self.free_qpos = int(m.jnt_qposadr[fj])
        self.free_qvel = int(m.jnt_dofadr[fj])
        self.gyro_adr = int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")])
        self.acc_adr = int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")])
        self.viewer = None
        if viewer:
            try:
                self.viewer = mujoco.viewer.launch_passive(m, self.data, show_left_ui=False, show_right_ui=False)
            except Exception as e:  # noqa: BLE001
                print(f"[plant] no viewer ({e!r})")
        self.reset()

    def reset(self):
        d, m = self.data, self.model
        mujoco.mj_resetData(m, d)
        d.qpos[self.free_qpos : self.free_qpos + 3] = [0.0, 0.0, HOME_TRUNK_Z]
        d.qpos[self.free_qpos + 3 : self.free_qpos + 7] = [1, 0, 0, 0]
        for k, qi in enumerate(self.qpos_idx):
            d.qpos[qi] = DEFAULT_POSE[k]
        d.qvel[:] = 0
        if self.bam is not None:
            self.bam.q_target[:] = DEFAULT_POSE
            self.bam.last_ts = d.time
            d.ctrl[:] = 0.0
        else:
            d.ctrl[:] = DEFAULT_POSE
        mujoco.mj_forward(m, d)
        self.fallen_since = None
        print("[plant] reset to HOME stand")

    def tilt_deg(self) -> float:
        w, x, y, z = self.data.xquat[self.trunk_id]
        return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

    def idle(self):
        """SITL not driving yet (all PWM zero): keep the duck pinned upright, advance time only."""
        self.t += TIMESTEP

    def step(self, q_target: np.ndarray):
        if self.bam is not None:
            self.bam.q_target[:] = q_target
            self.bam.update()  # writes torque into data.ctrl
        else:
            self.data.ctrl[:] = q_target
        mujoco.mj_step(self.model, self.data)
        self.t += TIMESTEP
        # auto-reset after lying on the ground for a while, so test loops do not need a restart
        if self.tilt_deg() > 70.0:
            if self.fallen_since is None:
                self.fallen_since = self.t
            elif self.auto_reset_s > 0 and self.t - self.fallen_since > self.auto_reset_s:
                print(f"[plant] fallen for {self.auto_reset_s:.0f} s -> auto reset")
                self.reset()
        else:
            self.fallen_since = None

    def sensors(self) -> dict:
        d = self.data
        gyro = d.sensordata[self.gyro_adr : self.gyro_adr + 3]
        acc = d.sensordata[self.acc_adr : self.acc_adr + 3]
        q = d.xquat[self.trunk_id]  # w x y z, world(z-up) -> trunk(FLU)
        pos = d.qpos[self.free_qpos : self.free_qpos + 3]
        vel = d.qvel[self.free_qvel : self.free_qvel + 3]  # world linear velocity
        jpos = d.qpos[self.qpos_idx]
        jvel = d.qvel[self.qvel_idx]
        flip = np.array([1.0, -1.0, -1.0])
        return {
            "timestamp": round(self.t, 6),
            "imu": {
                "gyro": [float(x) for x in gyro * flip],
                "accel_body": [float(x) for x in acc * flip],
            },
            "position": [float(x) for x in pos * flip],
            "velocity": [float(x) for x in vel * flip],
            "quaternion": [float(q[0]), float(q[1]), float(-q[2]), float(-q[3])],
            "joints": {
                "jpos": [round(float(x), 6) for x in jpos],
                "jvel": [round(float(x), 5) for x in jvel],
            },
        }

    def sync_viewer(self):
        if self.viewer is not None:
            if not self.viewer.is_running():
                raise KeyboardInterrupt
            self.viewer.sync()


def pwm_to_rad(pwm: np.ndarray) -> np.ndarray:
    q = (pwm.astype(np.float64) - PWM_CENTER) * RAD_PER_US
    return np.clip(q, -2.2, 2.2)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    ap.add_argument("--port", type=int, default=9002)
    ap.add_argument("--no-bam", action="store_true")
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--hold-default", action="store_true",
                    help="ignore PWM and hold the default pose (bring-up test of the IMU path)")
    ap.add_argument("--stats-every", type=float, default=5.0, help="print rate stats every N s wall")
    ap.add_argument("--debug-frames", type=int, default=8, help="print the first N driving frames (PWM, targets)")
    ap.add_argument("--record", type=Path, default=None,
                    help="record per-frame truth (t, gyro FLU, gravity FLU, jpos, jvel, pwm) to this .npz on exit")
    args = ap.parse_args()

    if not args.mjcf.exists():
        sys.exit(f"MJCF not found: {args.mjcf} (set MICRODUCK_MJCF)")
    plant = Plant(args.mjcf, use_bam=not args.no_bam, viewer=not args.no_viewer)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(0.5)
    print(f"[plant] listening UDP :{args.port} — start SITL with --model JSON:127.0.0.1 (SIM_RATE_HZ 200)")

    fmt16 = struct.Struct("<HHI16H")
    fmt32 = struct.Struct("<HHI32H")
    last_count = None
    driving = False
    debug_left = 0
    frames = 0
    rec: list = []
    t_stat = time.perf_counter()
    last_view = 0.0
    view_period = 1.0 / 30.0
    try:
        while True:
            try:
                buf, addr = sock.recvfrom(1024)
            except socket.timeout:
                plant.sync_viewer()
                continue
            if len(buf) == fmt16.size:
                magic, frame_rate, frame_count, *pwm = fmt16.unpack(buf)
                if magic != SERVO_MAGIC_16:
                    continue
            elif len(buf) == fmt32.size:
                magic, frame_rate, frame_count, *pwm = fmt32.unpack(buf)
                if magic != SERVO_MAGIC_32:
                    continue
            else:
                continue
            if last_count is not None and frame_count < last_count:
                print(f"[plant] SITL restarted (frame_count {last_count} -> {frame_count}); resetting")
                plant.reset()
            if last_count is not None and frame_count == last_count:
                # SITL resend (no reply received in 1 s): answer with current state, no step
                pass
            else:
                pwm14 = np.array(pwm[:14], dtype=np.float64)
                if np.all(pwm14 == 0):
                    # SITL booted but AP_MicroDuck is not writing servos yet: keep the duck pinned
                    if driving:
                        print("[plant] servo outputs went to zero -> pinned (idle)")
                        driving = False
                    plant.idle()
                else:
                    if not driving:
                        print("[plant] servo outputs active -> physics running")
                        driving = True
                        debug_left = args.debug_frames
                    if debug_left > 0:
                        debug_left -= 1
                        qv = plant.data.qvel[plant.qvel_idx]
                        print(f"[plant] frame {frame_count} pwm={pwm14.astype(int).tolist()} q_tgt-q0={np.round(pwm_to_rad(pwm14) - DEFAULT_POSE, 3).tolist()} |qd|max={np.abs(qv).max():.2f} tilt={plant.tilt_deg():.1f}")
                    plant.step(DEFAULT_POSE if args.hold_default else pwm_to_rad(pwm14))
            last_count = frame_count
            frames += 1
            sens = plant.sensors()
            msg = json.dumps(sens, separators=(",", ":"))
            sock.sendto(("\n" + msg + "\n").encode(), addr)
            if args.record is not None and driving:
                d = plant.data
                q = d.xquat[plant.trunk_id]
                w, x, y, z = q
                # gravity in trunk FLU (training convention) from the true quaternion
                gfl = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]) * -1.0
                rec.append(np.concatenate([[plant.t, frame_count],
                                           d.sensordata[plant.gyro_adr:plant.gyro_adr + 3], gfl,
                                           d.qpos[plant.qpos_idx], d.qvel[plant.qvel_idx], pwm[:14],
                                           d.qpos[plant.free_qpos:plant.free_qpos + 7]]))

            now = time.perf_counter()
            if now - last_view >= view_period:
                plant.sync_viewer()
                last_view = now
            if now - t_stat >= args.stats_every:
                hz = frames / (now - t_stat)
                x, y, z = plant.data.qpos[plant.free_qpos : plant.free_qpos + 3]
                print(f"[plant] {hz:6.1f} frames/s  sim t={plant.t:7.2f}s  xy=({x:+.2f},{y:+.2f}) z={z:.3f} m  "
                      f"tilt={plant.tilt_deg():4.1f} deg  sitl_rate={frame_rate}  {'DRIVING' if driving else 'idle'}")
                frames = 0
                t_stat = now
    except KeyboardInterrupt:
        pass
    finally:
        if args.record is not None and rec:
            arr = np.array(rec)
            np.savez(args.record, t=arr[:, 0], frame=arr[:, 1], gyro_flu=arr[:, 2:5], grav_flu=arr[:, 5:8],
                     jpos=arr[:, 8:22], jvel=arr[:, 22:36], pwm=arr[:, 36:50], trunk=arr[:, 50:57])
            print(f"[plant] recorded {len(rec)} frames -> {args.record}")
        print("[plant] bye")


if __name__ == "__main__":
    main()
