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

    def yaw_deg(self) -> float:
        w, x, y, z = self.data.xquat[self.trunk_id]
        return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

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


class Overlay:
    """Draws a GCS-side panel on the video: the MAVProxy commands / MAVLink messages the driver
    script is sending (read from a JSON-lines file appended by scripts/demo_mavproxy.py) plus the
    plant state. Rendered with Pillow onto the RGB frame."""

    FONTS = ["/System/Library/Fonts/Menlo.ttc", "/System/Library/Fonts/Monaco.ttf",
             "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"]

    def __init__(self, path: Path, width: int, height: int):
        from PIL import ImageFont

        self.path = path
        self.events: list[dict] = []
        self.mtime = -1.0
        self.width, self.height = width, height
        self.font = None
        self.font_b = None
        for f in self.FONTS:
            if Path(f).exists():
                self.font = ImageFont.truetype(f, 15)
                self.font_b = ImageFont.truetype(f, 17)
                break
        if self.font is None:
            self.font = ImageFont.load_default()
            self.font_b = self.font

    def refresh(self):
        try:
            st = self.path.stat()
        except FileNotFoundError:
            return
        if st.st_mtime == self.mtime:
            return
        self.mtime = st.st_mtime
        ev = []
        for line in self.path.read_text().splitlines():
            line = line.strip()
            if line:
                try:
                    ev.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
        self.events = ev

    def draw(self, frame: np.ndarray, plant: "Plant", driving: bool) -> np.ndarray:
        from PIL import Image, ImageDraw

        self.refresh()
        img = Image.fromarray(frame)
        d = ImageDraw.Draw(img, "RGBA")
        W = self.width
        # header: the chain
        d.rectangle([0, 0, W, 30], fill=(10, 20, 35, 200))
        d.text((10, 6), "MAVProxy script --MAVLink tcp:5760--> ArduRover SITL [AP_MicroDuck PPO 50 Hz] --SIM_JSON udp--> MuJoCo",
               font=self.font_b, fill=(230, 235, 245, 255))
        # credit box (top-left)
        x0, pw = 10, 470
        d.rectangle([x0, 40, x0 + pw, 92], fill=(10, 20, 35, 200), outline=(255, 200, 80, 255))
        d.text((x0 + 10, 46), "Preview - PPO development by Roberto Navoni", font=self.font_b, fill=(255, 200, 80, 255))
        d.text((x0 + 10, 68), "ArduPilot Dev Team  ·  r.navoni74@gmail.com", font=self.font, fill=(230, 235, 245, 255))
        # command panel (below the credit box)
        y0 = 100
        recent = self.events[-6:]
        ph = 30 + 40 * max(len(recent), 1) + 8
        d.rectangle([x0, y0, x0 + pw, y0 + ph], fill=(10, 20, 35, 185), outline=(120, 160, 220, 255))
        d.text((x0 + 10, y0 + 6), "Commands sent by the script -> MAVLink", font=self.font_b, fill=(255, 200, 80, 255))
        y = y0 + 32
        for i, e in enumerate(recent):
            last = i == len(recent) - 1
            col = (255, 255, 255, 255) if last else (170, 185, 205, 255)
            d.text((x0 + 10, y), f"MAV> {e.get('cmd', '')}", font=self.font, fill=col)
            d.text((x0 + 10, y + 18), f"     {e.get('mavlink', '')}", font=self.font,
                   fill=(120, 220, 160, 255) if last else (110, 150, 130, 255))
            y += 40
        # phase badge (top-right)
        phase = recent[-1].get("phase", "") if recent else ""
        if phase:
            # badge in the free area right of the credit box; shrink/trim so it never overlaps it
            max_w = W - (x0 + pw) - 30
            font = self.font_b
            if d.textlength(phase, font=font) + 24 > max_w:
                font = self.font
            while d.textlength(phase, font=font) + 24 > max_w and len(phase) > 8:
                phase = phase[:-2] + "…"
            tw = d.textlength(phase, font=font) + 24
            d.rectangle([W - tw - 10, 40, W - 10, 70], fill=(200, 60, 40, 210))
            d.text((W - tw + 2, 47), phase, font=font, fill=(255, 255, 255, 255))
        # plant status (bottom-left)
        x, yv, z = plant.data.qpos[plant.free_qpos : plant.free_qpos + 3]
        status = (f"MuJoCo  t={plant.t:6.2f}s  xy=({x:+.2f},{yv:+.2f}) m  z={z:.3f} m  tilt={plant.tilt_deg():4.1f} deg  "
                  f"yaw={plant.yaw_deg():+6.1f} deg  {'servos active (armed)' if driving else 'idle (disarmed)'}")
        d.rectangle([0, self.height - 28, W, self.height], fill=(10, 20, 35, 200))
        d.text((10, self.height - 23), status, font=self.font, fill=(230, 235, 245, 255))
        return np.asarray(img)


class VideoRecorder:
    """Offscreen render of the plant to an mp4 (ffmpeg pipe), camera tracking the trunk."""

    def __init__(self, plant: Plant, path: Path, fps: float = 25.0, width: int = 960, height: int = 540,
                 overlay: Path | None = None):
        import shutil
        import subprocess

        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg not found")
        self.plant = plant
        self.fps = fps
        self.overlay = Overlay(overlay, width, height) if overlay is not None else None
        self.driving = False
        # the scene's offscreen framebuffer defaults to 640x480; enlarge it for the recording
        plant.model.vis.global_.offwidth = max(plant.model.vis.global_.offwidth, width)
        plant.model.vis.global_.offheight = max(plant.model.vis.global_.offheight, height)
        self.renderer = mujoco.Renderer(plant.model, height=height, width=width)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.distance = 0.9
        self.cam.azimuth = 135.0
        self.cam.elevation = -18.0
        self.proc = subprocess.Popen(
            [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
             "-r", str(fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(path)],
            stdin=subprocess.PIPE,
        )
        self.next_t = 0.0
        self.n = 0
        self.path = path

    def maybe_capture(self, label: str = ""):
        if self.plant.t < self.next_t:
            return
        self.next_t = self.plant.t + 1.0 / self.fps
        d = self.plant.data
        self.cam.lookat[:] = d.qpos[self.plant.free_qpos : self.plant.free_qpos + 3]
        if self.overlay is not None:
            # keep the robot in the right part of the frame; the command panel lives on the left
            az = math.radians(self.cam.azimuth)
            self.cam.lookat[0] += 0.25 * -math.sin(az)
            self.cam.lookat[1] += 0.25 * math.cos(az)
        self.cam.lookat[2] = 0.08
        self.renderer.update_scene(d, camera=self.cam)
        frame = self.renderer.render()
        if self.overlay is not None:
            frame = self.overlay.draw(frame, self.plant, self.driving)
        if self.proc.stdin is not None:
            self.proc.stdin.write(frame.tobytes())
        self.n += 1

    def close(self):
        if self.proc.stdin is not None:
            self.proc.stdin.close()
        self.proc.wait(timeout=30)
        print(f"[plant] video: {self.n} frames -> {self.path}")


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
    ap.add_argument("--video", type=Path, default=None, help="record an mp4 of the plant (offscreen render, 25 fps)")
    ap.add_argument("--overlay", type=Path, default=None,
                    help="JSON-lines file with the GCS commands to show on the video (written by scripts/demo_mavproxy.py)")
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
    video = None
    if args.video is not None:
        try:
            video = VideoRecorder(plant, args.video, overlay=args.overlay)
            print(f"[plant] recording video to {args.video}" + (f" (overlay {args.overlay})" if args.overlay else ""))
        except Exception as e:  # noqa: BLE001
            print(f"[plant] video disabled ({e!r})")
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
            if video is not None and (driving or video.overlay is not None):
                # with an overlay we also film the idle phase (boot, arm) so the command panel tells the story
                video.driving = driving
                video.maybe_capture()
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
        if video is not None:
            video.close()
        if args.record is not None and rec:
            arr = np.array(rec)
            np.savez(args.record, t=arr[:, 0], frame=arr[:, 1], gyro_flu=arr[:, 2:5], grav_flu=arr[:, 5:8],
                     jpos=arr[:, 8:22], jvel=arr[:, 22:36], pwm=arr[:, 36:50], trunk=arr[:, 50:57])
            print(f"[plant] recorded {len(rec)} frames -> {args.record}")
        print("[plant] bye")


if __name__ == "__main__":
    main()
