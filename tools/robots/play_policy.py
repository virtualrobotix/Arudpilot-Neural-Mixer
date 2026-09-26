#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Drive a .nnm policy through a command sequence in the ArduPilot-contract MuJoCo env.

The policy runs exactly as on the autopilot (int8 forward, firmware gravity
filter, PWM encoding). Default sequence: forward 5 s, turn right 3 s, turn
left until the trunk has yawed 180 degrees, forward 5 s.

    .venv/bin/mjpython tools/robots/play_policy.py --robot microban --nnm robots/microban/policies/walk.nnm
    .venv/bin/python   tools/robots/play_policy.py --robot microban --nnm a.nnm b.nnm --headless
    .venv/bin/python   tools/robots/play_policy.py --robot microban --nnm a.nnm --video out.mp4
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
from export_nnm import unpack_nnm  # noqa: E402
from nnm_env import NNMixerEnv, load_ppo_config  # noqa: E402

SIGNATURE = "Realizzati da Roberto Navoni — DelphyAI LAB"


def yaw_of(q) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Sequence:
    """Phases: ('forward', seconds) | ('turn_right', seconds) | ('turn_left_deg', degrees)."""

    def __init__(self, vx: float, wz: float, phases=None):
        self.phases = phases or [("avanti 5 s", "time", (vx, 0.0, 0.0), 5.0),
                                 ("destra 3 s", "time", (0.0, 0.0, -wz), 3.0),
                                 ("sinistra 180°", "yaw", (0.0, 0.0, wz), 180.0),
                                 ("avanti 5 s", "time", (vx, 0.0, 0.0), 5.0)]
        self.i = 0
        self.t0 = 0.0
        self.yaw_acc = 0.0
        self.yaw_timeout_s = 10.0     # a policy that cannot turn gives up after 10 s

    def command(self, t: float, dyaw: float):
        """Return (label, twist) for time t; dyaw = yaw change since the last call (rad)."""
        while self.i < len(self.phases):
            label, kind, twist, amount = self.phases[self.i]
            if kind == "time" and t - self.t0 < amount:
                return label, np.array(twist, np.float32)
            if kind == "yaw":
                self.yaw_acc += dyaw
                if math.degrees(self.yaw_acc) < amount and t - self.t0 < self.yaw_timeout_s:
                    return label, np.array(twist, np.float32)
            self.i += 1
            self.t0 = t
            self.yaw_acc = 0.0
            dyaw = 0.0
        return None, None


def run(robot: str, nnm: Path, vx: float, wz: float, viewer=None, renderer=None, video=None,
        realtime: bool = False, phases=None, title: str = "", hold_after_fall_s: float = 0.0) -> dict:
    cfg = load_ppo_config(robot)
    pk = unpack_nnm(nnm.read_bytes())
    env = NNMixerEnv(robot, cfg, seed=0)
    env.max_steps = 10 ** 9
    obs = env.reset()
    seq = Sequence(vx, wz, phases)
    fall_steps = 0
    rate = env.contract.rate_hz
    d = env.data
    start_xy = d.qpos[env.free_qpos:env.free_qpos + 2].copy()
    yaw_prev = yaw_of(d.xquat[env.trunk_id])
    yaw_total = 0.0
    path_len = 0.0
    xy_prev = start_xy.copy()
    fell = False
    log = []
    k = 0
    t_wall = time.perf_counter()
    while True:
        t = k / rate
        yaw = yaw_of(d.xquat[env.trunk_id])
        dyaw = math.atan2(math.sin(yaw - yaw_prev), math.cos(yaw - yaw_prev))
        yaw_prev = yaw
        yaw_total += dyaw
        label, twist = seq.command(t, dyaw)
        if label is None:
            break
        env.command = twist
        obs[env.contract.twist_offset:env.contract.twist_offset + 3] = twist
        obs, _, fell, _, info = env.step(dc.int8_forward(pk["layers"], pk["mean"], pk["std"], obs))
        xy = d.qpos[env.free_qpos:env.free_qpos + 2].copy()
        path_len += float(np.linalg.norm(xy - xy_prev))
        xy_prev = xy
        if not log or log[-1][0] != label:
            log.append([label, t, 0.0, 0.0])
        log[-1][2] = t + 1 / rate
        log[-1][3] += dyaw
        if viewer is not None:
            if not viewer.is_running():
                break
            with viewer.lock():
                viewer.cam.lookat[:] = d.qpos[env.free_qpos:env.free_qpos + 3]
            viewer.sync()
        if renderer is not None and k % max(1, rate // 25) == 0:
            renderer.update_scene(d, camera=video["cam"])
            video["cam"].lookat[:] = d.qpos[env.free_qpos:env.free_qpos + 3]
            frame = renderer.render()
            video["proc"].stdin.write(_overlay(frame, label if not fell else "CADUTO", t, info, twist,
                                               title).tobytes())
        if realtime:
            dt = t_wall + (k + 1) / rate - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
        k += 1
        if fell:
            # keep filming the fall for a moment, then stop
            fall_steps += 1
            if fall_steps > hold_after_fall_s * rate:
                break
    disp = xy_prev - start_xy
    return {"policy": nnm.name, "fell": fell, "t_end": k / rate, "path_m": path_len,
            "net_displacement_m": float(np.linalg.norm(disp)), "yaw_total_deg": math.degrees(yaw_total),
            "phases": [(lbl, round(a, 2), round(b, 2), round(math.degrees(y), 1)) for lbl, a, b, y in log]}


def _font(size: int):
    from PIL import ImageFont
    for f in ("/System/Library/Fonts/Menlo.ttc", "/System/Library/Fonts/Monaco.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"):
        if Path(f).exists():
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def _overlay(frame: np.ndarray, label: str, t: float, info: dict, twist, title: str = "") -> np.ndarray:
    from PIL import Image, ImageDraw

    img = Image.fromarray(frame)
    dr = ImageDraw.Draw(img, "RGBA")
    top = 58 if title else 34
    dr.rectangle([0, 0, img.width, top], fill=(10, 20, 35, 190))
    y = 8
    if title:
        dr.text((10, y), title, fill=(255, 200, 80, 255), font=_font(18))
        y += 26
    lateral = f" vy={twist[1]:+.2f}" if twist[1] else ""
    dr.text((10, y), f"t={t:5.1f}s  fase: {label}  cmd vx={twist[0]:+.2f}{lateral} m/s wz={twist[2]:+.2f} rad/s  "
                     f"vx reale={info['vx']:+.2f}", fill=(240, 240, 240, 255), font=_font(14))
    dr.text((10, img.height - 22), SIGNATURE, fill=(220, 220, 220, 255), font=_font(13))
    return np.asarray(img)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--nnm", type=Path, nargs="+", required=True)
    ap.add_argument("--vx", type=float, default=0.3)
    ap.add_argument("--wz", type=float, default=0.8)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--video", type=Path, default=None)
    args = ap.parse_args()

    for nnm in args.nnm:
        if args.video:
            import mujoco
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.distance, cam.elevation, cam.azimuth = 1.2, -20, 135
            proc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                     "-s", "960x540", "-r", "25", "-i", "-", "-pix_fmt", "yuv420p",
                                     str(args.video)], stdin=subprocess.PIPE)
            # the renderer needs the env that is stepped: run() builds its own env, so render from that one
            res = _run_with_video(args, nnm, cam, proc)
            proc.stdin.close(); proc.wait()
            print(f"video: {args.video}")
        elif args.headless:
            res = run(args.robot, nnm, args.vx, args.wz)
        else:
            import mujoco.viewer
            res = _run_with_viewer(args, nnm)
        print(f"{res['policy']}: {'CADUTO' if res['fell'] else 'ok'} a t={res['t_end']:.1f}s  "
              f"percorso {res['path_m']:.2f} m  spostamento netto {res['net_displacement_m']:.2f} m  "
              f"imbardata totale {res['yaw_total_deg']:+.0f}°")
        for lbl, a, b, y in res["phases"]:
            print(f"    {lbl:14s} {a:5.1f}-{b:5.1f}s  imbardata {y:+6.1f}°")


def _run_with_viewer(args, nnm):
    import mujoco.viewer

    cfg = load_ppo_config(args.robot)
    holder = {}
    orig_reset = NNMixerEnv.reset

    def reset_and_launch(self):
        o = orig_reset(self)
        if "viewer" not in holder:
            holder["viewer"] = mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False,
                                                            show_right_ui=False)
            holder["viewer"].cam.distance, holder["viewer"].cam.elevation = 1.2, -20
        return o

    NNMixerEnv.reset = reset_and_launch
    try:
        res = run(args.robot, nnm, args.vx, args.wz, viewer=_LazyViewer(holder), realtime=True)
        time.sleep(2.0)
    finally:
        NNMixerEnv.reset = orig_reset
        if "viewer" in holder:
            holder["viewer"].close()
    _ = cfg
    return res


class _LazyViewer:
    def __init__(self, holder):
        self.h = holder

    def __getattr__(self, name):
        return getattr(self.h["viewer"], name)


def _run_with_video(args, nnm, cam, proc, phases=None, title="", hold_after_fall_s=0.0):
    import mujoco

    holder = {}
    orig_reset = NNMixerEnv.reset

    def reset_and_render(self):
        o = orig_reset(self)
        if "r" not in holder:
            self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, 960)
            self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, 540)
            holder["r"] = mujoco.Renderer(self.model, 540, 960)
        return o

    NNMixerEnv.reset = reset_and_render
    try:
        res = run(args.robot, nnm, args.vx, args.wz, renderer=_LazyRenderer(holder),
                  video={"cam": cam, "proc": proc}, phases=phases, title=title,
                  hold_after_fall_s=hold_after_fall_s)
    finally:
        NNMixerEnv.reset = orig_reset
    return res


class _LazyRenderer:
    def __init__(self, holder):
        self.h = holder

    def __getattr__(self, name):
        return getattr(self.h["r"], name)


if __name__ == "__main__":
    main()
