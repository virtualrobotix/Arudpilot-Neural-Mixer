#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Training evolution video: the same command sequence played by successive checkpoints.

Reads a train_velocity.py run directory (checkpoints <name>_itNNNN.nnm and
eval_checkpoints.jsonl), picks iteration 0, evenly spaced checkpoints and the
best ones, and renders one clip per checkpoint in MuJoCo (the int8 network as
it runs on the autopilot). Clips are concatenated into a single mp4.

    python tools/robots/evolution_video.py robots/microban/policies/walk_gait_run --robot microban
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nnm_env import load_ppo_config  # noqa: E402
from play_policy import _run_with_video  # noqa: E402

PHASES = [("avanti 4 s", "time", (0.3, 0.0, 0.0), 4.0),
          ("sinistra 3 s", "time", (0.0, 0.0, 0.8), 3.0)]


def pick(run_dir: Path, n_steps: int) -> list[tuple[int, Path, float | None]]:
    ck = {}
    for p in run_dir.glob("*_it[0-9][0-9][0-9][0-9].nnm"):
        ck[int(re.search(r"_it(\d{4})\.nnm$", p.name).group(1))] = p
    scores = {}
    ev = run_dir / "eval_checkpoints.jsonl"
    if ev.is_file():
        for line in ev.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                scores[r["iteration"]] = r["score"]
    its = sorted(ck)
    if not its:
        raise SystemExit(f"no checkpoints in {run_dir}")
    chosen = {its[0], its[-1]}
    step = max(1, (len(its) - 1) // max(1, n_steps - 1))
    chosen.update(its[::step])
    best = sorted((s, i) for i, s in scores.items() if i in ck)[-3:]
    chosen.update(i for _, i in best)
    return [(i, ck[i], scores.get(i)) for i in sorted(chosen)]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--clips", type=int, default=8, help="evenly spaced checkpoints (plus the 3 best)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--epochs-per-iter", type=int, default=5)
    ap.add_argument("--cam-dist", type=float, default=1.2, help="camera distance (m)")
    ap.add_argument("--clip-s", type=float, default=3.0, help="seconds per eval mode with env.eval_modes")
    args = ap.parse_args()
    phases = PHASES
    modes = load_ppo_config(args.robot).get("env", {}).get("eval_modes")
    if modes:
        phases = [(name, "time", tuple(cmd), args.clip_s) for name, cmd in modes.items() if any(cmd)]

    import mujoco

    selection = pick(args.run_dir, args.clips)
    out = args.out or args.run_dir / "evolution.mp4"
    best_its = {i for i, _, s in sorted(selection, key=lambda x: -(x[2] or -1))[:3] if s is not None}
    proc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                             "-s", "960x540", "-r", "25", "-i", "-", "-pix_fmt", "yuv420p", "-crf", "23",
                             str(out)], stdin=subprocess.PIPE)

    class A:
        robot = args.robot
        vx = 0.3
        wz = 0.8

    for it, path, score in selection:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance, cam.elevation, cam.azimuth = args.cam_dist, -20, 135
        tag = "  MIGLIORE" if it in best_its else ""
        sc = f"  punteggio {score:.2f}" if score is not None else ""
        title = f"{args.robot} — iterazione {it} ({it * args.epochs_per_iter} epoche){sc}{tag}"
        res = _run_with_video(A, path, cam, proc, phases=phases, title=title, hold_after_fall_s=1.0)
        print(f"iteration {it:4d}: {'CADUTO' if res['fell'] else 'ok'} t={res['t_end']:.1f}s "
              f"percorso {res['path_m']:.2f} m  imbardata {res['yaw_total_deg']:+.0f}°{sc}{tag}")
    proc.stdin.close()
    proc.wait()
    print(f"video: {out}")


if __name__ == "__main__":
    main()
