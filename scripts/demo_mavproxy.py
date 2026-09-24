#!/usr/bin/env python3
"""End-to-end visual demo: MuJoCo viewer + ArduRover SITL (AP_MicroDuck) + MAVProxy driving it.

Starts the three processes and then types a command sequence into MAVProxy, exactly what an
operator would do at the console (arm, sticks with `rc`, `mode hold`, `disarm`), while the duck
moves in the MuJoCo window. MAVProxy's own output stays visible in this terminal.

    scripts/demo_mavproxy.py                 # MLP
    scripts/demo_mavproxy.py --policy 1      # Cartan
    scripts/demo_mavproxy.py --keep          # leave everything running after the sequence (Ctrl-C to quit)
    scripts/demo_mavproxy.py --graph         # MAVProxy live graph of PPO_* telemetry (needs wxPython)
    scripts/demo_mavproxy.py --console       # MAVProxy console window (needs wxPython)

Sequence: boot -> arm (stand) 8 s -> forward rc2 2000 12 s -> lateral rc1 1800 8 s
-> turn rc4 2000 8 s -> mode HOLD 5 s -> manual -> disarm.
On macOS the plant runs under `.venv/bin/mjpython` (required by the MuJoCo viewer).
"""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PY = ROOT / ".venv" / "bin" / "python"
ARDUROVER = ROOT / "ardupilot" / "build" / "sitl" / "bin" / "ardurover"
RUN_DIR = ROOT / "sitl" / "run"
PARM = ROOT / "sitl" / "microduck.parm"


def log(msg: str) -> None:
    print(f"\033[1;36m[demo]\033[0m {msg}", flush=True)


def ensure_libpython_for_mjpython() -> None:
    """uv-managed CPython has no .venv/lib/libpython*.dylib; mjpython dlopens it via @executable_path/../lib."""
    import sysconfig
    ver = f"{sys.version_info.major}.{sys.version_info.minor}"
    target = ROOT / ".venv" / "lib" / f"libpython{ver}.dylib"
    if target.exists():
        return
    src = Path(sysconfig.get_config_var("LIBDIR") or "") / f"libpython{ver}.dylib"
    if src.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        target.symlink_to(src)
        log(f"linked {target.name} for mjpython")


class Demo:
    def __init__(self, args):
        self.args = args
        self.procs: list[subprocess.Popen] = []
        self.mav: subprocess.Popen | None = None

    def start_plant(self):
        # macOS: mujoco.viewer.launch_passive needs the mjpython launcher shipped with the mujoco wheel
        py = PY
        if not self.args.no_viewer and sys.platform == "darwin" and (PY.parent / "mjpython").exists():
            py = PY.parent / "mjpython"
            ensure_libpython_for_mjpython()
        cmd = [str(py), str(ROOT / "plant" / "mujoco_json_plant.py"), "--debug-frames", "0"]
        if self.args.no_viewer:
            cmd.append("--no-viewer")
        if self.args.no_bam:
            cmd.append("--no-bam")
        if self.args.video:
            cmd += ["--video", str(self.args.video)]
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        p = subprocess.Popen(cmd, env=env)
        self.procs.append(p)
        log("MuJoCo plant started" + ("" if self.args.no_viewer else " (viewer window)"))

    def start_sitl(self):
        RUN_DIR.mkdir(parents=True, exist_ok=True)
        cmd = [str(ARDUROVER), "--model", "JSON:127.0.0.1", "--speedup", "1", "--slave", "0",
               "--defaults", str(PARM), "--sim-address=127.0.0.1", "-I0",
               "--home", "-35.363261,149.165230,584,353"]
        if self.args.wipe:
            cmd.insert(1, "-w")
        p = subprocess.Popen(cmd, cwd=RUN_DIR, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.procs.append(p)
        log("ArduRover SITL started (JSON backend, waiting for MAVProxy on tcp:5760)")

    def start_mavproxy(self):
        mavproxy = shutil.which("mavproxy.py") or "mavproxy.py"
        cmd = [mavproxy, "--master", "tcp:127.0.0.1:5760", "--out", "udp:127.0.0.1:14550",
               "--aircraft", "microduck"]
        if self.args.console:
            cmd.append("--console")  # needs wxPython in MAVProxy's python
        self.mav = subprocess.Popen(cmd, stdin=subprocess.PIPE, cwd=RUN_DIR, text=True, bufsize=1)
        self.procs.append(self.mav)
        log("MAVProxy started")

    def send(self, line: str, wait: float = 0.0):
        assert self.mav and self.mav.stdin
        log(f"MAVProxy> {line}")
        self.mav.stdin.write(line + "\n")
        self.mav.stdin.flush()
        if wait > 0:
            time.sleep(wait)

    def sequence(self):
        a = self.args
        # let SITL boot, params load, INS calibrate
        time.sleep(a.boot_wait)
        if a.graph:
            self.send("module load graph", 1.0)
            self.send("graph NAMED_VALUE_FLOAT[PPO_PGZ].value NAMED_VALUE_FLOAT[PPO_VX].value NAMED_VALUE_FLOAT[PPO_FAIL].value", 1.0)
        self.send(f"param set MDK_POLICY {a.policy}", 1.0)
        self.send("param set MDK_ENABLE 1", 1.0)
        self.send("mode manual", 1.0)
        self.send("rc all 1500", 1.0)
        self.send("arm throttle force", 1.0)
        log(f"stand {a.t_stand:.0f} s")
        time.sleep(a.t_stand)
        # full stick forward: on the CPU plant the MLP only really walks at 0.3-0.4 m/s of command
        self.send(f"rc 2 {a.rc_fwd}", 0.0); log(f"forward (rc2 {a.rc_fwd}) {a.t_move:.0f} s"); time.sleep(a.t_move)
        self.send("rc 2 1500", 3.0)
        self.send(f"rc 1 {a.rc_lat}", 0.0); log(f"lateral (rc1 {a.rc_lat}) {a.t_side:.0f} s"); time.sleep(a.t_side)
        self.send("rc 1 1500", 3.0)
        self.send(f"rc 4 {a.rc_turn}", 0.0); log(f"turn (rc4 {a.rc_turn}) {a.t_side:.0f} s"); time.sleep(a.t_side)
        self.send("rc 4 1500", 2.0)
        self.send("mode hold", 0.0); log("HOLD 5 s (twist forced to 0)"); time.sleep(5.0)
        self.send("mode manual", 2.0)
        self.send("disarm force", 2.0)
        log("sequence done")

    def stop(self):
        for p in reversed(self.procs):
            if p.poll() is None:
                try:
                    p.send_signal(signal.SIGINT)
                except Exception:  # noqa: BLE001
                    pass
        time.sleep(1.5)
        for p in self.procs:
            if p.poll() is None:
                p.kill()

    def run(self):
        try:
            self.start_plant()
            time.sleep(3.0)
            self.start_sitl()
            time.sleep(1.5)
            self.start_mavproxy()
            self.sequence()
            if self.args.keep:
                log("keeping plant + SITL + MAVProxy alive; Ctrl-C to quit")
                while all(p.poll() is None for p in self.procs):
                    time.sleep(1.0)
            else:
                time.sleep(2.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
            log("bye")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", type=int, default=0, help="0 MLP, 1 Cartan")
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--wipe", action="store_true", help="wipe SITL eeprom (-w) before start")
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--no-bam", action="store_true")
    ap.add_argument("--console", action="store_true", help="MAVProxy console window (needs wxPython)")
    ap.add_argument("--graph", action="store_true", help="MAVProxy live graph of PPO_* (needs wxPython)")
    ap.add_argument("--video", type=Path, default=None, help="also record an mp4 of the MuJoCo plant")
    ap.add_argument("--rc-fwd", type=int, default=2000, help="rc2 for the forward leg (2000 = MDK_VX_MAX)")
    ap.add_argument("--rc-lat", type=int, default=1800)
    ap.add_argument("--rc-turn", type=int, default=2000, help="rc4 for the turn leg (2000 = MDK_WZ_MAX)")
    ap.add_argument("--boot-wait", type=float, default=12.0)
    ap.add_argument("--t-stand", type=float, default=8.0)
    ap.add_argument("--t-move", type=float, default=12.0)
    ap.add_argument("--t-side", type=float, default=8.0)
    args = ap.parse_args()
    if not ARDUROVER.exists():
        sys.exit(f"build the SITL first: cd ardupilot && ../.venv/bin/python ./waf rover  ({ARDUROVER} missing)")
    Demo(args).run()


if __name__ == "__main__":
    main()
