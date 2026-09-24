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

Sequence: boot -> arm (stand) 6 s -> forward 3 s -> backward 2 s -> lateral 2 s -> turn 90 deg
(closed loop on the ATTITUDE yaw streamed back over MAVLink) -> forward 5 s -> HOLD -> manual -> disarm.
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


OVERLAY = RUN_DIR / "overlay.jsonl"


def log(msg: str) -> None:
    print(f"\033[1;36m[demo]\033[0m {msg}", flush=True)


def mavlink_for(cmd: str) -> str:
    """The MAVLink message MAVProxy emits for a console command (for the video panel)."""
    p = cmd.split()
    if not p:
        return ""
    if p[0] == "rc" and len(p) >= 3:
        ch = "all channels" if p[1] == "all" else f"chan{p[1]}"
        return f"RC_CHANNELS_OVERRIDE  {ch}={p[2]} us  (sysid 255 -> RC_Channels)"
    if p[0] == "arm":
        return "COMMAND_LONG  MAV_CMD_COMPONENT_ARM_DISARM  param1=1" + ("  param2=21196 (force)" if "force" in p else "")
    if p[0] == "disarm":
        return "COMMAND_LONG  MAV_CMD_COMPONENT_ARM_DISARM  param1=0" + ("  param2=21196 (force)" if "force" in p else "")
    if p[0] == "mode" and len(p) >= 2:
        modes = {"manual": 0, "acro": 1, "steering": 3, "hold": 4, "guided": 15}
        return f"COMMAND_LONG  MAV_CMD_DO_SET_MODE  custom_mode={modes.get(p[1].lower(), '?')} ({p[1].upper()})"
    if p[0] == "param" and len(p) >= 4 and p[1] == "set":
        return f"PARAM_SET  {p[2]} = {p[3]}"
    if p[0] == "module":
        return "(local MAVProxy module)"
    return "(MAVProxy local command)"


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
        self.phase = "boot"
        self.telem = None
        self._turn_rad = 0.0
        self._turn_last_ms = None
        if args.video is not None:
            RUN_DIR.mkdir(parents=True, exist_ok=True)
            OVERLAY.write_text("")

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
            cmd += ["--video", str(self.args.video), "--overlay", str(OVERLAY)]
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
               "--streamrate", "20", "--aircraft", "microduck"]
        if self.args.console:
            cmd.append("--console")  # needs wxPython in MAVProxy's python
        self.mav = subprocess.Popen(cmd, stdin=subprocess.PIPE, cwd=RUN_DIR, text=True, bufsize=1)
        self.procs.append(self.mav)
        log("MAVProxy started")

    def note(self, cmd: str, mavlink: str, phase: str):
        """Append an event for the video overlay panel (read by the plant's VideoRecorder)."""
        self.phase = phase or self.phase
        if self.args.video is None:
            return
        import json
        with open(OVERLAY, "a") as f:
            f.write(json.dumps({"t": time.time(), "cmd": cmd, "mavlink": mavlink, "phase": self.phase}) + "\n")

    def send(self, line: str, wait: float = 0.0, phase: str = ""):
        assert self.mav and self.mav.stdin
        log(f"MAVProxy> {line}")
        self.note(line, mavlink_for(line), phase)
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
        pol = "Cartan" if a.policy == 1 else "MLP"
        self.send(f"param set MDK_POLICY {a.policy}", 1.0, phase=f"setup: policy {pol}")
        self.send("param set MDK_ENABLE 1", 1.0)
        self.send("mode manual", 1.0)
        self.send("rc all 1500", 1.0, phase="sticks centred")
        self.send("arm throttle force", 1.0, phase=f"ARMED - standing ({pol})")
        log(f"stand {a.t_stand:.0f} s")
        time.sleep(a.t_stand)
        # full stick: on the CPU plant the MLP only really walks at 0.3-0.4 m/s of command
        vx = (a.rc_fwd - 1500) / 500 * 0.4
        self.send(f"rc 2 {a.rc_fwd}", 0.0, phase=f"forward  vx = {vx:+.2f} m/s  ({a.t_fwd:.0f} s)"); log(f"forward {a.t_fwd:.0f} s"); time.sleep(a.t_fwd)
        rc_back = 3000 - a.rc_fwd
        self.send(f"rc 2 {rc_back}", 0.0, phase=f"backward  vx = {-vx:+.2f} m/s  ({a.t_back:.0f} s)"); log(f"backward {a.t_back:.0f} s"); time.sleep(a.t_back)
        self.send("rc 2 1500", 1.5, phase="stop")
        vy = (a.rc_lat - 1500) / 500 * 0.3
        self.send(f"rc 1 {a.rc_lat}", 0.0, phase=f"lateral  vy = {vy:+.2f} m/s  ({a.t_lat:.0f} s)"); log(f"lateral {a.t_lat:.0f} s"); time.sleep(a.t_lat)
        self.send("rc 1 1500", 1.5, phase="stop")
        # turn 90 deg, closed loop on the yaw rate ArduPilot streams back (ATTITUDE.yawspeed, gyro based —
        # the EKF heading of a legless "rover" with a simulated compass is not trustworthy for this)
        wz = (a.rc_turn - 1500) / 500 * 1.0
        self.reset_turn_integrator()
        self.send(f"rc 4 {a.rc_turn}", 0.0, phase=f"turn {a.turn_deg:.0f} deg  wz = {wz:+.2f} rad/s  (integrating ATTITUDE.yawspeed)")
        log(f"turning until the integrated yaw rate reaches {a.turn_deg:.0f} deg")
        t0 = time.time()
        turned = 0.0
        while time.time() - t0 < a.t_turn_max:
            time.sleep(0.02)
            turned = abs(self.turned_deg())
            if turned >= a.turn_deg:
                break
        self.send("rc 4 1500", 0.0, phase=f"turned {turned:.0f} deg (from ATTITUDE.yawspeed) -> stop")
        log(f"turned {turned:.0f} deg in {time.time() - t0:.1f} s")
        time.sleep(1.5)
        self.send(f"rc 2 {a.rc_fwd}", 0.0, phase=f"forward  vx = {vx:+.2f} m/s  ({a.t_fwd2:.0f} s)"); log(f"forward {a.t_fwd2:.0f} s"); time.sleep(a.t_fwd2)
        self.send("rc 2 1500", 2.0, phase="stop")
        self.send("mode hold", 0.0, phase="HOLD: twist forced to 0"); log("HOLD 3 s"); time.sleep(3.0)
        self.send("mode manual", 1.5, phase="MANUAL")
        self.send("disarm force", 2.0, phase="DISARMED")
        log("sequence done")

    # -- telemetry listener on MAVProxy's --out udp:14550 (ATTITUDE yaw for the closed-loop turn)
    def start_telemetry(self):
        try:
            from pymavlink import mavutil
        except ImportError:
            self.telem = None
            return
        self.telem = mavutil.mavlink_connection("udpin:127.0.0.1:14550", source_system=254)

    def reset_turn_integrator(self):
        self._turn_rad = 0.0
        self._turn_last_ms = None
        if self.telem is not None:
            while self.telem.recv_match(type="ATTITUDE", blocking=False) is not None:
                pass

    def turned_deg(self) -> float:
        """Integrate ATTITUDE.yawspeed (rad/s, body z) over the message timestamps."""
        if self.telem is None:
            return 0.0
        import math
        while True:
            m = self.telem.recv_match(type="ATTITUDE", blocking=False)
            if m is None:
                break
            if self._turn_last_ms is not None:
                dt = (m.time_boot_ms - self._turn_last_ms) * 1e-3
                if 0.0 < dt < 1.0:
                    self._turn_rad += m.yawspeed * dt
            self._turn_last_ms = m.time_boot_ms
        return math.degrees(self._turn_rad)

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
            self.start_telemetry()
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
    ap.add_argument("--t-stand", type=float, default=6.0)
    ap.add_argument("--t-fwd", type=float, default=3.0, help="first forward leg (s)")
    ap.add_argument("--t-back", type=float, default=2.0, help="backward leg (s)")
    ap.add_argument("--t-lat", type=float, default=2.0, help="lateral leg (s)")
    ap.add_argument("--turn-deg", type=float, default=90.0, help="turn until ATTITUDE.yaw changed by this much")
    ap.add_argument("--t-turn-max", type=float, default=8.0, help="turn timeout (s)")
    ap.add_argument("--t-fwd2", type=float, default=5.0, help="forward leg after the turn (s)")
    args = ap.parse_args()
    if not ARDUROVER.exists():
        sys.exit(f"build the SITL first: cd ardupilot && ../.venv/bin/python ./waf rover  ({ARDUROVER} missing)")
    Demo(args).run()


if __name__ == "__main__":
    main()
