#!/usr/bin/env python3
"""HIL test driver for NNMixer-on-ArduPilot (SITL + MuJoCo JSON plant).

Talks MAVLink to the SITL (default tcp:127.0.0.1:5760, i.e. sim_vehicle --no-mavproxy or the
bare ardurover binary) and drives the same battery a human would run from MAVProxy:
arm, sticks via RC override, HOLD mode, failsafe, disarm — while watching the PPO_* named
floats the task publishes and (optionally) the plant's UDP status.

    scripts/hil_test.py monitor                # print PPO_* telemetry
    scripts/hil_test.py battery                # full battery, exit code != 0 on failure
    scripts/hil_test.py battery --policy 1     # Cartan (when NNM_POLICY 1 is available)

Battery steps (each with pass criteria):
  stand     arm, sticks centred 15 s      -> PPO_FAIL == 0, PGZ < -0.9, no fall
  forward   RC2 1750 (vx ~ +0.2) 12 s     -> PPO_VX ~ 0.2, no fall
  lateral   RC1 1650 8 s                  -> no fall
  turn      RC4 1750 (wz ~ +0.5) 8 s      -> no fall
  hold      mode HOLD 5 s                 -> PPO_VX == 0
  disarm    disarm                        -> PPO_FAIL == 1
"""

from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil

ROVER_MODES = {"MANUAL": 0, "ACRO": 1, "STEERING": 3, "HOLD": 4, "GUIDED": 15}


class Link:
    def __init__(self, url: str):
        # source_system 255 = the GCS sysid ArduPilot accepts RC overrides from (MAV_GCS_SYSID)
        self.m = mavutil.mavlink_connection(url, source_system=255)
        print(f"[hil] connecting {url} ...")
        self.m.wait_heartbeat(timeout=60)
        print(f"[hil] heartbeat from sys {self.m.target_system} comp {self.m.target_component}")
        self.named: dict[str, float] = {}
        self.rc = [65535] * 18
        self.rc[:8] = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        self.last_rc_send = 0.0
        # ask for the data streams (SITL is quiet until a GCS asks)
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    def pump(self, seconds: float, quiet: bool = False):
        t_end = time.time() + seconds
        while time.time() < t_end:
            now = time.time()
            if now - self.last_rc_send > 0.1:
                self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *self.rc)
                self.last_rc_send = now
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "NAMED_VALUE_FLOAT":
                self.named[msg.name] = msg.value
            elif t == "STATUSTEXT" and not quiet:
                print(f"[ap] {msg.text}")

    def set_rc(self, ch: int, pwm: int):
        self.rc[ch - 1] = pwm

    def centre_sticks(self):
        for i in range(8):
            self.rc[i] = 1500

    def param_set(self, name: str, value: float):
        self.m.param_set_send(name, value)
        self.pump(0.3, quiet=True)

    def param_get(self, name: str, timeout: float = 3.0):
        self.m.param_fetch_one(name)
        t_end = time.time() + timeout
        while time.time() < t_end:
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg and msg.param_id == name:
                return msg.param_value
        return None

    def set_mode(self, name: str):
        self.m.set_mode(ROVER_MODES[name])
        self.pump(0.5)

    def arm(self, force: bool = True) -> bool:
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     1, 21196 if force else 0, 0, 0, 0, 0, 0)
        for _ in range(20):
            self.pump(0.25, quiet=True)
            if self.m.motors_armed():
                return True
        return False

    def disarm(self) -> bool:
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                     0, 21196, 0, 0, 0, 0, 0)
        for _ in range(20):
            self.pump(0.25, quiet=True)
            if not self.m.motors_armed():
                return True
        return False

    def status(self) -> str:
        n = self.named
        return (f"FAIL={n.get('PPO_FAIL', float('nan')):.0f} PGZ={n.get('PPO_PGZ', float('nan')):+.2f} "
                f"VX={n.get('PPO_VX', float('nan')):+.2f} fwd={n.get('PPO_MS', float('nan')):.3f}ms")


def cmd_monitor(link: Link, args):
    while True:
        link.pump(1.0)
        print("[hil]", link.status())


def check(name: str, cond: bool, detail: str, results: list):
    results.append((name, cond, detail))
    print(f"[hil] {'PASS' if cond else 'FAIL'}  {name}: {detail}")


def cmd_battery(link: Link, args):
    results: list = []
    link.param_set("NNM_ENABLE", 1)
    link.param_set("NNM_POLICY", args.policy)
    link.set_mode("MANUAL")
    link.centre_sticks()
    link.pump(2.0)

    # --- arm & stand
    armed = link.arm()
    check("arm", armed, "armed" if armed else "could not arm", results)
    if not armed:
        return results
    link.pump(15.0)
    n = link.named
    check("stand", n.get("PPO_FAIL", 9) == 0 and n.get("PPO_PGZ", 0) < -0.9,
          link.status(), results)

    # --- forward
    link.set_rc(2, 1750)
    link.pump(12.0)
    n = link.named
    check("forward", n.get("PPO_FAIL", 9) == 0 and abs(n.get("PPO_VX", 0) - args.vx_expect) < 0.05 and n.get("PPO_PGZ", 0) < -0.85,
          link.status(), results)
    link.centre_sticks()
    link.pump(3.0)

    # --- lateral
    link.set_rc(1, 1650)
    link.pump(8.0)
    n = link.named
    check("lateral", n.get("PPO_FAIL", 9) == 0 and n.get("PPO_PGZ", 0) < -0.85, link.status(), results)
    link.centre_sticks()
    link.pump(3.0)

    # --- turn
    link.set_rc(4, 1750)
    link.pump(8.0)
    n = link.named
    check("turn", n.get("PPO_FAIL", 9) == 0 and n.get("PPO_PGZ", 0) < -0.85, link.status(), results)
    link.centre_sticks()
    link.pump(2.0)

    # --- hold forces zero twist even with a stick deflected
    link.set_rc(2, 1750)
    link.set_mode("HOLD")
    link.pump(5.0)
    n = link.named
    check("hold", abs(n.get("PPO_VX", 1)) < 1e-3 and n.get("PPO_FAIL", 9) == 0, link.status(), results)
    link.centre_sticks()
    link.set_mode("MANUAL")
    link.pump(3.0)

    # --- disarm
    dis = link.disarm()
    link.pump(2.0)
    check("disarm", dis and link.named.get("PPO_FAIL", 0) == 1, link.status(), results)
    return results


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["monitor", "battery"])
    ap.add_argument("--url", default="tcp:127.0.0.1:5760")
    ap.add_argument("--policy", type=int, default=0)
    ap.add_argument("--vx-expect", type=float, default=0.2, help="expected PPO_VX for RC2=1750 (0.5 * NNM_VX_MAX)")
    ap.add_argument("--log-dir", default="sitl/run/logs", help="SITL dataflash dir for the in-situ parity check ('' to skip)")
    ap.add_argument("--onnx", default=None, help="ONNX to replay (default: the policy selected by --policy)")
    args = ap.parse_args()
    link = Link(args.url)
    if args.cmd == "monitor":
        cmd_monitor(link, args)
        return
    results = cmd_battery(link, args)

    # in-situ parity: replay the firmware's logged observations through the ONNX policy
    if args.log_dir:
        import glob
        import os
        import subprocess
        from pathlib import Path

        logs = sorted(glob.glob(os.path.join(args.log_dir, "*.BIN")), key=os.path.getmtime)
        if logs:
            here = Path(__file__).resolve().parents[1]
            # compare against what the firmware ran: the SD policy when SITL has one, else the baked MLP
            sd = here / "sitl" / "run" / "APM" / "nnm" / "microduck" / "policies" / "walk.nnm"
            onnx = args.onnx or str(sd if sd.is_file() else here / "policies" / "microduck_mlp_2048x2000_it1999.onnx")
            link.pump(1.0, quiet=True)  # let the logger flush
            out = subprocess.run([sys.executable, str(here / "tools" / "log_parity.py"), logs[-1], onnx],
                                 capture_output=True, text=True)
            print(out.stdout.strip())
            check("parity", "PARITY OK" in out.stdout, out.stdout.strip().splitlines()[1] if out.stdout else out.stderr[-200:], results)

    n_fail = sum(1 for _, ok, _ in results if not ok)
    print("\n[hil] SUMMARY")
    for name, ok, detail in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name:8s} {detail}")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
