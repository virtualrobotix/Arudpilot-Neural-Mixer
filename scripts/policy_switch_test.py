#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""SITL check of the dual-slot SD policy switch and of the robot-id guard.

Needs, in the SITL microSD (sitl/run/APM/nnm/microduck/policies/), three files in
alphabetical order: walk.nnm (0), walk_b.nnm (1, same robot), x_microban.nnm (2, another
robot). With the plant and SITL running:

    .venv/bin/python scripts/policy_switch_test.py
"""

from __future__ import annotations

import sys
import time

from pymavlink import mavutil


def main() -> None:
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
    m.wait_heartbeat(timeout=60)
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    named: dict[str, float] = {}
    texts: list[str] = []
    rc = [1500] * 8 + [65535] * 10

    def pump(sec: float) -> None:
        end = time.time() + sec
        last = 0.0
        while time.time() < end:
            if time.time() - last > 0.1:
                m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
                last = time.time()
            msg = m.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            if msg.get_type() == "NAMED_VALUE_FLOAT":
                named[msg.name] = msg.value
            elif msg.get_type() == "STATUSTEXT" and "NNMixer" in msg.text:
                texts.append(msg.text)
                print("  ", msg.text)

    def set_param(name: str, value: float) -> None:
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), value,
                             mavutil.mavlink.MAV_PARAM_TYPE_INT8)

    def mode(num: int) -> None:
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                1, num, 0, 0, 0, 0, 0)

    results = []

    def check(name: str, ok: bool, detail: str) -> None:
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}  {name:18s} {detail}")

    pump(3)
    set_param("NNM_POLICY", 0)
    mode(0)
    pump(1)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    pump(5)
    check("stand slot 0", named.get("PPO_FAIL") == 0 and named.get("PPO_PGZ", 0) < -0.9,
          f"FAIL={named.get('PPO_FAIL')} PGZ={named.get('PPO_PGZ', 0):.2f} SLOT={named.get('PPO_SLOT')}")

    rc[1] = 1750                       # walking: a switch request must wait
    pump(2)
    set_param("NNM_POLICY", 1)
    pump(3)
    check("deferred in MANUAL", named.get("PPO_SLOT") == 0, f"SLOT={named.get('PPO_SLOT')} while walking")

    rc[1] = 1500
    mode(4)                            # HOLD: switch allowed
    pump(4)
    check("switch in HOLD", named.get("PPO_SLOT") == 1 and named.get("PPO_FAIL") == 0
          and named.get("PPO_PGZ", 0) < -0.9,
          f"SLOT={named.get('PPO_SLOT')} FAIL={named.get('PPO_FAIL')} PGZ={named.get('PPO_PGZ', 0):.2f}")

    mode(0)
    rc[1] = 1750
    pump(4)
    check("walk on slot 1", named.get("PPO_FAIL") == 0 and named.get("PPO_PGZ", 0) < -0.9
          and named.get("PPO_VX", 0) > 0.1, f"VX={named.get('PPO_VX', 0):.2f} PGZ={named.get('PPO_PGZ', 0):.2f}")

    rc[1] = 1500
    mode(4)
    n_before = len(texts)
    set_param("NNM_POLICY", 2)          # x_microban.nnm: another robot
    pump(4)
    rejected = any("rejected" in t for t in texts[n_before:])
    check("reject other robot", rejected and named.get("PPO_SLOT") == 1 and named.get("PPO_FAIL") == 0,
          f"SLOT={named.get('PPO_SLOT')} FAIL={named.get('PPO_FAIL')} rejected={rejected}")

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
    set_param("NNM_POLICY", 0)
    pump(1)
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
