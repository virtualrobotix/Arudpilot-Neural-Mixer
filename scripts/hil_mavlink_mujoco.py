#!/usr/bin/env python3
"""Run the MuJoCo MicroDuck plant against AP_MicroDuck on real autopilot hardware.

Transport (MAVLink 2 over USB/serial):
  plant -> autopilot  DEBUG_FLOAT_ARRAY "MDK_HIL"
      q[14], qd[14], gyro_flu[3], projected_gravity_flu[3]
  autopilot -> plant  SERVO_OUTPUT_RAW
      servo1..14, encoded exactly like SITL (1500 us + rad / 0.003)

This is policy-in-the-loop HIL: AP_MicroDuck inference, scheduler, watchdog and
servo mapping run on the STM32H7; MuJoCo provides the robot state and dynamics.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from plant.mujoco_json_plant import (  # noqa: E402
    DEFAULT_MJCF,
    DEFAULT_POSE,
    Plant,
    pwm_to_rad,
)

N_JOINTS = 14
HIL_RATE_HZ = 100.0
PHYSICS_STEPS_PER_TICK = 2


def set_param(master, name: str, value: float, timeout: float = 3.0) -> None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return
    raise TimeoutError(f"no PARAM_VALUE acknowledgement for {name}")


def configure(master) -> None:
    params = {
        # Current Rover uses ARMING_SKIPCHK rather than ARMING_CHECK.
        "ARMING_SKIPCHK": 65535,
        "BRD_SAFETY_DEFLT": 0,
        "FS_THR_ENABLE": 0,
        "FS_GCS_ENABLE": 0,
        "MDK_ENABLE": 1,
        "MDK_POLICY": 0,
        "MDK_LOG": 1,
        "MDK_WD_MS": 100,
    }
    for i in range(N_JOINTS):
        params[f"SERVO{i + 1}_FUNCTION"] = 94 + i
        params[f"SERVO{i + 1}_MIN"] = 800
        params[f"SERVO{i + 1}_MAX"] = 2200
    for name, value in params.items():
        set_param(master, name, value)

def request_output_stream(master) -> None:
    # SERVO_OUTPUT_RAW does not implement MAV_CMD_SET_MESSAGE_INTERVAL on
    # ArduRover. The legacy RC_CHANNELS stream request yields ~40-50 Hz over USB.
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        50,
        1,
    )


def connect(port: str, baud: int):
    master = mavutil.mavlink_connection(
        port,
        baud=baud,
        autoreconnect=True,
        source_system=255,
        source_component=190,
    )
    heartbeat = master.wait_heartbeat(timeout=12)
    if heartbeat is None:
        master.close()
        raise RuntimeError(f"no MAVLink heartbeat on {port}")
    master.target_system = heartbeat.get_srcSystem()
    master.target_component = heartbeat.get_srcComponent()
    return master


def reboot_and_reconnect(master, port: str, baud: int):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0, 0, 0, 0, 0, 0,
    )
    master.close()
    time.sleep(3.0)
    last_error = None
    for _ in range(8):
        try:
            return connect(port, baud)
        except Exception as error:  # noqa: BLE001
            last_error = error
            time.sleep(1.0)
    raise RuntimeError(f"autopilot did not reconnect after reboot: {last_error}")


def arm(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        21196,  # force arm: this is a bench HIL with no actuators connected
        0, 0, 0, 0, 0,
    )
    end = time.monotonic() + 8.0
    while time.monotonic() < end:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is not None and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return
    raise RuntimeError("autopilot did not arm")


def disarm(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        21196,
        0, 0, 0, 0, 0,
    )


def servo_values(msg) -> np.ndarray:
    return np.array(
        [getattr(msg, f"servo{i}_raw", 0) for i in range(1, N_JOINTS + 1)],
        dtype=np.float64,
    )


def hil_sample(plant: Plant) -> list[float]:
    d = plant.data
    q = np.asarray(d.qpos[plant.qpos_idx], dtype=np.float32)
    qd = np.asarray(d.qvel[plant.qvel_idx], dtype=np.float32)
    gyro_flu = np.asarray(
        d.sensordata[plant.gyro_adr : plant.gyro_adr + 3], dtype=np.float32
    )
    w, x, y, z = d.xquat[plant.trunk_id]
    gravity_flu = np.array(
        [
            -2.0 * (x * z - w * y),
            -2.0 * (y * z + w * x),
            -(1.0 - 2.0 * (x * x + y * y)),
        ],
        dtype=np.float32,
    )
    data = np.concatenate((q, qd, gyro_flu, gravity_flu))
    return np.pad(data, (0, 58 - data.size)).astype(np.float32).tolist()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem01")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--no-bam", action="store_true")
    parser.add_argument("--configure", action="store_true",
                        help="write the required MDK/SERVO/arming parameters")
    args = parser.parse_args()

    master = connect(args.port, args.baud)
    print(f"[hil] connected sysid={master.target_system} compid={master.target_component}")

    if args.configure:
        configure(master)
        print("[hil] parameters configured; rebooting to apply safety defaults")
        master = reboot_and_reconnect(master, args.port, args.baud)
        print("[hil] autopilot reconnected")
        time.sleep(4.0)

    request_output_stream(master)
    plant = Plant(args.mjcf, use_bam=not args.no_bam, viewer=not args.no_viewer)
    pwm = np.zeros(N_JOINTS, dtype=np.float64)
    ppo_ms = math.nan
    ppo_fail = math.nan
    ppo_samples: list[float] = []
    cpu_load_samples: list[float] = []
    seq = 0
    period = 1.0 / HIL_RATE_HZ
    next_tick = time.monotonic()
    started = next_tick

    # Prime the firmware watchdog before arming.
    for _ in range(5):
        master.mav.debug_float_array_send(
            int(time.time_ns() // 1000), b"MDK_HIL", seq, hil_sample(plant)
        )
        seq = (seq + 1) & 0xFFFF
        time.sleep(period)
    arm(master)
    print("[hil] armed; STM32H7 policy loop active")

    try:
        while time.monotonic() - started < args.seconds:
            while True:
                msg = master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == "SERVO_OUTPUT_RAW":
                    candidate = servo_values(msg)
                    if np.count_nonzero(candidate) >= N_JOINTS:
                        pwm = candidate
                elif msg.get_type() == "NAMED_VALUE_FLOAT":
                    name = msg.name.rstrip("\x00")
                    if name == "PPO_MS":
                        ppo_ms = msg.value
                        ppo_samples.append(float(msg.value))
                    elif name == "PPO_FAIL":
                        ppo_fail = msg.value
                elif msg.get_type() == "SYS_STATUS":
                    cpu_load_samples.append(float(msg.load) * 0.1)

            if np.count_nonzero(pwm) >= N_JOINTS:
                target = pwm_to_rad(pwm)
                for _ in range(PHYSICS_STEPS_PER_TICK):
                    plant.step(target)
            else:
                plant.idle()

            master.mav.debug_float_array_send(
                int(time.time_ns() // 1000), b"MDK_HIL", seq, hil_sample(plant)
            )
            seq = (seq + 1) & 0xFFFF
            plant.sync_viewer()

            if seq % int(HIL_RATE_HZ) == 0:
                print(
                    f"[hil] t={plant.t:6.2f}s PPO_MS={ppo_ms:.3f} "
                    f"PPO_FAIL={ppo_fail:.0f} tilt={plant.tilt_deg():.1f}deg "
                    f"pwm=[{int(pwm.min())},{int(pwm.max())}]"
                )

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        disarm(master)
        print("[hil] disarmed")
        if ppo_samples:
            values = np.asarray(ppo_samples)
            print(
                f"[hil] forward latency: n={len(values)} "
                f"p50={np.percentile(values, 50):.3f} ms "
                f"p99={np.percentile(values, 99):.3f} ms "
                f"max={values.max():.3f} ms"
            )
        if cpu_load_samples:
            values = np.asarray(cpu_load_samples)
            print(
                f"[hil] autopilot CPU load: mean={values.mean():.1f}% "
                f"max={values.max():.1f}%"
            )


if __name__ == "__main__":
    main()
