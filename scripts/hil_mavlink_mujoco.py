#!/usr/bin/env python3
"""Run the MuJoCo NNMixer plant against AP_NNMixer on real autopilot hardware.

Transport (MAVLink 2 over USB/serial):
  plant -> autopilot  DEBUG_FLOAT_ARRAY "NNM_HIL"
      q[14], qd[14], gyro_flu[3], projected_gravity_flu[3]
  autopilot -> plant  DEBUG_FLOAT_ARRAY "NNM_ACT"
      14 joint targets in radians, emitted at every 50 Hz policy tick

SERVO_OUTPUT_RAW remains a compatibility fallback for older firmware.

This is policy-in-the-loop HIL: AP_NNMixer inference, scheduler, watchdog and
servo mapping run on the STM32H7; MuJoCo provides the robot state and dynamics.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# DEBUG_FLOAT_ARRAY exists only in MAVLink 2; pymavlink defaults to v1.
os.environ.setdefault("MAVLINK20", "1")

import numpy as np
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from plant.mujoco_json_plant import (  # noqa: E402
    DEFAULT_MJCF,
    DEFAULT_POSE,
    Plant,
    VideoRecorder,
    pwm_to_rad,
)

N_JOINTS = 14
HIL_RATE_HZ = 50.0
PHYSICS_STEPS_PER_TICK = 4
LEVEL_GRAVITY_FLU = np.array([0.0, 0.0, -1.0])


@dataclass(frozen=True)
class Phase:
    name: str
    duration: float
    rc1: int = 1500  # lateral
    rc2: int = 1500  # forward
    rc4: int = 1500  # yaw
    mode: int = 0    # Rover MANUAL=0, HOLD=4


BATTERY = [
    Phase("stand", 10.0),
    Phase("forward +0.4 m/s", 12.0, rc2=2000),
    Phase("lateral +0.3 m/s", 10.0, rc1=2000),
    Phase("yaw +1.0 rad/s", 8.0, rc4=2000),
    Phase("HOLD", 3.0, mode=4),
    Phase("MANUAL / stop", 2.0),
]


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
        "NNM_ENABLE": 1,
        "NNM_POLICY": 0,
        "NNM_LOG": 1,
        "NNM_WD_MS": 100,
        "RC1_MIN": 1000,
        "RC1_MAX": 2000,
        "RC1_DZ": 20,
        "RC2_MIN": 1000,
        "RC2_MAX": 2000,
        "RC2_DZ": 20,
        "RC3_MIN": 1000,
        "RC3_MAX": 2000,
        "RC4_MIN": 1000,
        "RC4_MAX": 2000,
        "RC4_DZ": 20,
    }
    for i in range(N_JOINTS):
        params[f"SERVO{i + 1}_FUNCTION"] = 94 + i
        params[f"SERVO{i + 1}_MIN"] = 800
        params[f"SERVO{i + 1}_MAX"] = 2200
    for name, value in params.items():
        set_param(master, name, value)

def request_output_stream(master) -> None:
    # SERVO_OUTPUT_RAW does not implement MAV_CMD_SET_MESSAGE_INTERVAL on
    # ArduRover. Ask the legacy stream for 100 Hz: Rover's scheduler caps servo
    # telemetry near the 50 Hz policy rate, while a 50 Hz request measured only
    # about 40 Hz on the Pixhawk 6C USB link.
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        100,
        1,
    )


def request_attitude_stream(master, rate_hz: int = 25) -> None:
    """ATTITUDE carries the physical board's own AHRS solution, used to feed its
    real motion back into the simulated robot."""
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        rate_hz,
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


def set_mode(master, mode: int) -> None:
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode,
    )


def send_rc(master, rc1: int, rc2: int, rc4: int) -> None:
    channels = [rc1, rc2, 1500, rc4] + [65535] * 14
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *channels,
    )


def phase_at(phases: list[Phase], elapsed: float) -> tuple[int, Phase]:
    cursor = 0.0
    for index, phase in enumerate(phases):
        cursor += phase.duration
        if elapsed < cursor:
            return index, phase
    return len(phases) - 1, phases[-1]


def servo_values(msg) -> np.ndarray:
    return np.array(
        [getattr(msg, f"servo{i}_raw", 0) for i in range(1, N_JOINTS + 1)],
        dtype=np.float64,
    )


def board_attitude_flu(msg) -> tuple[np.ndarray, np.ndarray]:
    """Gravity direction and angular rate of the physical autopilot, in the trunk
    FLU convention the policy was trained with. ATTITUDE is body FRD."""
    sr, cr = math.sin(msg.roll), math.cos(msg.roll)
    sp, cp = math.sin(msg.pitch), math.cos(msg.pitch)
    down_frd = np.array([-sp, sr * cp, cr * cp])
    gravity_flu = np.array([down_frd[0], -down_frd[1], -down_frd[2]])
    gyro_flu = np.array([msg.rollspeed, -msg.pitchspeed, -msg.yawspeed])
    return gravity_flu, gyro_flu


class HilHud:
    """Live overlay for HIL recordings: PPO runs on the Pixhawk, not on the host."""

    FONTS = [
        "/System/Library/Fonts/Menlo.ttc",
        "/System/Library/Fonts/Monaco.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    ]

    def __init__(self, width: int, height: int):
        from PIL import ImageFont

        self.width = width
        self.height = height
        self.font = None
        self.font_b = None
        for path in self.FONTS:
            if Path(path).exists():
                self.font = ImageFont.truetype(path, 15)
                self.font_b = ImageFont.truetype(path, 18)
                break
        if self.font is None:
            self.font = ImageFont.load_default()
            self.font_b = self.font
        self.phase = "stand"
        self.hil_att = 0
        self.ppo_ms = math.nan
        self.ppo_p50 = math.nan
        self.ppo_p99 = math.nan
        self.ppo_max = math.nan
        self.cpu = math.nan
        self.vx = math.nan
        self.fail = math.nan
        self.board_roll = math.nan
        self.board_pitch = math.nan
        self.board_tilt = math.nan
        self.action_hz = math.nan

    def draw(self, frame: np.ndarray, plant: Plant, driving: bool) -> np.ndarray:
        from PIL import Image, ImageDraw

        img = Image.fromarray(frame)
        d = ImageDraw.Draw(img, "RGBA")
        W, H = self.width, self.height
        d.rectangle([0, 0, W, 34], fill=(10, 20, 35, 220))
        d.text(
            (10, 8),
            "Pixhawk 6C Mini  STM32H743 @ 480 MHz  |  PPO MLP on-board 50 Hz  |  MuJoCo plant over MAVLink HIL",
            font=self.font_b,
            fill=(230, 235, 245, 255),
        )

        panel_w = 430
        d.rectangle([10, 44, 10 + panel_w, 236], fill=(10, 20, 35, 205), outline=(255, 200, 80, 255))
        d.text((22, 52), "PPO on Pixhawk 6C (not on the host)", font=self.font_b, fill=(255, 200, 80, 255))
        ppo_line = (
            f"forward  last={self.ppo_ms:5.3f} ms   "
            f"p50={self.ppo_p50:5.3f} ms   p99={self.ppo_p99:5.3f} ms"
            if not math.isnan(self.ppo_ms)
            else "forward  waiting for NAMED_VALUE_FLOAT PPO_MS"
        )
        d.text((22, 80), ppo_line, font=self.font, fill=(120, 220, 160, 255))
        max_line = (
            f"max={self.ppo_max:5.3f} ms   CPU={self.cpu:4.1f}%   "
            f"policy stream={self.action_hz:4.1f} Hz"
            if not math.isnan(self.ppo_max)
            else "timing from STM32H7 wall clock inside AP_NNMixer::update()"
        )
        d.text((22, 100), max_line, font=self.font, fill=(230, 235, 245, 255))
        d.text(
            (22, 122),
            f"vx cmd={self.vx:+.3f} m/s   PPO_FAIL={0 if math.isnan(self.fail) else int(self.fail)}   "
            f"NNM_HIL_ATT={self.hil_att}",
            font=self.font,
            fill=(230, 235, 245, 255),
        )

        d.text((22, 150), "Pixhawk IMU perturbation (real board)", font=self.font_b, fill=(255, 200, 80, 255))
        if math.isnan(self.board_tilt):
            imu_line = "waiting for ATTITUDE from the flight controller"
        else:
            imu_line = (
                f"roll={self.board_roll:+6.1f} deg  pitch={self.board_pitch:+6.1f} deg  "
                f"tilt={self.board_tilt:5.1f} deg"
            )
        d.text((22, 178), imu_line, font=self.font, fill=(120, 220, 160, 255))
        d.text(
            (22, 198),
            "mode 2: simulated body + board IMU as disturbance",
            font=self.font,
            fill=(170, 185, 205, 255),
        )

        badge = self.phase
        tw = d.textlength(badge, font=self.font_b) + 24
        d.rectangle([W - tw - 10, 44, W - 10, 76], fill=(200, 60, 40, 220))
        d.text((W - tw + 2, 51), badge, font=self.font_b, fill=(255, 255, 255, 255))

        x, yv, z = plant.data.qpos[plant.free_qpos : plant.free_qpos + 3]
        status = (
            f"MuJoCo  t={plant.t:6.2f}s  xy=({x:+.2f},{yv:+.2f}) m  z={z:.3f} m  "
            f"robot tilt={plant.tilt_deg():4.1f} deg  yaw={plant.yaw_deg():+6.1f} deg  "
            f"{'ARMED  policy driving 14 joints' if driving else 'idle'}"
        )
        d.rectangle([0, H - 30, W, H], fill=(10, 20, 35, 220))
        d.text((10, H - 22), status, font=self.font, fill=(230, 235, 245, 255))
        return np.asarray(img)


def hil_sample(
    plant: Plant,
    gyro_extra: np.ndarray | None = None,
    gravity_extra: np.ndarray | None = None,
) -> list[float]:
    d = plant.data
    q = np.asarray(d.qpos[plant.qpos_idx], dtype=np.float32)
    qd = np.asarray(d.qvel[plant.qvel_idx], dtype=np.float32)
    gyro_flu = np.asarray(
        d.sensordata[plant.gyro_adr : plant.gyro_adr + 3], dtype=np.float64
    )
    w, x, y, z = d.xquat[plant.trunk_id]
    gravity_flu = np.array(
        [
            -2.0 * (x * z - w * y),
            -2.0 * (y * z + w * x),
            -(1.0 - 2.0 * (x * x + y * y)),
        ],
    )
    if gyro_extra is not None:
        gyro_flu = gyro_flu + gyro_extra
    if gravity_extra is not None:
        gravity_flu = gravity_flu + gravity_extra
        norm = np.linalg.norm(gravity_flu)
        if norm > 1e-6:
            gravity_flu = gravity_flu / norm
    data = np.concatenate(
        (q, qd, gyro_flu.astype(np.float32), gravity_flu.astype(np.float32))
    )
    return np.pad(data, (0, 58 - data.size)).astype(np.float32).tolist()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem01")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--battery", action="store_true",
                        help="run stand/forward/lateral/yaw/HOLD/MANUAL test sequence")
    parser.add_argument("--battery-scale", type=float, default=1.0,
                        help="multiply battery phase durations (use 0.25 for a quick smoke test)")
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--no-bam", action="store_true")
    parser.add_argument("--configure", action="store_true",
                        help="write the required NNM/SERVO/arming parameters")
    parser.add_argument("--imu-inject", action="store_true",
                        help="add the physical board's own tilt and rotation to the "
                             "state sent to the policy, so moving the autopilot "
                             "disturbs the simulated robot")
    parser.add_argument("--imu-gain", type=float, default=1.0,
                        help="scale of the injected board motion")
    parser.add_argument("--hil-att", type=int, choices=(0, 1, 2), default=None,
                        help="firmware attitude source (NNM_HIL_ATT): 0 simulated body, "
                             "1 board IMU, 2 both")
    parser.add_argument(
        "--video",
        type=Path,
        default=None,
        help="record an mp4 of the plant with Pixhawk PPO timing and IMU overlay",
    )
    args = parser.parse_args()

    master = connect(args.port, args.baud)
    print(f"[hil] connected sysid={master.target_system} compid={master.target_component}")

    if args.configure:
        configure(master)
        print("[hil] parameters configured; rebooting to apply safety defaults")
        master = reboot_and_reconnect(master, args.port, args.baud)
        print("[hil] autopilot reconnected")
        time.sleep(4.0)

    if args.hil_att is not None:
        set_param(master, "NNM_HIL_ATT", args.hil_att)
        print(f"[hil] firmware attitude source NNM_HIL_ATT={args.hil_att}")

    request_output_stream(master)
    want_board_att = args.imu_inject or args.video is not None or (args.hil_att in (1, 2))
    if args.imu_inject:
        print(f"[hil] board IMU injection on (gain {args.imu_gain})")
    if want_board_att:
        request_attitude_stream(master)
        print("[hil] ATTITUDE stream requested for Pixhawk IMU overlay")
    plant = Plant(args.mjcf, use_bam=not args.no_bam, viewer=not args.no_viewer)
    pwm = np.zeros(N_JOINTS, dtype=np.float64)
    ppo_ms = math.nan
    ppo_fail = math.nan
    ppo_vx = math.nan
    ppo_samples: list[float] = []
    cpu_load_samples: list[float] = []
    servo_message_count = 0
    first_servo_wall = None
    last_servo_wall = None
    board_gravity_extra = None
    board_gyro_extra = None
    board_tilt_deg = 0.0
    board_roll_deg = math.nan
    board_pitch_deg = math.nan
    action_message_count = 0
    first_action_wall = None
    last_action_wall = None
    direct_target = None
    seq = 0
    period = 1.0 / HIL_RATE_HZ
    next_tick = time.monotonic()
    started = next_tick
    phases = (
        [
            Phase(p.name, p.duration * args.battery_scale, p.rc1, p.rc2, p.rc4, p.mode)
            for p in BATTERY
        ]
        if args.battery
        else [Phase("stand", args.seconds)]
    )
    run_seconds = sum(p.duration for p in phases) if args.battery else args.seconds
    phase_index = -1
    phase_start_xy = np.zeros(2)
    phase_stats: list[dict] = []
    current_stats = None
    hud = None
    video = None
    if args.video is not None:
        args.video.parent.mkdir(parents=True, exist_ok=True)
        hud = HilHud(960, 540)
        hud.hil_att = 0 if args.hil_att is None else args.hil_att
        video = VideoRecorder(plant, args.video, hud=hud)
        print(f"[hil] recording {args.video}")

    # Prime the firmware watchdog before arming.
    for _ in range(5):
        master.mav.debug_float_array_send(
            int(time.time_ns() // 1000), b"NNM_HIL", seq, hil_sample(plant)
        )
        seq = (seq + 1) & 0xFFFF
        time.sleep(period)
    arm(master)
    print("[hil] armed; STM32H7 policy loop active")
    set_mode(master, phases[0].mode)
    started = time.monotonic()
    next_tick = started

    try:
        while time.monotonic() - started < run_seconds:
            elapsed = time.monotonic() - started
            next_phase_index, phase = phase_at(phases, elapsed)
            if next_phase_index != phase_index:
                if current_stats is not None:
                    current_stats["end_xy"] = np.asarray(
                        plant.data.qpos[plant.free_qpos : plant.free_qpos + 2]
                    ).copy()
                    phase_stats.append(current_stats)
                phase_index = next_phase_index
                phase_start_xy = np.asarray(
                    plant.data.qpos[plant.free_qpos : plant.free_qpos + 2]
                ).copy()
                current_stats = {
                    "name": phase.name,
                    "start_xy": phase_start_xy,
                    "max_tilt": 0.0,
                    "fail": False,
                }
                set_mode(master, phase.mode)
                if hud is not None:
                    hud.phase = phase.name
                print(
                    f"[hil] phase {phase_index + 1}/{len(phases)}: {phase.name} "
                    f"RC=({phase.rc1},{phase.rc2},{phase.rc4}) mode={phase.mode}"
                )

            # RC override is a live control source and must be refreshed.
            if seq % 10 == 0:
                send_rc(master, phase.rc1, phase.rc2, phase.rc4)

            while True:
                try:
                    msg = master.recv_match(blocking=False)
                except Exception as error:  # noqa: BLE001
                    print(f"[hil] serial error: {error}")
                    run_seconds = 0
                    break
                if msg is None:
                    break
                if msg.get_type() == "SERVO_OUTPUT_RAW":
                    candidate = servo_values(msg)
                    if np.count_nonzero(candidate) >= N_JOINTS:
                        pwm = candidate
                        servo_message_count += 1
                        last_servo_wall = time.monotonic()
                        if first_servo_wall is None:
                            first_servo_wall = last_servo_wall
                elif msg.get_type() == "NAMED_VALUE_FLOAT":
                    name = msg.name.rstrip("\x00")
                    if name == "PPO_MS":
                        ppo_ms = msg.value
                        ppo_samples.append(float(msg.value))
                    elif name == "PPO_FAIL":
                        ppo_fail = msg.value
                        # Fail 1 is "disarmed". The last disarmed telemetry can
                        # arrive after arm() and would false-fail the stand phase.
                        if (
                            current_stats is not None
                            and msg.value != 0
                            and not (msg.value == 1.0 and elapsed < 2.0)
                        ):
                            current_stats["fail"] = True
                    elif name == "PPO_VX":
                        ppo_vx = msg.value
                elif msg.get_type() == "DEBUG_FLOAT_ARRAY":
                    name = msg.name.rstrip("\x00")
                    if name == "NNM_ACT":
                        direct_target = np.asarray(msg.data[:N_JOINTS], dtype=np.float64)
                        action_message_count += 1
                        last_action_wall = time.monotonic()
                        if first_action_wall is None:
                            first_action_wall = last_action_wall
                elif msg.get_type() == "ATTITUDE":
                    gravity, gyro = board_attitude_flu(msg)
                    board_roll_deg = math.degrees(msg.roll)
                    board_pitch_deg = math.degrees(msg.pitch)
                    board_tilt_deg = math.degrees(
                        math.acos(min(1.0, max(-1.0, -gravity[2])))
                    )
                    if args.imu_inject:
                        board_gravity_extra = args.imu_gain * (gravity - LEVEL_GRAVITY_FLU)
                        board_gyro_extra = args.imu_gain * gyro
                elif msg.get_type() == "SYS_STATUS":
                    cpu_load_samples.append(float(msg.load) * 0.1)

            if direct_target is not None:
                target = direct_target
                for _ in range(PHYSICS_STEPS_PER_TICK):
                    plant.step(target)
            elif np.count_nonzero(pwm) >= N_JOINTS:
                target = pwm_to_rad(pwm)
                for _ in range(PHYSICS_STEPS_PER_TICK):
                    plant.step(target)
            else:
                plant.idle()

            master.mav.debug_float_array_send(
                int(time.time_ns() // 1000),
                b"NNM_HIL",
                seq,
                hil_sample(plant, board_gyro_extra, board_gravity_extra),
            )
            seq = (seq + 1) & 0xFFFF
            plant.sync_viewer()
            if video is not None:
                video.driving = direct_target is not None or np.count_nonzero(pwm) >= N_JOINTS
                if hud is not None:
                    hud.ppo_ms = ppo_ms
                    hud.fail = ppo_fail
                    hud.vx = ppo_vx
                    hud.board_roll = board_roll_deg
                    hud.board_pitch = board_pitch_deg
                    hud.board_tilt = board_tilt_deg
                    if ppo_samples:
                        values = np.asarray(ppo_samples)
                        hud.ppo_p50 = float(np.percentile(values, 50))
                        hud.ppo_p99 = float(np.percentile(values, 99))
                        hud.ppo_max = float(values.max())
                    if cpu_load_samples:
                        hud.cpu = float(np.mean(cpu_load_samples[-8:]))
                    if (
                        action_message_count > 1
                        and first_action_wall is not None
                        and last_action_wall is not None
                        and last_action_wall > first_action_wall
                    ):
                        hud.action_hz = (action_message_count - 1) / (
                            last_action_wall - first_action_wall
                        )
                video.maybe_capture()
            if current_stats is not None:
                current_stats["max_tilt"] = max(
                    current_stats["max_tilt"], plant.tilt_deg()
                )

            if seq % int(HIL_RATE_HZ) == 0:
                print(
                    f"[hil] t={plant.t:6.2f}s PPO_MS={ppo_ms:.3f} "
                    f"PPO_FAIL={ppo_fail:.0f} vx={ppo_vx:+.3f} "
                    f"tilt={plant.tilt_deg():.1f}deg "
                    f"pwm=[{int(pwm.min())},{int(pwm.max())}]"
                    + (f" board_tilt={board_tilt_deg:.1f}deg" if not math.isnan(board_roll_deg) else "")
                )

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        if video is not None:
            try:
                video.close()
            except Exception as error:  # noqa: BLE001
                print(f"[hil] video close: {error}")
        if current_stats is not None:
            current_stats["end_xy"] = np.asarray(
                plant.data.qpos[plant.free_qpos : plant.free_qpos + 2]
            ).copy()
            phase_stats.append(current_stats)
        send_rc(master, 1500, 1500, 1500)
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
        if (
            servo_message_count > 1
            and first_servo_wall is not None
            and last_servo_wall is not None
            and last_servo_wall > first_servo_wall
        ):
            servo_hz = (servo_message_count - 1) / (last_servo_wall - first_servo_wall)
            print(f"[hil] servo output stream: {servo_hz:.1f} Hz")
        if (
            action_message_count > 1
            and first_action_wall is not None
            and last_action_wall is not None
            and last_action_wall > first_action_wall
        ):
            action_hz = (action_message_count - 1) / (last_action_wall - first_action_wall)
            print(f"[hil] direct policy-action stream: {action_hz:.1f} Hz")
        if args.battery:
            print("[hil] battery report:")
            all_ok = True
            for stat in phase_stats:
                delta = stat["end_xy"] - stat["start_xy"]
                ok = not stat["fail"] and stat["max_tilt"] < 45.0
                all_ok &= ok
                print(
                    f"  {'PASS' if ok else 'FAIL'} {stat['name']:<20} "
                    f"dxy=({delta[0]:+.3f},{delta[1]:+.3f}) m "
                    f"max_tilt={stat['max_tilt']:.1f} deg"
                )
            print(f"[hil] BATTERY {'PASS' if all_ok else 'FAIL'}")


if __name__ == "__main__":
    main()
