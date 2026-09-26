# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Python mirror of what AP_NNMixer computes on the autopilot.

A training environment that builds its observation and applies its action
through these functions sees exactly the numbers the firmware will produce:

- observation layout from the robot profile (no padding to 61)
- gyro raw in trunk FLU (INS_GYRO_FILTER 0 on the autopilot)
- gravity from the IMU-only complementary filter of AP_NNMixer::update_attitude(),
  run at the autopilot loop rate, not the simulator's ground-truth quaternion
- action clipped to NNM_ACT_MAX, stored as previous action after the clip
- joint target sent as PWM with 1 us = 3 mrad around 1500, clamped to 800..2200
- policy at 50 Hz / rate_div on a 200 Hz loop

Everything here must stay in sync with ardupilot/libraries/AP_NNMixer/AP_NNMixer.cpp.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

AUTOPILOT_LOOP_HZ = 200          # SCHED_LOOP_RATE in sitl/nnmixer.parm
POLICY_TASK_HZ = 50              # AP_NNMixer::update() scheduler rate
PWM_CENTER = 1500
RAD_PER_US = 0.003
PWM_MIN = 800
PWM_MAX = 2200
GRAVITY_MSS = 9.80665

# Firmware parameter defaults (AP_NNMixer::var_info)
DEFAULT_ACT_MAX = 2.0            # NNM_ACT_MAX
DEFAULT_ATT_TAU = 0.5            # NNM_ATT_TAU
DEFAULT_VX_MAX = 0.4             # NNM_VX_MAX
DEFAULT_VY_MAX = 0.3             # NNM_VY_MAX
DEFAULT_WZ_MAX = 1.0             # NNM_WZ_MAX


@dataclass
class Contract:
    n_joints: int
    q0: np.ndarray
    rate_hz: int = POLICY_TASK_HZ
    extra_cmd_dim: int = 0          # head/body commands after the twist; 0 on the autopilot
    act_max: float = DEFAULT_ACT_MAX
    att_tau: float = DEFAULT_ATT_TAU
    vx_max: float = DEFAULT_VX_MAX
    vy_max: float = DEFAULT_VY_MAX
    wz_max: float = DEFAULT_WZ_MAX
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def obs_dim(self) -> int:
        return 3 + 3 + 3 * self.n_joints + 3 + self.extra_cmd_dim

    @property
    def twist_offset(self) -> int:
        return 6 + 3 * self.n_joints

    @property
    def rate_div(self) -> int:
        if POLICY_TASK_HZ % self.rate_hz != 0:
            raise ValueError(f"rate_hz {self.rate_hz} must divide {POLICY_TASK_HZ} (50, 25, 10, 5)")
        return POLICY_TASK_HZ // self.rate_hz

    @property
    def loop_steps_per_policy(self) -> int:
        """Autopilot loop ticks (200 Hz) between two policy ticks."""
        return (AUTOPILOT_LOOP_HZ // POLICY_TASK_HZ) * self.rate_div

    @classmethod
    def from_profile(cls, profile: dict[str, Any], ppo: dict[str, Any] | None = None) -> "Contract":
        n = int(profile["n_joints"])
        if n <= 0:
            raise ValueError(f"{profile.get('robot_id')}: n_joints not set in profile")
        q0 = np.asarray(profile.get("q0") or [0.0] * n, dtype=np.float32)
        obs_dim = int(profile.get("obs_dim") or 0)
        base = 3 + 3 + 3 * n + 3
        extra = max(obs_dim - base, 0) if obs_dim else 0
        env = (ppo or {}).get("env", {})
        cmd = env.get("command_ranges", {})
        return cls(
            n_joints=n,
            q0=q0,
            rate_hz=int(profile.get("rate_hz") or POLICY_TASK_HZ),
            extra_cmd_dim=extra,
            act_max=float(env.get("act_max", DEFAULT_ACT_MAX)),
            att_tau=float(env.get("att_tau", DEFAULT_ATT_TAU)),
            vx_max=float(max(abs(v) for v in cmd.get("vx", [DEFAULT_VX_MAX]))),
            vy_max=float(max(abs(v) for v in cmd.get("vy", [DEFAULT_VY_MAX]))),
            wz_max=float(max(abs(v) for v in cmd.get("wz", [DEFAULT_WZ_MAX]))),
        )


# ---------------------------------------------------------------------------
# frames

def frd_to_flu(v: np.ndarray) -> np.ndarray:
    """ArduPilot body FRD <-> trunk FLU: (x, y, z) -> (x, -y, -z). Its own inverse."""
    out = np.array(v, dtype=np.float64, copy=True)
    out[..., 1] *= -1.0
    out[..., 2] *= -1.0
    return out


# ---------------------------------------------------------------------------
# gravity: AP_NNMixer::update_attitude() + gravity_body_flu()

class GravityFilter:
    """IMU-only down-vector estimate in body FRD, updated every autopilot loop tick.

    gyro_frd rad/s and accel_frd m/s^2 are what AP_InertialSensor reports
    (specific force: level and static -> accel = (0, 0, -g)).
    """

    def __init__(self, tau: float = DEFAULT_ATT_TAU):
        self.tau = tau
        self.down = np.array([0.0, 0.0, 1.0])
        self.valid = False

    def reset(self) -> None:
        self.down = np.array([0.0, 0.0, 1.0])
        self.valid = False

    def update(self, gyro_frd: np.ndarray, accel_frd: np.ndarray, dt: float) -> None:
        gyro = np.asarray(gyro_frd, dtype=np.float64)
        accel = np.asarray(accel_frd, dtype=np.float64)
        an = float(np.linalg.norm(accel))
        if not self.valid:
            if an > 1.0:
                self.down = -accel / an
                self.valid = True
            return
        d = self.down - np.cross(gyro, self.down) * dt
        if 0.5 * GRAVITY_MSS < an < 1.5 * GRAVITY_MSS and self.tau > 0.0:
            alpha = min(max(dt / self.tau, 0.0), 1.0)
            d = d * (1.0 - alpha) + (-accel / an) * alpha
        n = float(np.linalg.norm(d))
        if n > 1e-6:
            self.down = d / n

    def gravity_flu(self) -> np.ndarray:
        return frd_to_flu(self.down).astype(np.float32)


# ---------------------------------------------------------------------------
# observation: AP_NNMixer::update() observation block

def build_obs(c: Contract, gyro_flu, gravity_flu, q, qd, a_prev, twist, extra_cmd=None) -> np.ndarray:
    n = c.n_joints
    o = np.zeros(c.obs_dim, dtype=np.float32)
    o[0:3] = gyro_flu
    o[3:6] = gravity_flu
    o[6:6 + n] = np.asarray(q, dtype=np.float32) - c.q0
    o[6 + n:6 + 2 * n] = qd
    o[6 + 2 * n:6 + 3 * n] = a_prev
    t = c.twist_offset
    o[t:t + 3] = twist
    if c.extra_cmd_dim and extra_cmd is not None:
        o[t + 3:t + 3 + c.extra_cmd_dim] = extra_cmd
    return o


def saturate_twist(c: Contract, twist) -> np.ndarray:
    """Autopilot modes saturate the Rover desired speed / turn rate (vy = 0 outside MANUAL)."""
    t = np.asarray(twist, dtype=np.float32).copy()
    t[0] = np.clip(t[0], -c.vx_max, c.vx_max)
    t[1] = np.clip(t[1], -c.vy_max, c.vy_max)
    t[2] = np.clip(t[2], -c.wz_max, c.wz_max)
    return t


# ---------------------------------------------------------------------------
# action: clip, previous action, PWM wire encoding

def apply_action(c: Contract, action) -> tuple[np.ndarray, np.ndarray]:
    """Return (a_prev_for_next_obs, q_target_as_received_by_servo)."""
    a = np.clip(np.asarray(action, dtype=np.float32), -c.act_max, c.act_max)
    q_target = c.q0 + a
    pwm = PWM_CENTER + np.round(q_target / RAD_PER_US)
    pwm = np.clip(pwm, PWM_MIN, PWM_MAX)
    q_wire = ((pwm - PWM_CENTER) * RAD_PER_US).astype(np.float32)
    return a, q_wire


def stand_action(c: Contract) -> np.ndarray:
    """Disarmed / failsafe: action zero, previous action zero."""
    return np.zeros(c.n_joints, dtype=np.float32)


# ---------------------------------------------------------------------------
# int8 forward identical to nnmixer_forward_int8() in nnmixer_infer.c

def int8_forward(layers, mean, std, obs):
    """layers: list of (W_int8[out,in], w_scale[out], bias[out], elu: bool)."""
    x = (np.asarray(obs, dtype=np.float32) - mean) / std
    for W, s, b, elu in layers:
        y = (W.astype(np.float32) * s[:, None]) @ x + b
        x = np.where(y > 0, y, np.expm1(y)).astype(np.float32) if elu else y.astype(np.float32)
    return x
