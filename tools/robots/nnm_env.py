# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""MuJoCo velocity-tracking environment that is already the ArduPilot deployment.

One step() = one AP_NNMixer policy tick. Inside it the physics runs at the
autopilot loop rate (200 Hz), the simulated IMU feeds the same gravity filter
the firmware runs, and the action reaches the joints through the PWM wire
encoding. See deploy_contract.py for the list of matched details.

The robot comes entirely from robots/<id>/robot/profile.json ("sim" block):
MJCF path, trunk body, free joint, IMU sensors, actuator model. Missing IMU
sensors are added to the trunk at load time.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
from common import REPO_ROOT, load_profile, resolve_mjcf  # noqa: E402

SIM_DT = 1.0 / dc.AUTOPILOT_LOOP_HZ


@dataclass
class RewardWeights:
    track_lin_vel: float = 2.0
    track_ang_vel: float = 2.0
    upright: float = 2.0
    pose: float = 1.0
    action_rate: float = -0.8
    alive: float = 0.0
    tracking_sigma: float = 0.25


def _quat_rotate_inverse(q, v):
    w, x, y, z = q
    qv = np.array([x, y, z])
    return v * (2.0 * w * w - 1.0) - np.cross(qv, v) * w * 2.0 + qv * np.dot(qv, v) * 2.0


class NNMixerEnv:
    def __init__(self, robot_id: str, ppo: dict[str, Any] | None = None, mjcf: Path | None = None,
                 seed: int = 0, actuator: str | None = None):
        self.profile = load_profile(robot_id)
        self.ppo = ppo or {}
        self.contract = dc.Contract.from_profile(self.profile, self.ppo)
        sim = self.profile.get("sim", {})
        self.sim = sim
        path = mjcf or resolve_mjcf(robot_id, self.profile)
        if path is None:
            raise FileNotFoundError(
                f"{robot_id}: no MJCF. Run tools/robots/fetch_upstream.py --robot {robot_id} "
                f"or set NNMIXER_MJCF")
        self.rng = np.random.default_rng(seed)
        env_cfg = self.ppo.get("env", {})
        self.cmd_ranges = env_cfg.get("command_ranges", {"vx": [-0.4, 0.4], "vy": [-0.3, 0.3], "wz": [-1.0, 1.0]})
        self.p_zero_cmd = float(env_cfg.get("p_zero_command", 0.2))
        self.p_no_vy = float(env_cfg.get("p_no_lateral", 0.3))
        self.episode_s = float(env_cfg.get("episode_s", 20.0))
        self.gyro_noise = float(env_cfg.get("gyro_noise", 0.02))
        self.accel_noise = float(env_cfg.get("accel_noise", 0.05))
        self.obs_delay_max = int(env_cfg.get("obs_delay_steps_max", 1))
        self.fall_tilt_deg = float(env_cfg.get("fall_tilt_deg", 60.0))
        self.w = RewardWeights(**env_cfg.get("reward", {}))
        self.actuator = actuator or sim.get("actuator", "position")
        self._load(Path(path))
        self.max_steps = int(self.episode_s * self.contract.rate_hz)
        self.reset()

    # ------------------------------------------------------------------ model
    def _load(self, path: Path) -> None:
        names = self.profile["joint_names"]
        trunk = self.sim.get("trunk_body")
        spec = mujoco.MjSpec.from_file(str(path))
        gyro_name = self.sim.get("gyro_sensor", "nnm_gyro")
        acc_name = self.sim.get("accel_sensor", "nnm_accel")
        have = {s.name for s in spec.sensors}
        if gyro_name not in have or acc_name not in have:
            body = spec.body(trunk) if trunk else None
            if body is None:
                raise ValueError("profile sim.trunk_body missing and MJCF has no IMU sensors")
            site = body.add_site(name="nnm_imu")
            if gyro_name not in have:
                spec.add_sensor(name=gyro_name, type=mujoco.mjtSensor.mjSENS_GYRO,
                                objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site.name)
            if acc_name not in have:
                spec.add_sensor(name=acc_name, type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
                                objtype=mujoco.mjtObj.mjOBJ_SITE, objname=site.name)
        self.bam = None
        if self.actuator == "bam_xl330":
            sys.path.insert(0, str(REPO_ROOT / "plant"))
            from mujoco_json_plant import BAM_KP_FW, BAM_VIN, BAM_VIN_MIN, load_with_bam  # noqa: E402
            # load_with_bam compiles from file; write the augmented spec next to it
            tmp = path.parent / f".nnm_{path.stem}_sensors.xml"
            tmp.write_text(spec.to_xml())
            try:
                kp_fw = float(self.sim.get("bam_kp_fw", BAM_KP_FW))
                self.model, self.data, self.bam = load_with_bam(tmp, BAM_VIN, kp_fw)
            finally:
                tmp.unlink(missing_ok=True)
            _ = BAM_VIN_MIN
        else:
            self.model = spec.compile()
            self.model.opt.timestep = SIM_DT
            self.data = mujoco.MjData(self.model)
        m = self.model
        self.model.opt.timestep = SIM_DT
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
        if min(jid) < 0:
            missing = [n for n, j in zip(names, jid) if j < 0]
            raise ValueError(f"joints not in MJCF: {missing}")
        self.qpos_idx = np.array([m.jnt_qposadr[j] for j in jid])
        self.qvel_idx = np.array([m.jnt_dofadr[j] for j in jid])
        # actuator index for each profile joint
        act_for_joint = {}
        for a in range(m.nu):
            act_for_joint[int(m.actuator_trnid[a, 0])] = a
        self.act_idx = np.array([act_for_joint.get(j, -1) for j in jid])
        if (self.act_idx < 0).any() and self.bam is None:
            raise ValueError("every profile joint needs an actuator in the MJCF")
        fj_name = self.sim.get("freejoint")
        if fj_name:
            fj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, fj_name)
        else:
            fj = next(j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
        self.free_qpos = int(m.jnt_qposadr[fj])
        self.free_qvel = int(m.jnt_dofadr[fj])
        self.trunk_id = int(m.jnt_bodyid[fj])
        self.gyro_adr = int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, gyro_name)])
        self.acc_adr = int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, acc_name)])
        self.home_z = float(self.sim.get("home_z", 0.3))
        pd = self.sim.get("pd", {})
        self.kp = float(pd.get("kp", 0.0))
        self.kd = float(pd.get("kd", 0.0))

    # ------------------------------------------------------------------ helpers
    def _sample_command(self) -> np.ndarray:
        if self.rng.random() < self.p_zero_cmd:
            return np.zeros(3, np.float32)
        r = self.cmd_ranges
        t = np.array([self.rng.uniform(*r["vx"]), self.rng.uniform(*r["vy"]), self.rng.uniform(*r["wz"])],
                     np.float32)
        if self.rng.random() < self.p_no_vy:
            t[1] = 0.0   # Rover autopilot modes provide no lateral speed
        return dc.saturate_twist(self.contract, t)

    def _apply_target(self, q_wire: np.ndarray) -> None:
        d = self.data
        if self.bam is not None:
            # actuators outside the policy (e.g. Microban's head) keep their reset target
            self.bam.q_target[self.act_idx] = q_wire
            self.bam.update()
            return
        if self.actuator == "pd":
            q = d.qpos[self.qpos_idx]
            qd = d.qvel[self.qvel_idx]
            d.ctrl[self.act_idx] = self.kp * (q_wire - q) - self.kd * qd
        else:
            d.ctrl[self.act_idx] = q_wire

    def _imu_frd(self) -> tuple[np.ndarray, np.ndarray]:
        d = self.data
        gyro_flu = d.sensordata[self.gyro_adr:self.gyro_adr + 3].copy()
        acc_flu = d.sensordata[self.acc_adr:self.acc_adr + 3].copy()
        gyro_flu += self.rng.normal(0.0, self.gyro_noise, 3)
        acc_flu += self.rng.normal(0.0, self.accel_noise, 3)
        return dc.frd_to_flu(gyro_flu), dc.frd_to_flu(acc_flu)

    def tilt_deg(self) -> float:
        w, x, y, z = self.data.xquat[self.trunk_id]
        return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

    def _base_vel_body(self) -> tuple[np.ndarray, float]:
        d = self.data
        q = d.xquat[self.trunk_id]
        v_world = d.qvel[self.free_qvel:self.free_qvel + 3]
        v_body = _quat_rotate_inverse(q, v_world)
        wz = float(d.qvel[self.free_qvel + 5])   # free joint angular velocity is in the body frame
        return v_body, wz

    # ------------------------------------------------------------------ API
    def reset(self) -> np.ndarray:
        c = self.contract
        d, m = self.data, self.model
        mujoco.mj_resetData(m, d)
        d.qpos[self.free_qpos:self.free_qpos + 3] = [0.0, 0.0, self.home_z]
        d.qpos[self.free_qpos + 3:self.free_qpos + 7] = [1, 0, 0, 0]
        d.qpos[self.qpos_idx] = c.q0
        d.qvel[:] = 0.0
        if self.bam is not None:
            self.bam.q_target[:] = 0.0
            self.bam.q_target[self.act_idx] = c.q0
            self.bam.last_ts = d.time
            d.ctrl[:] = 0.0
        mujoco.mj_forward(m, d)
        self.gravity = dc.GravityFilter(c.att_tau)
        self.a_prev = dc.stand_action(c)
        self.q_wire = dc.apply_action(c, self.a_prev)[1]
        self.command = self._sample_command()
        self.steps = 0
        self._obs_hist: list[np.ndarray] = []
        # disarmed phase: the trunk is held (as the SITL plant pins it before arming) while the
        # gravity filter converges; the policy starts from the upright stand like after "arm"
        # A held board is static: gyro 0, accelerometer = gravity only. Overwriting the root pose is
        # not a physical constraint, so the simulated accelerometer would read spurious accelerations.
        root = d.qpos[self.free_qpos:self.free_qpos + 7].copy()
        for _ in range(dc.AUTOPILOT_LOOP_HZ // 2):
            self._apply_target(self.q_wire)
            mujoco.mj_step(m, d)
            d.qpos[self.free_qpos:self.free_qpos + 7] = root
            d.qvel[self.free_qvel:self.free_qvel + 6] = 0.0
            mujoco.mj_forward(m, d)
            up_flu = -_quat_rotate_inverse(d.xquat[self.trunk_id], np.array([0.0, 0.0, -1.0]))
            self.gravity.update(np.zeros(3), dc.frd_to_flu(up_flu * dc.GRAVITY_MSS), SIM_DT)
        return self._observe()

    def _observe(self) -> np.ndarray:
        c = self.contract
        d = self.data
        gyro_frd, _ = self._imu_frd()
        obs = dc.build_obs(c, dc.frd_to_flu(gyro_frd), self.gravity.gravity_flu(),
                           d.qpos[self.qpos_idx], d.qvel[self.qvel_idx], self.a_prev, self.command)
        self._obs_hist.append(obs)
        self._obs_hist = self._obs_hist[-(self.obs_delay_max + 1):]
        delay = int(self.rng.integers(0, self.obs_delay_max + 1)) if self.obs_delay_max else 0
        return self._obs_hist[max(0, len(self._obs_hist) - 1 - delay)].copy()

    def step(self, action: np.ndarray):
        c = self.contract
        self.a_prev, self.q_wire = dc.apply_action(c, action)
        m, d = self.model, self.data
        for _ in range(c.loop_steps_per_policy):
            self._apply_target(self.q_wire)
            mujoco.mj_step(m, d)
            g, a = self._imu_frd()
            self.gravity.update(g, a, SIM_DT)
        self.steps += 1
        reward, terms = self._reward(action)
        fell = self.tilt_deg() > self.fall_tilt_deg
        timeout = self.steps >= self.max_steps
        obs = self._observe()
        return obs, reward, fell, timeout, terms

    def _reward(self, action) -> tuple[float, dict]:
        w = self.w
        v_body, wz = self._base_vel_body()
        cmd = self.command
        lin_err = float(np.sum((cmd[:2] - v_body[:2]) ** 2))
        ang_err = float((cmd[2] - wz) ** 2)
        up = -self.gravity.gravity_flu()[2]            # 1 when upright
        pose_err = float(np.sum((self.data.qpos[self.qpos_idx] - self.contract.q0) ** 2))
        if not hasattr(self, "_last_raw_action"):
            self._last_raw_action = np.zeros_like(action)
        rate = float(np.sum((np.asarray(action) - self._last_raw_action) ** 2))
        self._last_raw_action = np.asarray(action).copy()
        terms = {
            "track_lin_vel": w.track_lin_vel * math.exp(-lin_err / w.tracking_sigma),
            "track_ang_vel": w.track_ang_vel * math.exp(-ang_err / w.tracking_sigma),
            "upright": w.upright * up,
            "pose": w.pose * math.exp(-pose_err),
            "action_rate": w.action_rate * rate,
            "alive": w.alive,
        }
        dt = 1.0 / self.contract.rate_hz
        return sum(terms.values()) * dt, terms


def load_ppo_config(robot_id: str) -> dict[str, Any]:
    import yaml

    from common import robot_dir
    p = robot_dir(robot_id) / "robot" / "ppo.yaml"
    if not p.is_file():
        raise FileNotFoundError(p)
    return yaml.safe_load(p.read_text())


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Step the ArduPilot-contract env at zero action")
    ap.add_argument("--robot", required=True)
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()
    env = NNMixerEnv(args.robot, load_ppo_config(args.robot))
    obs = env.reset()
    n = int(args.seconds * env.contract.rate_hz)
    for _ in range(n):
        obs, r, fell, timeout, _ = env.step(np.zeros(env.contract.n_joints, np.float32))
        if fell:
            break
    print(f"robot={args.robot} obs_dim={obs.size} steps={env.steps} tilt={env.tilt_deg():.1f} deg "
          f"gravity_flu={np.round(env.gravity.gravity_flu(), 3).tolist()}")
