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
    # exp(-err²/sigma): with 0.25 a robot commanded 0.3 m/s kept 70% of the reward by standing still
    tracking_sigma: float = 0.05
    tracking_sigma_ang: float = 0.25
    # time constant of a low-pass on the base twist used by the tracking terms (0 = instantaneous). A small
    # trotting robot swings its speed within every step: the instantaneous error punished a gait at the right
    # mean speed more than standing still.
    tracking_filter_s: float = 0.0
    # gait terms of the MicroDuck mjlab task; active only when the profile lists sim.feet
    air_time: float = 0.0
    air_time_min_s: float = 0.1
    air_time_max_s: float = 0.5
    air_time_mode: str = "touchdown"        # "in_range": mjlab feet_air_time, per step while airborne in range
    foot_clearance: float = 0.0
    swing_height_m: float = 0.03
    foot_swing_height: float = 0.0          # mjlab: (peak / target - 1)^2 at landing
    foot_slip: float = 0.0
    # phase-free gait term: one foot on the ground, the other in the air, while commanded to move.
    # (the phase-based feet_gait needs a gait phase in the observation, which AP_NNMixer does not send)
    single_stance: float = 0.0
    # positive reward for lifting the swing foot, linear in height up to swing_height_m, while moving
    foot_lift: float = 0.0
    # per touchdown after a real swing (>= alternation_min_air_s): +w if it is the other foot than the
    # previous touchdown, -w if the same foot steps again (a policy stepped with one foot only)
    foot_alternation: float = 0.0
    alternation_min_air_s: float = 0.05
    alternation_min_height_frac: float = 0.0     # step must reach this fraction of swing_height_m
    # per step, penalty on the height / air-time difference with the previous step of the other foot
    foot_symmetry: float = 0.0
    # quadruped trot, phase-free: each diagonal pair (sim.trot_pairs) shares its contact state and the two
    # pairs are in opposite states, while commanded to move
    trot: float = 0.0
    # sum of (actuator force / force limit)^2: small hobby servos run close to stall
    joint_torque: float = 0.0
    # quadruped four-beat walk, phase-free. footfall_sequence: per touchdown after a real swing, +w if the
    # foot follows the previous touchdown in sim.footfall_cycle (reversed when commanded backward), -w/2 if
    # the same foot steps again. three_stance: exactly one foot airborne (for less than air_time_max_s)
    # while moving. all_stance: all four feet down while moving (negative weight).
    footfall_sequence: float = 0.0
    three_stance: float = 0.0
    all_stance: float = 0.0
    # fewer than under_stance_feet feet on the ground (hops, flight phases), commanded or not (negative weight)
    under_stance: float = 0.0
    under_stance_feet: int = 2
    # per floor contact of a robot geom that is not a foot geom (shank, thigh, trunk lying on the floor)
    undesired_contacts: float = 0.0
    # sum of joint velocities squared (legged_gym dof_vel): slower, smoother leg motion
    joint_vel: float = 0.0
    # one-off reward when the episode ends by a fall (negative): falling never pays to end a costly episode
    termination: float = 0.0
    # standing still while commanded to move (negative weight): 1 - progress, with progress the share of the
    # commanded velocity actually achieved (filtered twist projected on the command, clipped to [0, 1]),
    # averaged over the commanded linear and yaw parts
    no_progress: float = 0.0
    no_progress_min_cmd: float = 0.03
    # exp(-((trunk height - target) / std)^2): keeps the stand height instead of crouching
    base_height: float = 0.0
    base_height_target_m: float = 0.1
    base_height_std_m: float = 0.01
    # MicroDuck (mjlab velocity task) shapes, off by default
    upright_std: float | None = None        # exp(-|g_xy|^2 / std^2) on the true trunk orientation
    pose_std_standing: dict | None = None   # variable_posture: per-joint std (regex -> std)
    pose_std_walking: dict | None = None
    walking_threshold: float = 0.01
    self_collisions: float = 0.0
    dof_pos_limits: float = 0.0
    soft_limit_factor: float = 0.9
    body_ang_vel: float = 0.0
    angular_momentum: float = 0.0


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
        self.p_single_axis = float(env_cfg.get("p_single_axis", 0.0))
        self.episode_s = float(env_cfg.get("episode_s", 20.0))
        self.gyro_noise = float(env_cfg.get("gyro_noise", 0.02))
        self.accel_noise = float(env_cfg.get("accel_noise", 0.05))
        self.obs_delay_max = int(env_cfg.get("obs_delay_steps_max", 1))
        self.fall_tilt_deg = float(env_cfg.get("fall_tilt_deg", 60.0))
        rs = env_cfg.get("resample_s")          # MicroDuck: new command every 3-8 s inside the episode
        self.resample_s = tuple(rs) if rs else None
        push = env_cfg.get("push") or {}        # random trunk velocity kicks (mjlab push_robot)
        self.push_interval = tuple(push["interval_s"]) if push else None
        self.push_vel = float(push.get("vel_xy", 0.0)) if push else 0.0
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
            import os
            tmp = path.parent / f".nnm_{path.stem}_sensors_{os.getpid()}_{id(self)}.xml"
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
        # feet: site for position, body for contacts with the floor
        self.feet = []
        floor = {g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE}
        self.floor_geoms = floor
        for f in self.sim.get("feet", []):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f["site"])
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f["body"])
            if sid < 0 or bid < 0:
                raise ValueError(f"foot {f} not in MJCF")
            self.feet.append((sid, bid))
        self.foot_body_ids = {bid for _, bid in self.feet}
        # optional foot geom: only that geom counts as the foot touching the floor (a shank lying on the
        # floor is not a foot contact)
        self.foot_geom_index = {}
        for i, f in enumerate(self.sim.get("feet", [])):
            if f.get("geom"):
                gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f["geom"])
                if gid < 0:
                    raise ValueError(f"foot geom {f['geom']} not in MJCF")
                self.foot_geom_index[gid] = i
        self.trot_pairs = [tuple(p) for p in self.sim.get("trot_pairs", [])]
        cycle = list(self.sim.get("footfall_cycle", []))
        self.footfall_next = {a: cycle[(k + 1) % len(cycle)] for k, a in enumerate(cycle)}
        self.footfall_prev = {a: cycle[k - 1] for k, a in enumerate(cycle)}
        lim = m.actuator_forcerange[np.maximum(self.act_idx, 0), 1]
        self.force_limit = np.where(lim > 0, lim, 1.0)
        # joint soft limits (mjlab joint_pos_limits) and per-joint posture std
        rng_lo, rng_hi = [], []
        for j in jid:
            lo, hi = m.jnt_range[j] if m.jnt_limited[j] else (-np.inf, np.inf)
            mid, half = (lo + hi) / 2, (hi - lo) / 2 * self.w.soft_limit_factor
            rng_lo.append(mid - half); rng_hi.append(mid + half)
        self.soft_lo, self.soft_hi = np.array(rng_lo), np.array(rng_hi)

        def stds(table):
            import re
            if not table:
                return None
            out = []
            for n in names:
                hit = [v for pat, v in table.items() if re.fullmatch(pat, n)]
                out.append(hit[0] if hit else 1.0)
            return np.array(out)
        self.pose_std_stand = stds(self.w.pose_std_standing)
        self.pose_std_walk = stds(self.w.pose_std_walking)
        # geoms of the robot (every body under the free-joint body) for self-collision counting
        robot_bodies = {b for b in range(m.nbody) if self._is_under(b, self.trunk_id)}
        self.robot_geom = np.array([m.geom_bodyid[g] in robot_bodies for g in range(m.ngeom)])
        pd = self.sim.get("pd", {})
        self.kp = float(pd.get("kp", 0.0))
        self.kd = float(pd.get("kd", 0.0))

    # ------------------------------------------------------------------ helpers
    def _is_under(self, body: int, root: int) -> bool:
        m = self.model
        while body > 0:
            if body == root:
                return True
            body = int(m.body_parentid[body])
        return body == root

    def _next_resample(self) -> int:
        if not self.resample_s:
            return 10 ** 9
        return self.steps + int(self.rng.uniform(*self.resample_s) * self.contract.rate_hz)

    def _sample_command(self) -> np.ndarray:
        if self.rng.random() < self.p_zero_cmd:
            return np.zeros(3, np.float32)
        r = self.cmd_ranges
        if self.rng.random() < self.p_single_axis:
            # one axis, one sign, between half and full range: every direction gets clear commands
            axis = int(self.rng.integers(3))
            lo, hi = r[("vx", "vy", "wz")[axis]]
            limit = hi if self.rng.random() < 0.5 else lo
            t = np.zeros(3, np.float32)
            t[axis] = limit * self.rng.uniform(0.5, 1.0)
            return dc.saturate_twist(self.contract, t)
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
        self._ep_terms: dict[str, float] = {}
        self._last_raw_action = np.zeros(c.n_joints, np.float32)
        self.twist_filt = np.zeros(4)
        nf = len(self.feet)
        self.foot_air = np.zeros(nf)
        self.foot_contact_prev = np.ones(nf, bool)
        self.foot_pos_prev = np.array([d.site_xpos[s].copy() for s, _ in self.feet]).reshape(nf, 3)
        self.foot_z0 = self.foot_pos_prev[:, 2].copy() if nf else np.zeros(0)
        self.foot_peak = np.zeros(nf)
        self.foot_air_prev = np.zeros(nf)
        self.foot_air_prev_step = np.zeros(nf)
        self.last_touchdown = -1
        self.last_footfall = -1
        self.foot_air_prev_seq = np.zeros(nf)
        self.last_step_peak = 0.0
        self.last_step_air = 0.0
        self.next_resample = self._next_resample()
        self.next_push = (int(self.rng.uniform(*self.push_interval) * c.rate_hz) if self.push_interval
                          else 10 ** 9)
        self._obs_hist: list[np.ndarray] = []
        # disarmed phase: the trunk is held (as the SITL plant pins it before arming) while the
        # gravity filter converges; the policy starts from the upright stand like after "arm"
        # A held board is static: gyro 0, accelerometer = gravity only. Overwriting the root pose is
        # not a physical constraint, so the simulated accelerometer would read spurious accelerations.
        root = d.qpos[self.free_qpos:self.free_qpos + 7].copy()
        for _ in range(dc.AUTOPILOT_LOOP_HZ // 10):
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
        if self.steps >= self.next_resample:
            self.command = self._sample_command()
            self.next_resample = self._next_resample()
        if self.steps >= self.next_push:
            d.qvel[self.free_qvel:self.free_qvel + 2] += self.rng.uniform(-self.push_vel, self.push_vel, 2)
            self.next_push = self.steps + int(self.rng.uniform(*self.push_interval) * c.rate_hz)
        reward, info = self._reward(action)
        fell = self.tilt_deg() > self.fall_tilt_deg
        if fell and self.w.termination:
            info["terms"]["termination"] = self.w.termination
            reward += self.w.termination
        timeout = self.steps >= self.max_steps
        for k, v in info["terms"].items():
            self._ep_terms[k] = self._ep_terms.get(k, 0.0) + v
        if fell or timeout:
            # rsl_rl convention: Episode_Reward/<term> = episode sum / max episode length (s)
            info["episode"] = {k: v / self.episode_s for k, v in self._ep_terms.items()}
            info["reason"] = "fell_over" if fell else "time_out"
        obs = self._observe()
        return obs, reward, fell, timeout, info

    def _feet_contact(self) -> np.ndarray:
        d, m = self.data, self.model
        touch = np.zeros(len(self.feet), bool)
        index = {bid: i for i, (_, bid) in enumerate(self.feet)}
        for k in range(d.ncon):
            con = d.contact[k]
            g1, g2 = con.geom1, con.geom2
            if g1 in self.floor_geoms:
                g = g2
            elif g2 in self.floor_geoms:
                g = g1
            else:
                continue
            if self.foot_geom_index:
                if g in self.foot_geom_index:
                    touch[self.foot_geom_index[g]] = True
            elif m.geom_bodyid[g] in index:
                touch[index[m.geom_bodyid[g]]] = True
        return touch

    def _undesired_contacts(self) -> int:
        """Robot geoms other than the feet touching the floor (legged_gym collision penalty)."""
        d = self.data
        n = 0
        for k in range(d.ncon):
            con = d.contact[k]
            g1, g2 = con.geom1, con.geom2
            g = g2 if g1 in self.floor_geoms else (g1 if g2 in self.floor_geoms else -1)
            if g >= 0 and self.robot_geom[g] and g not in self.foot_geom_index:
                n += 1
        return n

    def _gait_terms(self, dt: float) -> dict:
        """feet_air_time, foot_clearance and foot_slip of the MicroDuck mjlab velocity task."""
        w = self.w
        d = self.data
        contact = self._feet_contact()
        pos = np.array([d.site_xpos[s] for s, _ in self.feet])
        vel_xy = np.linalg.norm((pos - self.foot_pos_prev)[:, :2], axis=1) / dt
        self.foot_pos_prev = pos.copy()
        cmd_norm = float(np.linalg.norm(self.command[:2]) + abs(self.command[2]))
        moving = float(cmd_norm > (w.walking_threshold if w.air_time_mode == "in_range" else 0.05))
        first = contact & ~self.foot_contact_prev
        self.foot_air = np.where(contact, 0.0, self.foot_air + dt)
        if w.air_time_mode == "in_range":
            # mjlab feet_air_time(threshold_min, threshold_max): feet airborne for a time in range, every step
            air = float(np.sum((self.foot_air > w.air_time_min_s) & (self.foot_air < w.air_time_max_s)))
        else:
            air = 0.0
            for i in np.flatnonzero(first):
                air += min(self.foot_air_prev[i] if hasattr(self, "foot_air_prev") else 0.0,
                           w.air_time_max_s) - w.air_time_min_s
        self.foot_air_prev = self.foot_air.copy()
        self.foot_contact_prev = contact
        # height above the foot's last stance position (flat floor: the mjlab height scan)
        self.foot_z0 = np.where(contact, pos[:, 2], self.foot_z0)
        height = pos[:, 2] - self.foot_z0
        # mjlab feet_clearance: |h - target| weighted by foot speed, only with a command
        clearance = float(np.sum(np.abs(height - w.swing_height_m) * vel_xy * ~contact)) * moving
        # mjlab feet_swing_height: peak height error at landing
        swing = float(np.sum(((self.foot_peak / w.swing_height_m - 1.0) ** 2) * first)) * moving
        peak_at_touch = self.foot_peak.copy()
        self.foot_peak = np.where(contact, 0.0, np.maximum(self.foot_peak, height))
        slip = float(np.sum((vel_xy ** 2) * contact)) * moving
        out = {"air_time": w.air_time * air * moving * dt,
               "foot_clearance": w.foot_clearance * clearance * dt,
               "foot_slip": w.foot_slip * slip * dt}
        if w.single_stance:
            # one foot down, the other airborne for less than air_time_max_s (a step, not standing on one leg)
            step_now = contact.sum() == len(contact) - 1 and bool(np.all(self.foot_air[~contact] < w.air_time_max_s))
            out["single_stance"] = w.single_stance * float(step_now) * moving * dt
        if w.foot_alternation or w.foot_symmetry:
            alt = sym = 0.0
            for i in np.flatnonzero(first):
                air_i, peak_i = float(self.foot_air_prev_step[i]), float(peak_at_touch[i])
                # a step counts only with a real swing: long enough and high enough (a policy satisfied
                # alternation with 1 cm shuffles of one foot while the other took 5 cm steps)
                if air_i < w.alternation_min_air_s or peak_i < w.alternation_min_height_frac * w.swing_height_m:
                    continue
                if self.last_touchdown >= 0:
                    alt += 1.0 if i != self.last_touchdown else -1.0
                    if i != self.last_touchdown:
                        # left/right symmetry: this step against the previous step of the other foot
                        sym += ((peak_i - self.last_step_peak) / w.swing_height_m) ** 2
                        sym += ((air_i - self.last_step_air) / w.air_time_max_s) ** 2
                self.last_touchdown = int(i)
                self.last_step_peak, self.last_step_air = peak_i, air_i
            if w.foot_alternation:
                out["foot_alternation"] = w.foot_alternation * alt * moving
            if w.foot_symmetry:
                out["foot_symmetry"] = w.foot_symmetry * sym * moving
        self.foot_air_prev_step = self.foot_air.copy()
        if w.foot_lift:
            # only during a real step: this foot airborne for less than air_time_max_s while the other
            # stands. Without the bound a policy kept one foot up for good and leaned on the other leg.
            stepping = (~contact) & (self.foot_air < w.air_time_max_s) & (contact.sum() == len(contact) - 1)
            lift = np.clip(height / w.swing_height_m, 0.0, 1.0) * stepping
            out["foot_lift"] = w.foot_lift * float(np.sum(lift)) * moving * dt
        if w.foot_swing_height:
            out["foot_swing_height"] = w.foot_swing_height * swing * dt
        n_feet = len(contact)
        if w.three_stance and n_feet == 4:
            one_up = contact.sum() == 3 and bool(np.all(self.foot_air[~contact] < w.air_time_max_s))
            out["three_stance"] = w.three_stance * float(one_up) * moving * dt
        if w.under_stance:
            out["under_stance"] = w.under_stance * float(contact.sum() < w.under_stance_feet) * dt
        if w.all_stance and n_feet == 4:
            out["all_stance"] = w.all_stance * float(contact.all()) * moving * dt
        if w.footfall_sequence and self.footfall_next:
            seq = 0.0
            backward = self.command[0] < 0
            for i in np.flatnonzero(first):
                if (self.foot_air_prev_seq[i] < w.alternation_min_air_s
                        or peak_at_touch[i] < w.alternation_min_height_frac * w.swing_height_m):
                    continue
                last = self.last_footfall
                if last >= 0:
                    expected = self.footfall_prev[last] if backward else self.footfall_next[last]
                    seq += 1.0 if i == expected else (-0.5 if i == last else 0.0)
                self.last_footfall = int(i)
            out["footfall_sequence"] = w.footfall_sequence * seq * moving
        self.foot_air_prev_seq = self.foot_air.copy()
        if w.trot and self.trot_pairs:
            (a, b), (c, e) = self.trot_pairs
            trot_now = contact[a] == contact[b] and contact[c] == contact[e] and contact[a] != contact[c]
            out["trot"] = w.trot * float(trot_now) * moving * dt
        return out

    def set_curriculum(self, action_rate: float | None = None, standing_envs: float | None = None) -> None:
        """MicroDuck curriculum knobs: action_rate_l2 weight and share of zero-command (standing) episodes."""
        if action_rate is not None:
            self.w.action_rate = float(action_rate)
        if standing_envs is not None:
            self.p_zero_cmd = float(standing_envs)

    def _reward(self, action) -> tuple[float, dict]:
        w = self.w
        d, m = self.data, self.model
        v_body, wz = self._base_vel_body()
        cmd = self.command
        w_body = d.qvel[self.free_qvel + 3:self.free_qvel + 6]       # free-joint angular velocity, body frame
        # mjlab track_linear_velocity / track_angular_velocity
        if w.tracking_filter_s:
            alpha = min(1.0, (1.0 / self.contract.rate_hz) / w.tracking_filter_s)
            self.twist_filt += alpha * (np.array([*v_body, wz]) - self.twist_filt)
            v_track, wz_track = self.twist_filt[:3], float(self.twist_filt[3])
        else:
            v_track, wz_track = v_body, wz
        lin_err = float(np.sum((cmd[:2] - v_track[:2]) ** 2)) + 2.0 * float(v_track[2] ** 2)
        ang_err = float((cmd[2] - wz_track) ** 2) + 0.05 * float(np.sum(w_body[:2] ** 2))
        q = d.qpos[self.qpos_idx]
        if w.upright_std:
            g_true = _quat_rotate_inverse(d.xquat[self.trunk_id], np.array([0.0, 0.0, -1.0]))
            up = math.exp(-float(np.sum(g_true[:2] ** 2)) / w.upright_std ** 2)
        else:
            up = -self.gravity.gravity_flu()[2]            # 1 when upright
        moving = float(np.linalg.norm(cmd[:2]) + abs(cmd[2]))
        if self.pose_std_stand is not None:
            std = self.pose_std_stand if moving < w.walking_threshold else self.pose_std_walk
            pose = math.exp(-float(np.mean(((q - self.contract.q0) / std) ** 2)))
        else:
            pose = math.exp(-float(np.sum((q - self.contract.q0) ** 2)))
        rate = float(np.sum((np.asarray(action) - self._last_raw_action) ** 2))
        self._last_raw_action = np.asarray(action, dtype=np.float32).copy()
        dt = 1.0 / self.contract.rate_hz
        terms = {
            "track_linear_velocity": w.track_lin_vel * math.exp(-lin_err / w.tracking_sigma) * dt,
            "track_angular_velocity": w.track_ang_vel * math.exp(-ang_err / w.tracking_sigma_ang) * dt,
            "upright": w.upright * up * dt,
            "pose": w.pose * pose * dt,
            "action_rate_l2": w.action_rate * rate * dt,
        }
        if w.alive:
            terms["alive"] = w.alive * dt
        if w.body_ang_vel:
            terms["body_ang_vel"] = w.body_ang_vel * float(np.sum(w_body[:2] ** 2)) * dt
        if w.angular_momentum:
            mujoco.mj_subtreeVel(m, d)
            terms["angular_momentum"] = w.angular_momentum * float(np.sum(d.subtree_angmom[self.trunk_id] ** 2)) * dt
        if w.no_progress:
            lack = []
            c_xy = float(np.linalg.norm(cmd[:2]))
            if c_xy > w.no_progress_min_cmd:
                lack.append(1.0 - float(np.clip(np.dot(v_track[:2], cmd[:2]) / c_xy ** 2, 0.0, 1.0)))
            if abs(cmd[2]) > 3 * w.no_progress_min_cmd:
                lack.append(1.0 - float(np.clip(wz_track / cmd[2], 0.0, 1.0)))
            if lack:
                terms["no_progress"] = w.no_progress * float(np.mean(lack)) * dt
        if w.joint_vel:
            terms["joint_vel"] = w.joint_vel * float(np.sum(d.qvel[self.qvel_idx] ** 2)) * dt
        if w.undesired_contacts:
            terms["undesired_contacts"] = w.undesired_contacts * self._undesired_contacts() * dt
        if w.base_height:
            dz = float(d.qpos[self.free_qpos + 2]) - w.base_height_target_m
            terms["base_height"] = w.base_height * math.exp(-(dz / w.base_height_std_m) ** 2) * dt
        if w.joint_torque:
            tau = d.actuator_force[self.act_idx] / self.force_limit
            terms["joint_torque"] = w.joint_torque * float(np.sum(tau ** 2)) * dt
        if w.dof_pos_limits:
            out = np.clip(self.soft_lo - q, 0, None) + np.clip(q - self.soft_hi, 0, None)
            terms["dof_pos_limits"] = w.dof_pos_limits * float(np.sum(out)) * dt
        if w.self_collisions:
            n_self = 0
            for k in range(d.ncon):
                con = d.contact[k]
                if self.robot_geom[con.geom1] and self.robot_geom[con.geom2]:
                    n_self += 1
            terms["self_collisions"] = w.self_collisions * n_self * dt
        if self.feet:
            terms.update(self._gait_terms(dt))
        info = {"terms": terms, "error_vel_xy": math.sqrt(lin_err), "error_vel_yaw": math.sqrt(ang_err),
                "vx": float(v_body[0]), "upright": float(up)}
        return sum(terms.values()), info


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
