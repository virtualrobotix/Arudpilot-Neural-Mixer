#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""PPO velocity training for a catalog robot, int8 from the first iteration.

Environment: tools/robots/nnm_env.py, which is the ArduPilot deployment
contract (autopilot-loop physics, firmware gravity filter, PWM encoding,
policy rate, observation layout from the robot profile).
Network: robots/<id>/robot/ppo.yaml, actor trained on the int8 weight grid.
Output: robots/<id>/policies/<name>.nnm, the file copied to /APM/nnm/<id>/policies/.

Logging, curriculum and evaluation follow the MicroDuck training
(mjlab + rsl_rl, W&B project mjlab_microduck): the same metric names
(Train/*, Episode_Reward/*, Episode_Termination/*, Metrics/twist/*, Loss/*,
Policy/*, Perf/*, Curriculum/*), the action_rate and standing_envs curriculum,
and the stand (vx 0) / walk (vx 0.3) evaluation of run_microduck_cartan_bench.py.

Usage:
    python tools/robots/train_velocity.py --robot microduck --envs 512 --workers 16 --iters 1000 --wandb
    python tools/robots/train_velocity.py --robot microban --init-onnx robots/microban/policies/walk.onnx ...
    python tools/robots/train_velocity.py --robot microduck --eval robots/microduck/policies/walk.nnm
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
from common import robot_dir  # noqa: E402
from export_nnm import unpack_nnm  # noqa: E402
from nnm_env import NNMixerEnv, load_ppo_config  # noqa: E402

# MicroDuck mjlab curriculum (Curriculum/* in W&B run mjlab_microduck/vfaof1ds)
DEFAULT_CURRICULUM = {"action_rate": [-0.1, -1.0], "standing_envs": [0.02, 0.2], "ramp_fraction": 0.5}
EVAL_MODES = {"stand": (0.0, 0.0, 0.0), "walk_0.3": (0.3, 0.0, 0.0),
              "lateral_0.2": (0.0, 0.2, 0.0), "turn_0.8": (0.0, 0.0, 0.8)}
REWARD_TERMS = ["track_linear_velocity", "track_angular_velocity", "upright", "pose", "action_rate_l2",
                "alive", "air_time", "single_stance", "foot_lift", "foot_alternation", "foot_symmetry",
                "foot_clearance",
                "foot_swing_height",
                "foot_slip", "trot", "joint_torque",
                "self_collisions",
                "dof_pos_limits", "body_ang_vel", "angular_momentum"]


def eval_modes(cfg: dict) -> dict:
    """Evaluation commands: env.eval_modes of ppo.yaml (name -> [vx, vy, wz]) or the MicroDuck set."""
    modes = cfg.get("env", {}).get("eval_modes")
    return {k: tuple(v) for k, v in modes.items()} if modes else EVAL_MODES


def eval_score(ev: dict) -> float:
    """Checkpoint ranking: survival everywhere, then how much of each command is achieved."""
    def ratio(got, want):
        return float(np.clip(got / want, 0.0, 1.0)) if want else 0.0
    if "walk_0.3" not in ev:
        # configured modes: stand 0.15, the moving modes share 0.85, each on its commanded axis
        moving = [m for m in ev.values() if any((m["command_vx"], m["command_vy"], m["command_wz"]))]
        s = 0.15 * ev["stand"]["survival"]
        for m in moving:
            cmd = (m["command_vx"], m["command_vy"], m["command_wz"])
            axis = int(np.argmax(np.abs(cmd)))
            got = (m["mean_vx"], m["mean_vy"], m["mean_wz"])[axis]
            s += 0.85 / len(moving) * m["survival"] * (0.2 + 0.8 * ratio(got, cmd[axis]))
        return s
    s = 0.15 * ev["stand"]["survival"]
    s += 0.45 * ev["walk_0.3"]["survival"] * ratio(ev["walk_0.3"]["mean_vx"], 0.3) + 0.1 * ev["walk_0.3"]["survival"]
    if "lateral_0.2" in ev:
        s += 0.15 * ev["lateral_0.2"]["survival"] * ratio(ev["lateral_0.2"]["mean_vy"], 0.2)
    if "turn_0.8" in ev:
        s += 0.15 * ev["turn_0.8"]["survival"] * ratio(ev["turn_0.8"]["mean_wz"], 0.8)
    return s


# ---------------------------------------------------------------------------
# evaluation: the deployed int8 network, stand and walk (run_microduck_cartan_bench.evaluate)

def evaluate_modes(robot: str, cfg: dict, pk: dict, n_ep: int = 8, horizon: int = 300) -> dict:
    out = {}
    for mode, cmd in eval_modes(cfg).items():
        rets, ups, surv, vxs, vys, wzs = [], [], [], [], [], []
        for ep in range(n_ep):
            env = NNMixerEnv(robot, cfg, seed=1000 + ep)
            env.reset()
            env.command = np.array(cmd, np.float32)
            obs = env._observe()
            ret = up = vx = vy = wz = 0.0
            fell = False
            steps = 0
            for _ in range(horizon):
                obs, r, fell, _, info = env.step(dc.int8_forward(pk["layers"], pk["mean"], pk["std"], obs))
                v_body, w_z = env._base_vel_body()
                ret += r
                up += info["upright"]
                vx += v_body[0]; vy += v_body[1]; wz += w_z
                steps += 1
                if fell:
                    break
            rets.append(ret)
            ups.append(up / steps)
            vxs.append(vx / steps); vys.append(vy / steps); wzs.append(wz / steps)
            surv.append(0.0 if fell else 1.0)
        out[mode] = {"mean_return": float(np.mean(rets)), "std_return": float(np.std(rets)),
                     "mean_upright": float(np.mean(ups)), "survival": float(np.mean(surv)),
                     "mean_vx": float(np.mean(vxs)), "mean_vy": float(np.mean(vys)),
                     "mean_wz": float(np.mean(wzs)), "command_vx": cmd[0], "command_vy": cmd[1],
                     "command_wz": cmd[2], "n_episodes": n_ep, "horizon_steps": horizon}
    return out


def print_eval(ev: dict) -> None:
    for mode, m in ev.items():
        print(f"  {mode:14s} survival {m['survival'] * 100:5.1f}%  upright {m['mean_upright']:.3f}  "
              f"vx {m['mean_vx']:+.3f} vy {m.get('mean_vy', 0):+.3f} wz {m.get('mean_wz', 0):+.3f} "
              f"(cmd {m['command_vx']:+.2f} {m.get('command_vy', 0):+.2f} {m.get('command_wz', 0):+.2f})  "
              f"return {m['mean_return']:.2f}")
    print(f"  score {eval_score(ev):.3f}")


# ---------------------------------------------------------------------------
# normalizer and vectorized envs

class RunningNorm:
    """Empirical normalization (rsl_rl EmpiricalNormalization), batched Welford/Chan update."""

    def __init__(self, dim: int):
        self.n = 0
        self.mean = np.zeros(dim, np.float64)
        self.m2 = np.zeros(dim, np.float64)

    def update(self, x: np.ndarray) -> None:
        x = np.atleast_2d(x).astype(np.float64)
        nb = x.shape[0]
        mb = x.mean(0)
        m2b = ((x - mb) ** 2).sum(0)
        delta = mb - self.mean
        tot = self.n + nb
        self.mean = self.mean + delta * nb / tot
        self.m2 = self.m2 + m2b + delta ** 2 * self.n * nb / tot
        self.n = tot

    @property
    def std(self) -> np.ndarray:
        if self.n < 2:
            return np.ones_like(self.mean)
        return np.sqrt(np.maximum(self.m2 / (self.n - 1), 1e-4))

    def __call__(self, x: np.ndarray) -> np.ndarray:
        return ((x - self.mean) / self.std).astype(np.float32)


def _step_envs(envs, actions):
    """Step each env; on fall/timeout return the terminal obs and reset."""
    obs, rew, fell, tout, term, infos = [], [], [], [], [], []
    for e, a in zip(envs, actions):
        o2, r, f, t, info = e.step(a)
        term.append(o2)
        if f or t:
            o2 = e.reset()
        obs.append(o2); rew.append(r); fell.append(f); tout.append(t); infos.append(info)
    return np.stack(obs), np.array(rew), np.array(fell), np.array(tout), np.stack(term), infos


def _worker(conn, robot, cfg, seeds):
    envs = [NNMixerEnv(robot, cfg, seed=s) for s in seeds]
    conn.send(np.stack([e.reset() for e in envs]))
    while True:
        cmd, data = conn.recv()
        if cmd == "step":
            conn.send(_step_envs(envs, data))
        elif cmd == "curriculum":
            for e in envs:
                e.set_curriculum(**data)
        else:
            conn.close()
            return


class VecEnv:
    """N contract envs split over worker processes (one process when workers <= 1)."""

    def __init__(self, robot, cfg, n_envs, workers):
        self.n = n_envs
        self.contract = NNMixerEnv(robot, cfg, seed=10_000).contract
        workers = max(1, min(workers, n_envs))
        self.local = None
        if workers == 1:
            self.local = [NNMixerEnv(robot, cfg, seed=i) for i in range(n_envs)]
            self.first_obs = np.stack([e.reset() for e in self.local])
            return
        import multiprocessing as mp
        ctx = mp.get_context("spawn")
        chunks = np.array_split(np.arange(n_envs), workers)
        self.conns, self.procs, self.sizes = [], [], []
        for ch in chunks:
            a, b = ctx.Pipe()
            p = ctx.Process(target=_worker, args=(b, robot, cfg, [int(s) for s in ch]), daemon=True)
            p.start()
            self.conns.append(a); self.procs.append(p); self.sizes.append(len(ch))
        self.first_obs = np.concatenate([c.recv() for c in self.conns])

    def set_curriculum(self, **kw):
        if self.local is not None:
            for e in self.local:
                e.set_curriculum(**kw)
        else:
            for c in self.conns:
                c.send(("curriculum", kw))

    def step(self, actions):
        if self.local is not None:
            return _step_envs(self.local, actions)
        k = 0
        for c, s in zip(self.conns, self.sizes):
            c.send(("step", actions[k:k + s])); k += s
        parts = [c.recv() for c in self.conns]
        arrays = tuple(np.concatenate([p[i] for p in parts]) for i in range(5))
        infos = [i for p in parts for i in p[5]]
        return (*arrays, infos)

    def close(self):
        if self.local is None:
            for c in self.conns:
                c.send(("close", None))
            for p in self.procs:
                p.join(timeout=5)


# ---------------------------------------------------------------------------
# training

def train(args) -> None:
    import torch
    from torch import nn

    from nnm_qat import build_actor, export_nnm_from_actor
    from plot_training import plot_run

    torch.set_num_threads(max(1, args.torch_threads))
    cfg = load_ppo_config(args.robot)
    net, ppo = cfg["network"], cfg["ppo"]
    cur = {**DEFAULT_CURRICULUM, **cfg["env"].get("curriculum", {})}
    venv = VecEnv(args.robot, cfg, args.envs, args.workers)
    c = venv.contract
    obs_dim, n = c.obs_dim, c.n_joints
    torch.manual_seed(args.seed)
    qat = bool(cfg.get("quantization", {}).get("qat", True))
    actor = build_actor([obs_dim, *net["actor_hidden"], n], net.get("activation", "elu"), qat=qat)
    critic_layers: list[nn.Module] = []
    dims = [obs_dim, *net["critic_hidden"], 1]
    for i in range(len(dims) - 1):
        critic_layers.append(nn.Linear(dims[i], dims[i + 1]))
        if i < len(dims) - 2:
            critic_layers.append(nn.ELU())
    critic = nn.Sequential(*critic_layers)
    log_std = nn.Parameter(torch.full((n,), float(np.log(net.get("init_noise_std", 1.0)))))
    norm = RunningNorm(obs_dim)
    freeze_norm = False
    if args.init_onnx:
        # fine-tune an upstream policy on the ArduPilot contract: same weights, same normalizer
        from export_nnm import parse_onnx
        mean, std, layers, _, _ = parse_onnx(args.init_onnx)
        lins = [m for m in actor.modules() if isinstance(m, nn.Linear)]
        if [tuple(l.weight.shape) for l in lins] != [tuple(L["W"].shape) for L in layers]:
            raise SystemExit(f"{args.init_onnx}: layer shapes differ from ppo.yaml network")
        with torch.no_grad():
            for lin, L in zip(lins, layers):
                lin.weight.copy_(torch.from_numpy(L["W"]))
                lin.bias.copy_(torch.from_numpy(L["b"]))
        norm.mean = mean.astype(np.float64)
        norm.m2 = (std.astype(np.float64) ** 2) * 1e6
        norm.n = 1_000_001
        freeze_norm = True
        with torch.no_grad():
            log_std.fill_(float(np.log(args.init_std)))
        print(f"initialised actor and normalizer from {args.init_onnx}; exploration std {args.init_std}")
    start_iter = 0
    if args.resume:
        # continue a run: same networks, exploration noise and normalizer statistics
        blob = torch.load(args.resume, map_location="cpu", weights_only=False)
        actor.load_state_dict(blob["actor"])
        critic.load_state_dict(blob["critic"])
        with torch.no_grad():
            log_std.copy_(torch.as_tensor(blob["log_std"]))
        norm.mean = np.asarray(blob["norm_mean"], np.float64)
        norm.n = 10_000_000
        norm.m2 = np.asarray(blob["norm_std"], np.float64) ** 2 * (norm.n - 1)
        start_iter = args.start_iter
        if args.reset_std:
            with torch.no_grad():
                log_std.fill_(float(np.log(args.reset_std)))
        print(f"resumed from {args.resume} at iteration {start_iter}; std {log_std.exp().mean().item():.3f}")
    params = list(actor.parameters()) + list(critic.parameters()) + [log_std]
    if args.resume:
        lr = float(args.lr or ppo["learning_rate"])
    else:
        lr = float(ppo["learning_rate"]) if not args.init_onnx else float(args.lr or 1e-5)
    opt = torch.optim.Adam(params, lr=lr)
    warmup = args.critic_warmup if args.critic_warmup is not None else (10 if args.init_onnx else 0)
    T = int(args.steps or ppo["num_steps_per_env"])
    gamma, lam, clip = float(ppo["gamma"]), float(ppo["lam"]), float(ppo["clip_param"])
    # an upstream policy has already been through its own curriculum: start at the final values
    ramp_iters = 0 if (args.init_onnx and not args.curriculum_from_start) else int(cur["ramp_fraction"] * args.iters)
    if args.ramp_iters is not None:
        ramp_iters = args.ramp_iters

    def staged(stages, step):
        """mjlab stage curriculum: value of the last stage whose step (per-env policy steps) is reached."""
        val = stages[0][1]
        for s, v in stages:
            if step >= s:
                val = v
        return float(val)

    def curriculum_at(it: int) -> dict:
        if "action_rate_stages" in cur:
            step = it * T                           # mjlab common_step_counter: policy steps per env
            return {"action_rate": staged(cur["action_rate_stages"], step),
                    "standing_envs": staged(cur["standing_stages"], step)}
        f = 1.0 if ramp_iters == 0 else min(it / ramp_iters, 1.0)
        return {k: cur[k][0] + f * (cur[k][1] - cur[k][0]) for k in ("action_rate", "standing_envs")}

    out = Path(args.out) if args.out else robot_dir(args.robot) / "policies" / f"{args.name}.nnm"
    run_dir = out.parent / f"{out.stem}_run"
    run_dir.mkdir(parents=True, exist_ok=True)
    csv_path = run_dir / "train_log.csv"
    csv_file = None
    run_name = f"velocity_mlp_int8_{out.stem}"
    wb = None
    if args.wandb:
        import wandb
        wb = wandb.init(project=args.wandb_project or f"mjlab_{args.robot}", name=run_name,
                        id=args.wandb_id, resume="allow" if args.wandb_id else None, config={
            "robot": args.robot, "num_envs": args.envs, "max_iterations": args.iters, "num_steps_per_env": T,
            "init_onnx": str(args.init_onnx or ""), "obs_dim": obs_dim, "n_joints": n, "rate_hz": c.rate_hz,
            "network": net, "ppo": ppo, "env": cfg["env"], "curriculum": cur, "ramp_iters": ramp_iters,
            "quantization": cfg.get("quantization", {}), "deploy": "AP_NNMixer int8 .nnm"})
        print(f"wandb: {wb.url}")

    def checkpoint(tag: str) -> Path:
        path = run_dir / f"{out.stem}_{tag}.nnm"
        export_nnm_from_actor(actor, norm.mean, norm.std, c.q0, args.robot, path)
        torch.save({"actor": actor.state_dict(), "critic": critic.state_dict(), "log_std": log_std.detach(),
                    "norm_mean": norm.mean, "norm_std": norm.std}, run_dir / f"{out.stem}_{tag}.pt")
        return path

    scores: list[tuple[float, int, Path]] = []
    eval_log = run_dir / "eval_checkpoints.jsonl"
    if args.resume and eval_log.is_file():
        for line in eval_log.read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                if r["iteration"] <= start_iter and (run_dir / r["nnm"]).is_file():
                    scores.append((r["score"], r["iteration"], run_dir / r["nnm"]))
    else:
        eval_log.write_text("")

    def evaluate_checkpoint(iteration: int, path: Path) -> None:
        ev = evaluate_modes(args.robot, cfg, unpack_nnm(path.read_bytes()), n_ep=4, horizon=250)
        sc = eval_score(ev)
        scores.append((sc, iteration, path))
        print(f"eval iteration {iteration}:")
        print_eval(ev)
        with eval_log.open("a") as f:
            f.write(json.dumps({"iteration": iteration, "score": sc, "eval": ev, "nnm": path.name}) + "\n")
        if wb is not None:
            wb.log({**{f"Eval/{mode}/{k}": v for mode, mm in ev.items() for k, v in mm.items()
                       if isinstance(v, float)}, "Eval/score": sc}, step=max(iteration, 1))

    if args.save_every and not args.resume:
        evaluate_checkpoint(0, checkpoint("it0000"))   # the untrained network: first frame of the evolution video

    obs = venv.first_obs
    ep_len = np.zeros(args.envs, int)
    t_start = time.time()
    for it in range(start_iter, args.iters):
        cv = curriculum_at(it)
        venv.set_curriculum(**cv)
        t_col = time.time()
        buf_o, buf_a, buf_lp, buf_r, buf_v, buf_d = [], [], [], [], [], []
        ep_rewards, ep_lengths, term_counts = [], [], {"time_out": 0, "fell_over": 0}
        ep_terms: dict[str, list[float]] = {}
        err_xy, err_yaw = [], []
        step_rewards = []
        for _ in range(T):
            if not freeze_norm:
                norm.update(obs)
            on = norm(obs)
            with torch.no_grad():
                ot = torch.from_numpy(on)
                mu = actor(ot)
                dist = torch.distributions.Normal(mu, log_std.exp())
                a = dist.sample()
                lp = dist.log_prob(a).sum(-1)
                v = critic(ot).squeeze(-1)
            nxt, rew, fell, tout, term, infos = venv.step(a.numpy())
            ep_len += 1
            done = fell | tout
            tval = np.zeros(args.envs)
            boot = tout & ~fell
            if boot.any():
                with torch.no_grad():
                    tval[boot] = critic(torch.from_numpy(norm(term[boot]))).squeeze(-1).numpy()
            step_rewards.append(rew)
            for i, info in enumerate(infos):
                err_xy.append(info["error_vel_xy"]); err_yaw.append(info["error_vel_yaw"])
                if "episode" in info:
                    term_counts[info["reason"]] += 1
                    for k, val in info["episode"].items():
                        ep_terms.setdefault(k, []).append(val)
                    ep_rewards.append(sum(info["episode"].values()) * (cfg["env"]["episode_s"]))
                    ep_lengths.append(ep_len[i])
                    ep_len[i] = 0
            buf_o.append(on); buf_a.append(a.numpy()); buf_lp.append(lp.numpy()); buf_v.append(v.numpy())
            buf_r.append(rew + gamma * tval); buf_d.append(done.astype(float))
            obs = nxt
        collection_time = time.time() - t_col

        t_learn = time.time()
        with torch.no_grad():
            last_v = critic(torch.from_numpy(norm(obs))).squeeze(-1).numpy()
        R = np.array(buf_r); V = np.array(buf_v); D = np.array(buf_d)
        adv = np.zeros_like(R)
        gae = np.zeros(args.envs)
        for t in reversed(range(T)):
            nv = last_v if t == T - 1 else V[t + 1]
            delta = R[t] + gamma * nv * (1 - D[t]) - V[t]
            gae = delta + gamma * lam * (1 - D[t]) * gae
            adv[t] = gae
        ret = adv + V
        O = torch.from_numpy(np.concatenate(buf_o)); A = torch.from_numpy(np.concatenate(buf_a))
        LP = torch.from_numpy(np.concatenate(buf_lp)); RET = torch.from_numpy(ret.reshape(-1)).float()
        ADV = torch.from_numpy(adv.reshape(-1)).float()
        ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
        N = O.shape[0]
        mb = max(N // int(ppo["num_mini_batches"]), 1)
        kl_mean = 0.0
        l_val, l_sur, l_ent, n_upd = 0.0, 0.0, 0.0, 0
        for _ in range(int(ppo["num_learning_epochs"])):
            perm = torch.randperm(N)
            for s in range(0, N, mb):
                idx = perm[s:s + mb]
                dist = torch.distributions.Normal(actor(O[idx]), log_std.exp())
                lp_new = dist.log_prob(A[idx]).sum(-1)
                ratio = (lp_new - LP[idx]).exp()
                s1 = ratio * ADV[idx]
                s2 = ratio.clamp(1 - clip, 1 + clip) * ADV[idx]
                surrogate = -torch.min(s1, s2).mean()
                v_loss = ((critic(O[idx]).squeeze(-1) - RET[idx]) ** 2).mean()
                entropy = dist.entropy().sum(-1).mean()
                loss = surrogate + float(ppo["value_loss_coef"]) * v_loss - float(ppo["entropy_coef"]) * entropy
                opt.zero_grad()
                loss.backward()
                if it < warmup:
                    # an untrained critic would drag an upstream policy away before advantages mean anything
                    for p_ in list(actor.parameters()) + [log_std]:
                        p_.grad = None
                nn.utils.clip_grad_norm_(params, float(ppo["max_grad_norm"]))
                opt.step()
                with torch.no_grad():
                    kl_mean = float((LP[idx] - lp_new).mean())
                l_val += float(v_loss); l_sur += float(surrogate); l_ent += float(entropy); n_upd += 1
        if ppo.get("schedule") == "adaptive" and it >= warmup:
            if kl_mean > 2 * float(ppo["desired_kl"]):
                lr = max(lr / 1.5, 1e-5)
            elif 0 < kl_mean < float(ppo["desired_kl"]) / 2:
                lr = min(lr * 1.5, 1e-2)
            for g in opt.param_groups:
                g["lr"] = lr
        learning_time = time.time() - t_learn

        el = time.time() - t_start
        m = {
            "iteration": it + 1,
            "Train/mean_reward": float(np.mean(ep_rewards)) if ep_rewards else float("nan"),
            "Train/mean_episode_length": float(np.mean(ep_lengths)) if ep_lengths else float("nan"),
            "Train/mean_step_reward": float(np.mean(step_rewards)),
            "Episode_Termination/time_out": term_counts["time_out"] / T,
            "Episode_Termination/fell_over": term_counts["fell_over"] / T,
            "Metrics/twist/error_vel_xy": float(np.mean(err_xy)),
            "Metrics/twist/error_vel_yaw": float(np.mean(err_yaw)),
            "Loss/value": l_val / max(n_upd, 1),
            "Loss/surrogate": l_sur / max(n_upd, 1),
            "Loss/entropy": l_ent / max(n_upd, 1),
            "Loss/learning_rate": lr,
            "Loss/kl": kl_mean,
            "Policy/mean_std": float(log_std.exp().mean()),
            "Perf/total_fps": args.envs * T / (collection_time + learning_time),
            "Perf/collection_time": collection_time,
            "Perf/learning_time": learning_time,
            "Curriculum/action_rate_weight": cv["action_rate"],
            "Curriculum/standing_envs": cv["standing_envs"],
            "time_s": el,
            "env_steps": (it + 1) * args.envs * T,
        }
        for k in REWARD_TERMS + [t for t in ep_terms if t not in REWARD_TERMS]:
            vals = ep_terms.get(k)
            m[f"Episode_Reward/{k}"] = float(np.mean(vals)) if vals else float("nan")
        if csv_file is None:
            keys = list(m.keys())
            if args.resume and csv_path.is_file():
                # keep the earlier rows up to the resume point; columns added since then are
                # appended and padded with nan in the old rows
                old = csv_path.read_text().splitlines()
                old_keys = old[0].split(",")
                keys = old_keys + [k for k in m if k not in old_keys]
                pad = ",nan" * (len(keys) - len(old_keys))
                kept = [r + pad for r in old[1:] if r and float(r.split(",")[0]) <= start_iter]
                csv_file = csv_path.open("w")
                csv_file.write("\n".join([",".join(keys), *kept]) + "\n")
            else:
                csv_file = csv_path.open("w")
                csv_file.write(",".join(keys) + "\n")
        csv_file.write(",".join(f"{m.get(k, float('nan')):.6g}" for k in keys) + "\n")
        csv_file.flush()
        if wb is not None:
            wb.log({k: v for k, v in m.items() if k != "iteration"}, step=it + 1)
        print(f"it {it + 1:4d}/{args.iters}  reward {m['Train/mean_reward']:7.2f}  "
              f"ep_len {m['Train/mean_episode_length']:6.1f}  fell {term_counts['fell_over']:4d}  "
              f"err_xy {m['Metrics/twist/error_vel_xy']:.3f}  std {m['Policy/mean_std']:.3f}  "
              f"lr {lr:.1e}  fps {m['Perf/total_fps']:6.0f}  {el:6.1f}s", flush=True)
        if args.save_every and (it + 1) % args.save_every == 0:
            ck = checkpoint(f"it{it + 1:04d}")
            if args.eval_every and (it + 1) % args.eval_every == 0:
                evaluate_checkpoint(it + 1, ck)

    venv.close()
    csv_file.close()
    final = checkpoint("final")
    if not (args.eval_every and args.save_every and args.iters % args.eval_every == 0
            and args.iters % args.save_every == 0):
        evaluate_checkpoint(args.iters, final)
    # the policy file is the best evaluated checkpoint, not necessarily the last one
    import shutil
    best_dir = run_dir / "best"
    best_dir.mkdir(exist_ok=True)
    ranked = sorted(scores, key=lambda s: -s[0])
    for rank, (sc, iteration, path) in enumerate(ranked[:3], 1):
        shutil.copy(path, best_dir / f"{out.stem}_best{rank}_it{iteration:04d}.nnm")
        print(f"best {rank}: iteration {iteration} score {sc:.3f}")
    best_score, best_it, best_path = ranked[0]
    shutil.copy(best_path, out)
    blob = out.read_bytes()
    pk = unpack_nnm(blob)
    print(f"policy file {out} = checkpoint of iteration {best_it} (score {best_score:.3f})")
    blob_train = export_nnm_from_actor(actor, norm.mean, norm.std, c.q0, args.robot, run_dir / "_last.nnm")
    pk_last = unpack_nnm(blob_train)
    probe = norm.mean + norm.std * np.random.default_rng(0).normal(size=(32, obs_dim))
    with torch.no_grad():
        y_train = actor(torch.from_numpy(norm(probe))).numpy()
    y_nnm = np.stack([dc.int8_forward(pk_last["layers"], pk_last["mean"], pk_last["std"], p.astype(np.float32))
                      for p in probe])
    (run_dir / "_last.nnm").unlink()
    print(f"wrote {out} ({len(blob)} bytes). last iteration: max |train - deployed| = "
          f"{np.max(np.abs(y_train - y_nnm)):.2e}")

    print(f"final evaluation of {out.name} (iteration {best_it}, deployed int8 network, 8 episodes x 300 steps):")
    ev = evaluate_modes(args.robot, cfg, pk)
    print_eval(ev)
    (run_dir / "eval_report.json").write_text(json.dumps({"policy": out.name, "eval": ev}, indent=2))
    fig = plot_run(csv_path, run_dir / "training_curves.png",
                   title=f"{args.robot} — PPO MLP int8 (QAT) su contratto ArduPilot — {args.envs} env × {args.iters} iter",
                   eval_report={k: ev[k] for k in list(ev)[:2]})
    print(f"curves: {fig}")
    if wb is not None:
        import wandb
        wb.log({f"Eval_final/{mode}/{k}": v for mode, mm in ev.items() for k, v in mm.items()
                if isinstance(v, float)})
        wb.log({"training_curves": wandb.Image(str(fig))})
        art = wandb.Artifact(f"{args.robot}-{out.stem}", type="nnm-policy")
        art.add_file(str(out))
        for p in sorted(best_dir.glob("*.nnm")):
            art.add_file(str(p))
        art.add_file(str(run_dir / "eval_report.json"))
        wb.log_artifact(art)
        wb.finish()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--workers", type=int, default=1, help="processes stepping the envs in parallel")
    ap.add_argument("--torch-threads", type=int, default=4, help="threads for the PPO update")
    ap.add_argument("--iters", type=int, default=50)
    ap.add_argument("--steps", type=int, default=0, help="steps per env per iteration (default from ppo.yaml)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--name", default="walk_cpu")
    ap.add_argument("--out", default=None)
    ap.add_argument("--save-every", type=int, default=100, help="checkpoint .nnm/.pt every N iterations")
    ap.add_argument("--eval-every", type=int, default=0, help="stand/walk eval at checkpoints (multiple of save-every)")
    ap.add_argument("--wandb", action="store_true", help="log to Weights & Biases (project mjlab_<robot>)")
    ap.add_argument("--wandb-project", default=None)
    ap.add_argument("--eval", type=Path, default=None, help="evaluate a .nnm (stand / walk) and exit")
    ap.add_argument("--init-onnx", type=Path, default=None,
                    help="start from an upstream MLP ONNX (fine-tune on the ArduPilot contract, int8)")
    ap.add_argument("--init-std", type=float, default=0.1, help="exploration std when fine-tuning")
    ap.add_argument("--lr", type=float, default=None, help="learning rate when fine-tuning (default 1e-5)")
    ap.add_argument("--critic-warmup", type=int, default=None,
                    help="iterations that train only the critic (default 10 with --init-onnx, else 0)")
    ap.add_argument("--curriculum-from-start", action="store_true",
                    help="run the curriculum from its initial values even with --init-onnx")
    ap.add_argument("--ramp-iters", type=int, default=None, help="curriculum ramp length (iterations)")
    ap.add_argument("--resume", type=Path, default=None, help="checkpoint .pt to continue from")
    ap.add_argument("--start-iter", type=int, default=0, help="iteration number of the --resume checkpoint")
    ap.add_argument("--wandb-id", default=None, help="continue this W&B run id")
    ap.add_argument("--reset-std", type=float, default=None, help="exploration std after --resume")
    args = ap.parse_args()
    if args.eval:
        cfg = load_ppo_config(args.robot)
        pk = unpack_nnm(args.eval.read_bytes())
        if pk["robot_id"] != args.robot:
            raise SystemExit(f"refusing: {args.eval} is for robot {pk['robot_id']!r}, not {args.robot!r}")
        print(f"{args.eval} (deployed int8 network, 8 episodes x 300 steps):")
        ev = evaluate_modes(args.robot, cfg, pk)
        print_eval(ev)
        return
    train(args)


if __name__ == "__main__":
    main()
