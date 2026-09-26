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

This CPU trainer runs the same contract end to end and is meant for pipeline
checks and small robots. Full-quality policies (2048 envs x 2000 iterations)
use mjlab on a GPU with the same two pieces: deploy_contract for observation
and action, nnm_qat.enable_qat() on the actor. See docs/robots/training.md.

Usage:
    python tools/robots/train_velocity.py --robot microduck --envs 8 --iters 50 --name walk_cpu
    python tools/robots/train_velocity.py --robot microduck --eval robots/microduck/policies/walk.nnm
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
from common import robot_dir  # noqa: E402
from export_nnm import unpack_nnm  # noqa: E402
from nnm_env import NNMixerEnv, load_ppo_config  # noqa: E402


def evaluate(robot: str, nnm_path: Path, seconds: float = 10.0) -> None:
    cfg = load_ppo_config(robot)
    pk = unpack_nnm(nnm_path.read_bytes())
    if pk["robot_id"] != robot:
        raise SystemExit(f"refusing: {nnm_path} is for robot {pk['robot_id']!r}, not {robot!r}")
    for label, cmd in [("stand", [0, 0, 0]), ("forward", [0.3, 0, 0]), ("turn", [0, 0, 0.8])]:
        env = NNMixerEnv(robot, cfg, seed=0)
        env.reset()
        env.command = np.array(cmd, np.float32)
        obs = env._observe()
        x0 = env.data.qpos[env.free_qpos:env.free_qpos + 2].copy()
        fell = False
        for _ in range(int(seconds * env.contract.rate_hz)):
            obs, _, fell, _, _ = env.step(dc.int8_forward(pk["layers"], pk["mean"], pk["std"], obs))
            if fell:
                break
        d = env.data.qpos[env.free_qpos:env.free_qpos + 2] - x0
        t = env.steps / env.contract.rate_hz
        print(f"{label:8s} cmd={cmd} t={t:.1f}s fell={fell} tilt={env.tilt_deg():.1f}deg "
              f"displacement={np.round(d, 3).tolist()} speed={np.linalg.norm(d) / max(t, 1e-6):.3f} m/s")


class RunningNorm:
    def __init__(self, dim: int):
        self.n = 0
        self.mean = np.zeros(dim, np.float64)
        self.m2 = np.zeros(dim, np.float64)

    def update(self, x: np.ndarray) -> None:
        for row in np.atleast_2d(x):
            self.n += 1
            d = row - self.mean
            self.mean += d / self.n
            self.m2 += d * (row - self.mean)

    @property
    def std(self) -> np.ndarray:
        var = self.m2 / max(self.n - 1, 1)
        return np.sqrt(np.maximum(var, 1e-4))

    def __call__(self, x: np.ndarray) -> np.ndarray:
        return ((x - self.mean) / self.std).astype(np.float32)


def train(args) -> None:
    import torch
    from torch import nn

    from nnm_qat import build_actor, export_nnm_from_actor

    cfg = load_ppo_config(args.robot)
    net, ppo = cfg["network"], cfg["ppo"]
    envs = [NNMixerEnv(args.robot, cfg, seed=i) for i in range(args.envs)]
    c = envs[0].contract
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
    params = list(actor.parameters()) + list(critic.parameters()) + [log_std]
    lr = float(ppo["learning_rate"]) if not args.init_onnx else float(args.lr or 1e-5)
    opt = torch.optim.Adam(params, lr=lr)
    warmup = args.critic_warmup if args.critic_warmup is not None else (10 if args.init_onnx else 0)
    T = int(args.steps or ppo["num_steps_per_env"])
    gamma, lam, clip = float(ppo["gamma"]), float(ppo["lam"]), float(ppo["clip_param"])

    obs = np.stack([e.reset() for e in envs])
    ep_ret = np.zeros(args.envs)
    ep_len = np.zeros(args.envs, int)
    done_ret: list[float] = []
    done_len: list[int] = []
    t_start = time.time()
    for it in range(args.iters):
        buf_o, buf_a, buf_lp, buf_r, buf_v, buf_d, buf_tv = [], [], [], [], [], [], []
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
            nxt, rew, dn, tval = [], [], [], []
            for i, e in enumerate(envs):
                o2, r, fell, timeout, _ = e.step(a[i].numpy())
                ep_ret[i] += r
                ep_len[i] += 1
                tv = 0.0
                if timeout and not fell:
                    with torch.no_grad():
                        tv = float(critic(torch.from_numpy(norm(o2[None]))).item())
                if fell or timeout:
                    done_ret.append(ep_ret[i]); done_len.append(ep_len[i])
                    ep_ret[i] = 0.0; ep_len[i] = 0
                    o2 = e.reset()
                nxt.append(o2); rew.append(r); dn.append(fell or timeout); tval.append(tv)
            buf_o.append(on); buf_a.append(a.numpy()); buf_lp.append(lp.numpy()); buf_v.append(v.numpy())
            buf_r.append(np.array(rew) + gamma * np.array(tval)); buf_d.append(np.array(dn, float))
            obs = np.stack(nxt)
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
        for _ in range(int(ppo["num_learning_epochs"])):
            perm = torch.randperm(N)
            for s in range(0, N, mb):
                idx = perm[s:s + mb]
                dist = torch.distributions.Normal(actor(O[idx]), log_std.exp())
                lp_new = dist.log_prob(A[idx]).sum(-1)
                ratio = (lp_new - LP[idx]).exp()
                s1 = ratio * ADV[idx]
                s2 = ratio.clamp(1 - clip, 1 + clip) * ADV[idx]
                v_loss = ((critic(O[idx]).squeeze(-1) - RET[idx]) ** 2).mean()
                loss = -torch.min(s1, s2).mean() + float(ppo["value_loss_coef"]) * v_loss \
                    - float(ppo["entropy_coef"]) * dist.entropy().sum(-1).mean()
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
        if ppo.get("schedule") == "adaptive" and it >= warmup:
            if kl_mean > 2 * float(ppo["desired_kl"]):
                lr = max(lr / 1.5, 1e-5)
            elif 0 < kl_mean < float(ppo["desired_kl"]) / 2:
                lr = min(lr * 1.5, 1e-2)
            for g in opt.param_groups:
                g["lr"] = lr
        mr = np.mean(done_ret[-50:]) if done_ret else float("nan")
        ml = np.mean(done_len[-50:]) if done_len else float("nan")
        print(f"it {it + 1:4d}/{args.iters}  mean_ep_return {mr:7.3f}  mean_ep_len {ml:6.1f}  "
              f"kl {kl_mean:+.4f}  lr {lr:.2e}  std {log_std.exp().mean().item():.3f}  "
              f"{time.time() - t_start:6.1f}s", flush=True)

    out = Path(args.out) if args.out else robot_dir(args.robot) / "policies" / f"{args.name}.nnm"
    blob = export_nnm_from_actor(actor, norm.mean, norm.std, c.q0, args.robot, out)
    pk = unpack_nnm(blob)
    probe = norm.mean + norm.std * np.random.default_rng(0).normal(size=(32, obs_dim))
    with torch.no_grad():
        y_train = actor(torch.from_numpy(norm(probe))).numpy()
    y_nnm = np.stack([dc.int8_forward(pk["layers"], pk["mean"], pk["std"], p.astype(np.float32)) for p in probe])
    print(f"wrote {out} ({len(blob)} bytes). max |train - deployed| = {np.max(np.abs(y_train - y_nnm)):.2e}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--iters", type=int, default=50)
    ap.add_argument("--steps", type=int, default=0, help="steps per env per iteration (default from ppo.yaml)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--name", default="walk_cpu")
    ap.add_argument("--out", default=None)
    ap.add_argument("--eval", type=Path, default=None, help="run a .nnm through the contract env and exit")
    ap.add_argument("--init-onnx", type=Path, default=None,
                    help="start from an upstream MLP ONNX (fine-tune on the ArduPilot contract, int8)")
    ap.add_argument("--init-std", type=float, default=0.1, help="exploration std when fine-tuning")
    ap.add_argument("--lr", type=float, default=None, help="learning rate when fine-tuning (default 1e-5)")
    ap.add_argument("--critic-warmup", type=int, default=None,
                    help="iterations that train only the critic (default 10 with --init-onnx, else 0)")
    args = ap.parse_args()
    if args.eval:
        evaluate(args.robot, args.eval)
        return
    train(args)


if __name__ == "__main__":
    main()
