# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Int8-aware PPO actor for AP_NNMixer (quantization-aware training).

The firmware runs weight-only int8: each weight row has one float scale,
bias and activations stay float32 (nnmixer_forward_int8). Training the actor
with the same fake quantization in the forward pass means the policy learns
on the exact weight grid it will run on. Export then copies the int8 values;
there is no post-training quantization step and no parity loss from it.

Usage inside an rsl_rl / mjlab training script:

    from nnm_qat import enable_qat, export_nnm_from_actor
    enable_qat(runner.alg.policy.actor)          # before training, or before a fine-tune
    ...
    export_nnm_from_actor(runner.alg.policy.actor, normalizer_mean, normalizer_std,
                          q0, robot_id="microban", path="robots/microban/policies/walk.nnm")

Gradients use the straight-through estimator: the backward pass treats the
rounding as identity, the forward pass sees the quantized weights.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parent))
from export_nnm import pack_nnm_quantized  # noqa: E402

QMAX = 127


def quantize_rows_torch(W: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Per-output-row symmetric int8, same rule as export_nnm.quantize_rows."""
    vmax = W.detach().abs().amax(dim=1)
    scale = torch.where(vmax > 1e-12, vmax / QMAX, torch.ones_like(vmax))
    q = torch.clamp(torch.round(W / scale[:, None]), -QMAX, QMAX)
    return q, scale


def fake_quant_rows(W: torch.Tensor) -> torch.Tensor:
    q, scale = quantize_rows_torch(W)
    Wq = q * scale[:, None]
    return W + (Wq - W).detach()


class QATLinear(nn.Linear):
    """nn.Linear whose forward uses the int8 grid the firmware runs."""

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return nn.functional.linear(x, fake_quant_rows(self.weight), self.bias)

    @classmethod
    def from_linear(cls, lin: nn.Linear) -> "QATLinear":
        q = cls(lin.in_features, lin.out_features, bias=lin.bias is not None,
                device=lin.weight.device, dtype=lin.weight.dtype)
        with torch.no_grad():
            q.weight.copy_(lin.weight)
            if lin.bias is not None:
                q.bias.copy_(lin.bias)
        return q


def enable_qat(module: nn.Module) -> nn.Module:
    """Replace every nn.Linear inside module (e.g. the rsl_rl actor) with QATLinear, in place."""
    for name, child in list(module.named_children()):
        if isinstance(child, nn.Linear) and not isinstance(child, QATLinear):
            setattr(module, name, QATLinear.from_linear(child))
        else:
            enable_qat(child)
    return module


def build_actor(dims: list[int], activation: str = "elu", qat: bool = True) -> nn.Sequential:
    """Actor MLP obs -> hidden... -> n_joints, same topology the firmware parses."""
    if activation != "elu":
        raise ValueError("AP_NNMixer supports ELU(alpha=1) hidden activations only")
    layers: list[nn.Module] = []
    for i in range(len(dims) - 1):
        lin = nn.Linear(dims[i], dims[i + 1])
        layers.append(QATLinear.from_linear(lin) if qat else lin)
        if i < len(dims) - 2:
            layers.append(nn.ELU())
    return nn.Sequential(*layers)


def actor_linears(actor: nn.Module) -> list[nn.Linear]:
    lins = [m for m in actor.modules() if isinstance(m, nn.Linear)]
    if not lins:
        raise ValueError("actor has no Linear layers")
    return lins


def export_nnm_from_actor(actor: nn.Module, obs_mean, obs_std, q0, robot_id: str, path) -> bytes:
    """Write the .nnm the firmware loads. Weights are the int8 values used in training."""
    lins = actor_linears(actor)
    qlayers = []
    for i, lin in enumerate(lins):
        q, scale = quantize_rows_torch(lin.weight.detach().float().cpu())
        bias = (lin.bias.detach().float().cpu().numpy() if lin.bias is not None
                else np.zeros(lin.out_features, dtype=np.float32))
        qlayers.append({
            "Wq": q.numpy().astype(np.int8),
            "w_scale": scale.numpy().astype(np.float32),
            "b": bias.astype(np.float32),
            "act": i < len(lins) - 1,
        })
    blob = pack_nnm_quantized(
        robot_id,
        np.asarray(obs_mean, dtype=np.float32).reshape(-1),
        np.asarray(obs_std, dtype=np.float32).reshape(-1),
        qlayers,
        np.asarray(q0, dtype=np.float32).reshape(-1),
    )
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(blob)
    return blob
