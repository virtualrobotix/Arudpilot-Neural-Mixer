# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Int8 training -> .nnm -> firmware C forward, and the deployment contract.

    .venv/bin/python -m pytest tests/test_nnm_pipeline.py -q
"""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "robots"))

import deploy_contract as dc  # noqa: E402
from export_nnm import unpack_nnm  # noqa: E402

torch = pytest.importorskip("torch")
from nnm_qat import build_actor, enable_qat, export_nnm_from_actor, QATLinear  # noqa: E402


def _random_actor(dims, seed=0):
    torch.manual_seed(seed)
    return build_actor(dims, qat=True)


def test_qat_export_is_exact(tmp_path):
    dims = [61, 512, 256, 128, 14]
    actor = _random_actor(dims)
    rng = np.random.default_rng(0)
    mean = rng.normal(size=61).astype(np.float32)
    std = (rng.random(61) + 0.5).astype(np.float32)
    q0 = rng.normal(size=14).astype(np.float32) * 0.1
    path = tmp_path / "p.nnm"
    export_nnm_from_actor(actor, mean, std, q0, "microduck", path)
    pk = unpack_nnm(path.read_bytes())
    assert pk["robot_id"] == "microduck" and pk["dims"] == dims and pk["flags"] == 3
    obs = rng.normal(size=(64, 61)).astype(np.float32)
    with torch.no_grad():
        y_train = actor(torch.from_numpy((obs - mean) / std)).numpy()
    y_nnm = np.stack([dc.int8_forward(pk["layers"], pk["mean"], pk["std"], o) for o in obs])
    assert np.max(np.abs(y_train - y_nnm)) < 1e-4


def test_qat_trains():
    torch.manual_seed(1)
    actor = build_actor([10, 32, 4], qat=True)
    x = torch.randn(256, 10)
    target = torch.randn(256, 4) * 0.5
    opt = torch.optim.Adam(actor.parameters(), lr=1e-2)
    loss0 = None
    for _ in range(200):
        loss = ((actor(x) - target) ** 2).mean()
        if loss0 is None:
            loss0 = loss.item()
        opt.zero_grad()
        loss.backward()
        opt.step()
    assert loss.item() < 0.5 * loss0


def test_enable_qat_in_place():
    seq = torch.nn.Sequential(torch.nn.Linear(4, 8), torch.nn.ELU(), torch.nn.Linear(8, 2))
    enable_qat(seq)
    assert all(isinstance(m, QATLinear) for m in seq if isinstance(m, torch.nn.Linear))


@pytest.mark.skipif(shutil.which("cc") is None, reason="needs a C compiler")
def test_c_forward_matches(tmp_path):
    dims = [45, 512, 256, 128, 12]
    actor = _random_actor(dims, seed=3)
    rng = np.random.default_rng(3)
    mean = np.zeros(45, np.float32)
    std = np.ones(45, np.float32)
    path = tmp_path / "q.nnm"
    export_nnm_from_actor(actor, mean, std, np.zeros(12, np.float32), "rex", path)
    exe = tmp_path / "parity_nnm"
    subprocess.check_call(["cc", "-O2", "-I", str(ROOT / "tools"),
                           str(ROOT / "tools/robots/parity_nnm.c"), str(ROOT / "tools/nnmixer_infer.c"),
                           "-lm", "-o", str(exe)])
    obs = rng.normal(size=(128, 45)).astype(np.float32)
    (tmp_path / "obs.f32").write_bytes(obs.tobytes())
    subprocess.check_call([str(exe), str(path), str(tmp_path / "obs.f32"), str(tmp_path / "act.f32")])
    y_c = np.frombuffer((tmp_path / "act.f32").read_bytes(), np.float32).reshape(128, 12)
    with torch.no_grad():
        y_train = actor(torch.from_numpy(obs)).numpy()
    assert np.max(np.abs(y_c - y_train)) < 1e-4


def test_contract_layout_and_pwm():
    c = dc.Contract(n_joints=12, q0=np.zeros(12, np.float32))
    assert c.obs_dim == 45 and c.twist_offset == 42 and c.loop_steps_per_policy == 4
    a_prev, q_wire = c_apply = dc.apply_action(c, np.full(12, 3.0, np.float32))
    assert np.allclose(a_prev, 2.0)
    assert np.allclose(q_wire, 2.001, atol=1e-6)          # 1500 + round(2.0/0.003)=2167 us
    _, q_wire = dc.apply_action(c, np.full(12, 0.0014, np.float32))
    assert np.allclose(q_wire, 0.0)                        # below half a PWM step


def test_rate_must_divide_50():
    with pytest.raises(ValueError):
        _ = dc.Contract(n_joints=8, q0=np.zeros(8), rate_hz=20).rate_div
    assert dc.Contract(n_joints=8, q0=np.zeros(8), rate_hz=25).loop_steps_per_policy == 8


def test_gravity_filter_level():
    g = dc.GravityFilter()
    for _ in range(10):
        g.update(np.zeros(3), np.array([0.0, 0.0, -dc.GRAVITY_MSS]), 0.005)
    assert np.allclose(g.gravity_flu(), [0.0, 0.0, -1.0], atol=1e-6)
