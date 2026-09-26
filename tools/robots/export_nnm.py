#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Quantize an NNMixer MLP ONNX (baked normalizer) to a .nnm int8 pack for SD/RAM.

File layout (little-endian):
  magic[4] = b'NNM1'
  robot_id[16] null-padded ASCII
  obs_dim u16, act_dim u16, n_layers u8, flags u8 (bit0=int8, bit1=per-row W scale)
  reserved u16
  dims[n_layers+1] u16
  act[n_layers] u8  (1 = ELU after layer)
  obs_mean[obs_dim] f32, obs_std[obs_dim] f32
  default_pose[act_dim] f32
  for each layer (out x in):
    w_scale[out] f32   # per-row symmetric scale
    b[out] f32         # bias kept float32 (tiny)
    W int8[out * in] row-major

Also writes optional float32 C header via tools/export_policy_c.py for parity.

Usage:
    python tools/robots/export_nnm.py policies/microduck_mlp_2048x2000_it1999.onnx \\
        --robot microduck --out robots/microduck/policies/walk.nnm
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROBOT_INDEX, load_profile, robot_dir  # noqa: E402


def quantize_rows(W: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per-output-row symmetric int8 quantization. Returns (W_int8, scales[out])."""
    W = W.astype(np.float32)
    out, _ = W.shape
    scales = np.empty(out, dtype=np.float32)
    Q = np.empty_like(W, dtype=np.int8)
    for r in range(out):
        row = W[r]
        vmax = float(np.max(np.abs(row))) if row.size else 0.0
        if vmax < 1e-12:
            scales[r] = 1.0
            Q[r] = 0
        else:
            scales[r] = vmax / 127.0
            Q[r] = np.clip(np.round(row / scales[r]), -127, 127).astype(np.int8)
    return Q, scales


def parse_onnx(path: Path):
    import onnx
    from onnx import numpy_helper

    m = onnx.load(str(path))
    g = m.graph
    inits = {t.name: numpy_helper.to_array(t) for t in g.initializer}
    meta = {p.key: p.value for p in m.metadata_props}
    nodes = list(g.node)
    if nodes[0].op_type != "Sub" or nodes[1].op_type != "Div":
        raise SystemExit("expected baked normalizer (Sub, Div) at graph start")
    mean = inits[nodes[0].input[1]].reshape(-1).astype(np.float32)
    std = inits[nodes[1].input[1]].reshape(-1).astype(np.float32)
    layers = []
    for n in nodes[2:]:
        if n.op_type == "Gemm":
            attrs = {a.name: onnx.helper.get_attribute_value(a) for a in n.attribute}
            W = inits[n.input[1]].astype(np.float32)
            b = inits[n.input[2]].astype(np.float32)
            if not attrs.get("transB", 0):
                W = W.T
            layers.append({"W": W, "b": b, "act": False})
        elif n.op_type == "Elu":
            layers[-1]["act"] = True
        else:
            raise SystemExit(f"unsupported op: {n.op_type}")
    default_pose = np.array(
        [float(x) for x in meta.get("default_joint_pos", "").split(",") if x],
        dtype=np.float32,
    )
    return mean, std, layers, default_pose, meta


def pack_nnm(
    robot_id: str,
    mean: np.ndarray,
    std: np.ndarray,
    layers: list,
    default_pose: np.ndarray,
) -> bytes:
    obs_dim = int(mean.size)
    act_dim = int(layers[-1]["W"].shape[0])
    n_layers = len(layers)
    dims = [int(layers[0]["W"].shape[1])] + [int(l["W"].shape[0]) for l in layers]
    if dims[0] != obs_dim or dims[-1] != act_dim:
        raise SystemExit(f"shape mismatch dims={dims} obs={obs_dim} act={act_dim}")
    if default_pose.size == 0:
        default_pose = np.zeros(act_dim, dtype=np.float32)
    if default_pose.size != act_dim:
        raise SystemExit(f"default_pose length {default_pose.size} != act_dim {act_dim}")

    rid = robot_id.encode("ascii")[:16]
    rid = rid + b"\0" * (16 - len(rid))
    buf = bytearray()
    buf += b"NNM1"
    buf += rid
    flags = 0x03  # bit0 int8, bit1 per-row W scale, bias float32
    buf += struct.pack("<HHBBH", obs_dim, act_dim, n_layers, flags, 0)
    buf += struct.pack("<" + "H" * len(dims), *dims)
    buf += bytes(1 if l["act"] else 0 for l in layers)
    buf += mean.astype(np.float32).tobytes()
    buf += std.astype(np.float32).tobytes()
    buf += default_pose.astype(np.float32).tobytes()
    for l in layers:
        Wq, w_scales = quantize_rows(l["W"])
        buf += w_scales.astype(np.float32).tobytes()
        buf += l["b"].astype(np.float32).tobytes()
        buf += Wq.tobytes()
    return bytes(buf)


def dequant_forward(mean, std, layers_q, obs: np.ndarray) -> np.ndarray:
    x = (obs.astype(np.float32) - mean) / std
    for Wq, w_scales, b, use_elu in layers_q:
        W = Wq.astype(np.float32) * w_scales[:, None]
        x = W @ x + b
        if use_elu:
            x = np.where(x > 0, x, np.expm1(x))
    return x


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("onnx_path", type=Path)
    ap.add_argument("--robot", required=True, choices=sorted(ROBOT_INDEX))
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--parity", action="store_true", help="Compare int8 vs float ONNX on random obs")
    ap.add_argument("--also-c-header", type=Path, default=None, help="Also run export_policy_c.py")
    args = ap.parse_args()

    profile = load_profile(args.robot)
    mean, std, layers, default_pose, meta = parse_onnx(args.onnx_path)
    act_dim = layers[-1]["W"].shape[0]
    n_joints = int(profile.get("n_joints") or act_dim)
    if profile.get("n_joints") and profile["n_joints"] != act_dim:
        raise SystemExit(
            f"refusing: ONNX act_dim={act_dim} != profile n_joints={profile['n_joints']} "
            f"(policy of another robot?)"
        )
    if profile.get("obs_dim") and profile["obs_dim"] != mean.size:
        raise SystemExit(
            f"refusing: ONNX obs_dim={mean.size} != profile obs_dim={profile['obs_dim']}"
        )

    out = args.out or (robot_dir(args.robot) / "policies" / (args.onnx_path.stem + ".nnm"))
    out.parent.mkdir(parents=True, exist_ok=True)
    blob = pack_nnm(args.robot, mean, std, layers, default_pose)
    out.write_bytes(blob)
    print(
        f"wrote {out} ({len(blob)} bytes, ~{len(blob)/1024:.1f} KB) "
        f"robot_id={args.robot} obs={mean.size} act={act_dim} layers={len(layers)}"
    )

    if args.parity:
        layers_q = []
        for l in layers:
            Wq, w_scales = quantize_rows(l["W"])
            layers_q.append((Wq, w_scales, l["b"].astype(np.float32), l["act"]))
        rng = np.random.default_rng(0)
        errs = []
        for _ in range(32):
            obs = rng.normal(size=mean.size).astype(np.float32)
            x = (obs - mean) / std
            for l in layers:
                x = l["W"] @ x + l["b"]
                if l["act"]:
                    x = np.where(x > 0, x, np.expm1(x))
            yq = dequant_forward(mean, std, layers_q, obs)
            errs.append(float(np.max(np.abs(x - yq))))
        print(f"parity max|float-int8| over 32 random obs: {max(errs):.6e} (mean {np.mean(errs):.6e})")

    if args.also_c_header:
        import subprocess

        cmd = [
            sys.executable,
            str(Path(__file__).resolve().parents[1] / "export_policy_c.py"),
            str(args.onnx_path),
            "--out",
            str(args.also_c_header),
            "--name",
            "mlp",
        ]
        subprocess.check_call(cmd)

    _ = n_joints  # documented in profile check above
    _ = meta


if __name__ == "__main__":
    main()
