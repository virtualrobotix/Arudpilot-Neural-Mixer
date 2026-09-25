#!/usr/bin/env python3
"""Parity test: ONNX Runtime vs the pure-C forward on random + realistic observations.

    python tools/parity_check.py policies/microduck_mlp_2048x2000_it1999.onnx --name mlp

Builds parity_check.c with the generated header in a temp dir, streams float32 obs through it,
and compares with onnxruntime. Also reports forward latency of the C build (p50/p99) as a
first proxy for the MCU budget discussion.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent


def realistic_obs(n: int, rng: np.random.Generator) -> np.ndarray:
    """Observations in the ranges the policy saw in training (SI units)."""
    obs = np.zeros((n, 61), dtype=np.float32)
    obs[:, 0:3] = rng.normal(0, 0.8, (n, 3))                       # gyro rad/s
    g = rng.normal([0, 0, -1], 0.15, (n, 3))
    obs[:, 3:6] = g / np.linalg.norm(g, axis=1, keepdims=True)     # projected gravity
    obs[:, 6:20] = rng.normal(0, 0.3, (n, 14))                      # q - q0
    obs[:, 20:34] = rng.normal(0, 2.0, (n, 14))                     # qdot
    obs[:, 34:48] = rng.normal(0, 0.4, (n, 14))                     # last action
    obs[:, 48] = rng.uniform(-0.4, 0.4, n)
    obs[:, 49] = rng.uniform(-0.3, 0.3, n)
    obs[:, 50] = rng.uniform(-1.0, 1.0, n)
    return obs


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("onnx_path")
    ap.add_argument("--name", default="mlp")
    ap.add_argument("--n", type=int, default=2000)
    ap.add_argument("--header", default=None, help="existing generated header (default: generate in temp)")
    ap.add_argument("--cc", default="cc")
    ap.add_argument("--opt", default="-O2")
    ap.add_argument("--kind", choices=["mlp", "cartan"], default="mlp")
    args = ap.parse_args()

    exporter = "export_policy_c.py" if args.kind == "mlp" else "export_cartan_c.py"
    harness = "parity_check.c" if args.kind == "mlp" else "parity_check_cartan.c"
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        header = Path(args.header) if args.header else td / f"policy_{args.name}.h"
        if not args.header:
            subprocess.run([sys.executable, str(HERE / exporter), args.onnx_path, "--out", str(header), "--name", args.name], check=True)
        exe = td / "parity"
        cmd = [args.cc, args.opt, "-std=c11", "-Wall", "-Wextra",
               f"-DPOLICY_HEADER=\"{header}\"", f"-DPOLICY_PREFIX={args.name}", f"-DPOLICY_PREFIX_U={args.name.upper()}",
               "-I", str(HERE), str(HERE / harness), str(HERE / "nnmixer_infer.c"), "-o", str(exe), "-lm"]
        subprocess.run(cmd, check=True)

        rng = np.random.default_rng(0)
        obs = np.concatenate([realistic_obs(args.n, rng), rng.normal(0, 3, (200, 61)).astype(np.float32), np.zeros((1, 61), np.float32)])
        obs = np.ascontiguousarray(obs, dtype=np.float32)

        sess = ort.InferenceSession(args.onnx_path, providers=["CPUExecutionProvider"])
        in_name = sess.get_inputs()[0].name
        ref = np.concatenate([sess.run(None, {in_name: o[None]})[0] for o in obs])

        t0 = time.perf_counter()
        res = subprocess.run([str(exe)], input=obs.tobytes(), stdout=subprocess.PIPE, check=True)
        dt = time.perf_counter() - t0
        out = np.frombuffer(res.stdout, dtype=np.float32).reshape(-1, ref.shape[1])
        assert out.shape == ref.shape, (out.shape, ref.shape)
        err = np.abs(out - ref)
        print(f"samples {len(obs)}  max|err| {err.max():.3e}  mean|err| {err.mean():.3e}  ref |act| max {np.abs(ref).max():.3f}")
        print(f"C forward wall time incl. pipe: {dt / len(obs) * 1e3:.3f} ms/sample (host, {args.opt})")
        # zero obs sanity (idle stand)
        print("zero-obs action:", np.array2string(out[-1], precision=3, suppress_small=True))
        ok = err.max() < 1e-4
        print("PARITY", "OK" if ok else "FAIL")
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
