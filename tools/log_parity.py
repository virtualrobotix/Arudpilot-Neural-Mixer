#!/usr/bin/env python3
"""Replay the observations AP_NNMixer logged (NNM/NNMQ/NNMV/NNMA) through the ONNX policy and
compare with the actions the firmware actually produced. Proves the C forward + observation
assembly in the firmware are the network the lab validated.

    python tools/log_parity.py sitl/run/logs/00000002.BIN policies/microduck_mlp_2048x2000_it1999.onnx
"""

from __future__ import annotations

import argparse
import glob
import os

import numpy as np
import onnxruntime as ort
from pymavlink import DFReader


def load_ticks(path: str):
    r = DFReader.DFReader_binary(path)
    mdk, q, v, a = {}, {}, {}, {}
    while True:
        m = r.recv_msg()
        if m is None:
            break
        t = m.get_type()
        if t == "NNM":
            mdk[m.TimeUS] = m
        elif t == "NNMQ":
            q[m.TimeUS] = [getattr(m, f"Q{i}") for i in range(14)]
        elif t == "NNMV":
            v[m.TimeUS] = [getattr(m, f"V{i}") for i in range(14)]
        elif t == "NNMA":
            a[m.TimeUS] = [getattr(m, f"A{i}") for i in range(14)]
    ticks = []
    for ts in sorted(mdk):
        if ts in q and ts in v and ts in a:
            ticks.append((ts, mdk[ts], np.array(q[ts]), np.array(v[ts]), np.array(a[ts])))
    return ticks


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?", default=None, help="dataflash .BIN (default: newest in sitl/run/logs)")
    ap.add_argument("onnx", nargs="?", default="policies/microduck_mlp_2048x2000_it1999.onnx")
    ap.add_argument("--act-max", type=float, default=2.0)
    args = ap.parse_args()
    log = args.log or sorted(glob.glob("sitl/run/logs/*.BIN"), key=os.path.getmtime)[-1]
    ticks = load_ticks(log)
    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    errs = []
    prev_act = None
    prev_ts = None
    n_armed = 0
    outliers = []
    for ts, m, q, v, a in ticks:
        if m.Fail != 0:
            prev_act = None
            continue
        # disarmed ticks are not logged: a gap in the 50 Hz tick train means the firmware
        # zeroed its action history in between
        if prev_ts is not None and ts - prev_ts > 30000:
            prev_act = None
        prev_ts = ts
        if prev_act is None:
            # first logged tick of an armed segment: the logger opens on arm and drops the
            # first few ticks, so the firmware's action history is unknown here — skip it
            prev_act = a.astype(np.float32)
            continue
        obs = np.zeros(61, np.float32)
        obs[0:3] = [m.GX, m.GY, m.GZ]
        obs[3:6] = [m.PGX, m.PGY, m.PGZ]
        obs[6:20] = q
        obs[20:34] = v
        obs[34:48] = prev_act
        obs[48:51] = [m.VX, m.VY, m.WZ]
        ref = sess.run(None, {in_name: obs[None]})[0][0]
        ref = np.clip(ref, -args.act_max, args.act_max)
        e = np.abs(ref - a).max()
        errs.append(e)
        if e > 1e-3:
            outliers.append((ts / 1e6, e))
        prev_act = a.astype(np.float32)
        n_armed += 1
    errs = np.array(errs)
    print(f"{log}: {n_armed} armed ticks replayed")
    print(f"max|onnx - firmware| = {errs.max():.3e}   p99 {np.percentile(errs, 99):.3e}   mean {errs.mean():.3e}")
    if outliers:
        print(f"{len(outliers)} tick(s) above 1e-3:", ", ".join(f"t={t:.2f}s err={e:.2e}" for t, e in outliers[:10]))
    print("PARITY", "OK" if errs.max() < 2e-3 else "FAIL")


if __name__ == "__main__":
    main()
