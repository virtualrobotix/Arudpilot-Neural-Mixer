#!/usr/bin/env bash
# MuJoCo plant (JSON physics backend) — run in its own terminal before/alongside SITL
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec "$ROOT/.venv/bin/python" "$ROOT/plant/mujoco_json_plant.py" "$@"
