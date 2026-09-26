#!/usr/bin/env bash
# ArduRover SITL + NNMixer task, JSON physics backend, MAVProxy console.
#   scripts/run_sitl.sh                  # MAVProxy console + map
#   scripts/run_sitl.sh --no-mavproxy    # headless (for scripts/hil_test.py)
# Extra args go to sim_vehicle.py.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# SITL's filesystem root is its working directory (ardupilot/ under sim_vehicle.py):
# mirror the microSD layout there for NNM_ROBOT / NNM_POLICY.
mkdir -p "$ROOT/ardupilot/APM/nnm"
if [[ -d "$ROOT/sitl/APM/nnm" ]]; then
  cp -R "$ROOT/sitl/APM/nnm/." "$ROOT/ardupilot/APM/nnm/"
fi
cd "$ROOT/ardupilot"
exec "$ROOT/.venv/bin/python" Tools/autotest/sim_vehicle.py -v Rover --model JSON:127.0.0.1 \
    --no-rebuild --add-param-file "$ROOT/sitl/nnmixer.parm" --speedup 1 "$@"
