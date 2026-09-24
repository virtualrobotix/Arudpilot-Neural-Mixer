#!/usr/bin/env bash
# ArduRover SITL + MicroDuck task, JSON physics backend, MAVProxy console.
#   scripts/run_sitl.sh                  # MAVProxy console + map
#   scripts/run_sitl.sh --no-mavproxy    # headless (for scripts/hil_test.py)
# Extra args go to sim_vehicle.py.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/ardupilot"
exec "$ROOT/.venv/bin/python" Tools/autotest/sim_vehicle.py -v Rover --model JSON:127.0.0.1 \
    --no-rebuild --add-param-file "$ROOT/sitl/microduck.parm" --speedup 1 "$@"
