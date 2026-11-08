#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DURATION="${1:-30}"
OUTPUT="${2:-results/run_$(date +%Y%m%d_%H%M%S).csv}"

if ! python3 - "$DURATION" <<'PYVALID'
import math, sys
try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(value) and value > 0.0 else 1)
PYVALID
then
  echo "Usage: $0 [duration_seconds>0] [output.csv]" >&2
  exit 2
fi
if [[ "$OUTPUT" != /* ]]; then OUTPUT="$ROOT/$OUTPUT"; fi
mkdir -p "$(dirname "$OUTPUT")"
if [[ -e "$OUTPUT" ]]; then
  echo "Refusing to overwrite existing recording: $OUTPUT" >&2
  exit 2
fi

source /opt/ros/lyrical/setup.bash
LAUNCH_LOG="${OUTPUT%.csv}.launch.log"
"$ROOT/scripts/run_demo.sh" gui:=false auto_demo:=true auto_duration_s:="$DURATION" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  trap - EXIT INT TERM
  if kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  "$ROOT/scripts/stop.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

ready=0
for _ in $(seq 1 90); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "Launch exited before topics were ready. See $LAUNCH_LOG" >&2
    tail -n 80 "$LAUNCH_LOG" >&2 || true
    exit 1
  fi
  if ros2 topic list 2>/dev/null | grep -Fxq /odom; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" -ne 1 ]]; then
  echo "Timed out waiting for /odom. See $LAUNCH_LOG" >&2
  tail -n 80 "$LAUNCH_LOG" >&2 || true
  exit 1
fi

python3 "$ROOT/scripts/record_run.py" --duration "$DURATION" --output "$OUTPUT"
python3 "$ROOT/scripts/analyze_run.py" "$OUTPUT"
echo "Launch log: $LAUNCH_LOG"
