#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
COUNT="${1:-3}"
DURATION="${2:-30}"

if ! [[ "$COUNT" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [repeat_count>=1] [duration_seconds>0]" >&2
  exit 2
fi

for ((i=1; i<=COUNT; i++)); do
  OUTPUT="results/repeat_$(date +%Y%m%d_%H%M%S)_${i}.csv"
  echo "=== Run $i / $COUNT ==="
  "$ROOT/scripts/run_experiment.sh" "$DURATION" "$OUTPUT"
done
