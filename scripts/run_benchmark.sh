#!/bin/bash
# Run a benchmark from any working directory, forwarding all arguments.
set -euo pipefail
robot_root="$(cd "$(dirname "$0")/.." && pwd)"
case "${1:-}" in
  vision_bakeoff|lfm_navigation_benchmark|lfm_memory_benchmark) benchmark="$1" ;;
  *) echo "Usage: $0 {vision_bakeoff|lfm_navigation_benchmark|lfm_memory_benchmark} [arguments]" >&2; exit 2 ;;
esac
shift
if [[ -n "${ROBOT_PYTHON:-}" ]]; then
  robot_python="$ROBOT_PYTHON"
elif [[ -x "$robot_root/.conda/bin/python" ]]; then
  robot_python="$robot_root/.conda/bin/python"
elif [[ -x "$robot_root/.venv/bin/python" ]]; then
  robot_python="$robot_root/.venv/bin/python"
else
  robot_python="python3"
fi
cd "$robot_root"
exec "$robot_python" "$robot_root/benchmarks/$benchmark.py" "$@"
