#!/bin/zsh
# Finder launcher; camera access is attributed to Terminal.
exec "$(dirname "$0")/run_benchmark.sh" vision_bakeoff "$@"
