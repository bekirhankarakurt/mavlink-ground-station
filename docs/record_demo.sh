#!/usr/bin/env bash
# Re-records docs/demo.cast and renders docs/demo.gif from a real mavsim/mavgcs run.
# Requires: asciinema, agg (https://github.com/asciinema/agg), a built tree in ./build.
set -euo pipefail
cd "$(dirname "$0")/.."

BIN="${BIN:-build}"
"$BIN/mavsim" --speed 3 --loss 0.02 --seed 7 --duration 40 >/dev/null 2>&1 &
SIM_PID=$!
trap 'kill $SIM_PID 2>/dev/null || true' EXIT
sleep 0.3

asciinema rec docs/demo.cast --overwrite --quiet --cols 78 --rows 24 --idle-time-limit 2 \
  --command "$BIN/mavgcs --ui dashboard --script mission:examples/mission_square.txt,arm,start,wait-disarmed"

agg --fps-cap 8 --speed 1.5 --font-size 16 --last-frame-duration 4 docs/demo.cast docs/demo.gif
ls -l docs/demo.gif
