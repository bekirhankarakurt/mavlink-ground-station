#!/usr/bin/env bash
# End-to-end check over real UDP sockets: mavsim <-> mavgcs flies a mission,
# records it, and the recording replays with the same frame count.
set -euo pipefail

BIN_DIR="$1"
SRC_DIR="$2"
WORK="$(mktemp -d)"
trap 'kill "${SIM_PID:-0}" 2>/dev/null || true; rm -rf "$WORK"' EXIT

GCS_PORT=14650
SIM_PORT=14680

"$BIN_DIR/mavsim" --port "$SIM_PORT" --target "127.0.0.1:$GCS_PORT" --speed 10 --duration 60 \
  >"$WORK/sim.log" 2>&1 &
SIM_PID=$!

"$BIN_DIR/mavgcs" --ui log --port "$GCS_PORT" --record "$WORK/run.tlog" --duration 50 \
  --script "mission:$SRC_DIR/examples/mission_square.txt,arm,start,wait-disarmed" \
  >"$WORK/gcs.log" 2>&1

grep -q "mission uploaded: 6 items" "$WORK/gcs.log"
grep -q "vehicle disarmed" "$WORK/gcs.log"
grep -q "Landed, disarmed" "$WORK/sim.log"

LIVE_FRAMES="$(sed -n 's/^summary: \([0-9]*\) frames.*/\1/p' "$WORK/gcs.log")"

"$BIN_DIR/mavgcs" --ui log --replay "$WORK/run.tlog" --speed 100 >"$WORK/replay.log" 2>&1
REPLAY_FRAMES="$(sed -n 's/^summary: \([0-9]*\) frames.*/\1/p' "$WORK/replay.log")"

[ "$LIVE_FRAMES" = "$REPLAY_FRAMES" ] || {
  echo "frame count mismatch: live=$LIVE_FRAMES replay=$REPLAY_FRAMES" >&2
  exit 1
}
echo "e2e ok: $LIVE_FRAMES frames recorded and replayed"
