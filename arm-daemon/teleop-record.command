#!/bin/zsh
# Teleop + recording (state, action, gripper frames) with arm-daemon running. Usage:
#   ./arm-daemon/teleop-record.command "pick up the carton"   [FPS=30] [MAX_STEP=15]
HERE="$(cd "$(dirname "$0")" && pwd)"
URL=${ARM_URL:-http://127.0.0.1:8800}; KEY=$(cat "$HERE/human.key" 2>/dev/null)
c() { curl -s --noproxy '*' -m 5 "$@"; }
if c "$URL/health" >/dev/null; then
  c -X POST -H "X-Arm-Human-Key: $KEY" -H 'Content-Type: application/json' -d '{"holder":"released"}' "$URL/control" >/dev/null
  for i in {1..50}; do lsof /dev/cu.usbmodem5B790812561 >/dev/null 2>&1 || break; sleep 0.1; done
  trap 'c -X POST -H "X-Arm-Human-Key: $KEY" -H "Content-Type: application/json" -d "{\"holder\":\"human\"}" "$URL/control" >/dev/null; echo "arm-daemon has the bus again"' EXIT
fi
if lsof /dev/cu.usbmodem5B790812561 >/dev/null 2>&1; then echo "follower port busy:"; lsof /dev/cu.usbmodem5B790812561; exit 1; fi
cd "$HERE/../tools/arm-lerobot" && uv run python -B ../../arm-daemon/teleop-record.py "${1:-teleop}" --fps ${FPS:-30} --max-step ${MAX_STEP:-15}
