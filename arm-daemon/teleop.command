#!/bin/zsh
# Leader-arm teleop while arm-daemon is running: hands the bus to the leader (daemon disconnects,
# torque untouched), runs LeRobot teleop, and gives the bus back to the daemon when teleop exits.
# Ctrl-C stops teleop. Env: MAX_STEP (deg per step, default 15), FPS (default 30), ARM_URL.
HERE="$(cd "$(dirname "$0")" && pwd)"
URL=${ARM_URL:-http://127.0.0.1:8800}
KEY=$(cat "$HERE/human.key" 2>/dev/null)
FOLLOWER=/dev/cu.usbmodem5B790812561
LEADER=/dev/cu.usbmodem5B901010331
c() { curl -s --noproxy '*' -m 5 "$@"; }
if c "$URL/health" >/dev/null; then
  echo "arm-daemon: releasing the bus (holder=released) for LeRobot teleop"
  c -X POST -H "X-Arm-Human-Key: $KEY" -H 'Content-Type: application/json' -d '{"holder":"released"}' "$URL/control"; echo
  for i in {1..50}; do lsof "$FOLLOWER" >/dev/null 2>&1 || break; sleep 0.1; done
  trap 'echo; echo "arm-daemon: taking the bus back (holder=human)"; c -X POST -H "X-Arm-Human-Key: $KEY" -H "Content-Type: application/json" -d "{\"holder\":\"human\"}" "$URL/control"; echo' EXIT
else
  echo "arm-daemon not running; plain teleop"
fi
if lsof "$FOLLOWER" >/dev/null 2>&1; then echo "follower port still busy:"; lsof "$FOLLOWER"; exit 1; fi
cd "$HERE/../tools/arm-lerobot" && uv run lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=$FOLLOWER --robot.id=hackaiwoo_follower \
  --robot.max_relative_target=${MAX_STEP:-15} \
  --teleop.type=so101_leader --teleop.port=$LEADER --teleop.id=hackaiwoo_leader \
  --fps=${FPS:-30} --display_data=false
