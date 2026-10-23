#!/bin/zsh
# Smoke test against a running arm-daemon. Usage: ARM_URL=http://127.0.0.1:8800 arm-daemon/smoke.sh
set -u
cd "$(dirname "$0")"
URL=${ARM_URL:-http://127.0.0.1:8800}
KEY=$(cat human.key 2>/dev/null || true)
c() { curl -s --noproxy "*" -m 20 "$@"; }
fail=0
check() {  # name, expected-substring, actual
  if [[ "$3" == *"$2"* ]]; then echo "PASS $1"; else echo "FAIL $1: expected '$2' in: ${3:0:200}"; fail=1; fi
}
check health '"ok": true' "$(c $URL/health)"
check state  '"servos"'   "$(c $URL/state)"
check agent-refused '"human has control"' "$(c -X POST -H 'Content-Type: application/json' -d '{"delta": 2}' $URL/servo/elbow/nudge)"
check human-nudge-up   '"outcome": "ok"' "$(c -X POST -H "X-Arm-Human-Key: $KEY" -H 'Content-Type: application/json' -d '{"delta": 2}' $URL/servo/elbow/nudge)"
check human-nudge-down '"outcome": "ok"' "$(c -X POST -H "X-Arm-Human-Key: $KEY" -H 'Content-Type: application/json' -d '{"delta": -2}' $URL/servo/elbow/nudge)"
code=$(c -o /dev/null -w '%{http_code}' $URL/camera/gripper/frame.jpg)
if [[ "$code" == "200" || "$code" == "503" ]]; then echo "PASS camera ($code)"; else echo "FAIL camera: http $code"; fail=1; fi
exit $fail
