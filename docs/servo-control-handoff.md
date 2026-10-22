# Per-servo control interface — handoff (22 Oct 2026)

## State at handoff
- Nothing from this work is running or registered. Port 8790 is free; the `servos` MCP entry was removed.
- The arm's serial port (`/dev/cu.usbmodem5B790812561`) is held only by the **arm-agent runtime, pid 42060**
  (`run_runtime.py --backend real --i-own-the-bus`), not started by this work. Left untouched.
- No real move was ever made through this interface: every move test was refused (409) because pid 42060 owned the bus.

## What exists (files only)
1. `scripts/servo-server.py` — HTTP server on 127.0.0.1:8790, stdlib only, runs in the `tools/arm-lerobot` venv.
   - `POST /servo/<name>` `{"value": x}` → move that one servo (degrees; gripper % open 0–95)
   - `GET /servo/<name>`, `GET /servos` (values, torque, ranges, owner), `POST /torque_off`
   - `<name>`: joint name, bus id 1–6, or alias pan/lift/shoulder/elbow/wrist/roll/grip
   - All moves go through `scripts/control.py:set_joint` (range clamp, 1.5 cm table-clearance refusal,
     load/lag freeze, refuses with 409 while another process owns the port). Connects per command; never holds the bus idle.
2. `arm-agent/servo_mcp.py` — stdio MCP server `servos` for an agent that closes the loop with its own camera.
   Tools: `read_servos`, `set_shoulder_pan`, `set_shoulder_lift`, `set_elbow_flex`, `set_wrist_flex`,
   `set_wrist_roll`, `set_gripper` (absolute value each). Thin HTTP client to :8790; auto-starts servo-server
   if not answering (log: `lab/arm-vision/live/servo-server.log`). Uses a no-proxy opener (shell sets HTTP(S)_PROXY).

Tested: tool list over stdio, `read_servos` live values, 409 refusal while owned, 404 on unknown servo.
Not tested: an actual move.

Note: `arm-agent/CLAUDE.md` says `scripts/` is read-only for arm-agent dev; `servo-server.py` was put there on direct user request.

## To bring it back
1. Free the bus: `arm-agent/STOP-ARM.command` (stops the arm-agent runtime).
2. Register: `claude mcp add servos -s local -- uv --directory /Users/zeugh/Documents/ais/hackaiwoo/arm-agent run python servo_mcp.py`
3. New session → `read_servos` (should show no `owner`) → one small step, e.g. elbow −56.8 → −50, check camera.
