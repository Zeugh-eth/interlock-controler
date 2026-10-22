# arm-daemon: single bus owner with a client API (design)

Date: 2026-10-23. Status: approved in conversation; builder check done 2026-10-23.

## Intent

One long-lived process owns the SO-101 serial bus and the camera feeds, and exposes them over a
small documented API so that a human dashboard, an agent MCP, and the existing `arm-agent`
runtime can all be built on top of it as clients. Today every tool (scripts, arm-agent runtime,
servo-server) opens the serial port itself, so only one of them can run, they fight over the port
("Port is in use"), and a watchdog respawning the runtime stole the bus from a working session.

The daemon sits at the bottom. `arm-agent` (scan/select/pick/place skills and its MCP) becomes a
client. Whether arm-agent remains the main agent daemon is decided later.

## Goals

- Exactly one process holds `/dev/cu.usbmodem5B790812561` for its whole life.
- Live state: every servo polled continuously; latest camera frame per camera kept in memory.
- Reads are open to any client at any time.
- Writes change any servo in either direction, absolute or relative, within what the servo can do.
- A human/agent control token, switchable only by the human side.
- Every command returns a clear outcome so a client never has to guess what happened.
- One `API.md` a builder can code a dashboard or MCP from without reading the daemon source.

## Non-goals

- No object detection, no scan/pick/place skills, no task memory. Those stay in arm-agent.
- No camera capture inside the daemon (macOS camera permission needs Terminal.app; a camera crash
  must not take the arm down). The daemon proxies the existing cam-server on :8765.
- No authentication beyond the human key. The daemon binds to 127.0.0.1 only.
- No trajectory planning or IK endpoints beyond forward kinematics for reporting.

## Anti-goals (failure even if it "works")

- An agent that can take control away from a human, or that starts in control.
- A guard that can be disabled by the agent side: clamp and load/lag are never optional.
- A client that must know the serial protocol, calibration, or raw ticks.
- A motion call that blocks the daemon's state loop, so the dashboard freezes during moves.

## Constraints

- Python 3.12 in the `tools/arm-lerobot` uv venv (LeRobot 0.6.1 pins it; 3.14 breaks draccus).
- Reuse `scripts/grasp.py` (bus object, `read`, `hold`, `clamp`, `move`, `move_watch`,
  `torque_on`) and `scripts/kin.py` (`fk`, `tool_pose`, `LIMITS`) and `scripts/arm-go.py`
  (`path_min_z`, pose library at `lab/arm-vision/pose-library.json`). Do not copy their logic.
- Shell sets `HTTP(S)_PROXY=127.0.0.1:3213`; local HTTP calls must use a no-proxy opener.
- Calibration id `hackaiwoo_follower`; joints in degrees, gripper in percent open 0 to 95.
- The daemon is started by the user from Terminal (`arm-daemon/start.command`), never by a Claude
  shell, and never by a watchdog. If it is not running, clients report that; nobody respawns it.
- Table-clearance model is wrong when the arm rests on something (reported -2.5 cm at wake-up on
  2026-10-23), which is why it is advisory.

## Decided design

### Process

- Folder `arm-daemon/` at repo root: `daemon.py`, `API.md`, `start.command`, `smoke.sh`.
- Binds `127.0.0.1:8800`. HTTP for commands and reads; one WebSocket path for live state.
- Two threads plus the server: a **bus thread** that owns the serial connection and runs a
  command queue, and a **camera thread** that polls cam-server `/health` and fetches
  `/<cam>/frame.jpg` for each camera at up to 10 Hz, keeping the newest JPEG and its age.
- Bus thread loop: pop one command if any and execute it to completion, otherwise read all
  servos (position, goal, load, torque enabled, temperature) at ~20 Hz. Reads and the WebSocket
  broadcast never wait on a motion: the state struct is updated between motion sub-steps.
- Torque is left as found on start and on exit (`disconnect(disable_torque=False)`). A SIGTERM
  finishes the current motion sub-step, then disconnects with torque on.

### State (what every read and every WebSocket message carries)

```
{
  "t": 1761180000.12,
  "bus": "ok" | "error:<text>",
  "control": {"holder": "human" | "agent", "since": t},
  "stop": {"set": bool, "reason": str, "by": "human"|"agent"|"file"},
  "servos": {
    "shoulder_pan": {"deg": -0.1, "goal_deg": 0.0, "load": 12, "torque": true, "temp_c": 38,
                      "min_deg": -118.8, "max_deg": 118.8},
    ... elbow_flex, wrist_flex, wrist_roll, shoulder_lift,
    "gripper": {"pct": 9.0, "goal_pct": 0.0, "load": 80, "torque": true, "temp_c": 41,
                 "min_pct": 0, "max_pct": 95}
  },
  "tool": {"xyz_m": [0.20, 0.02, 0.074], "tilt_from_down_deg": 4.4, "lowest_link_z_m": 0.06},
  "motion": {"id": 17, "state": "idle"|"moving"|"frozen", "result": <last result object>},
  "cameras": {"gripper": {"age_s": 0.03, "ok": true}, "mac": {...}, "phone": {...}}
}
```

### Control token

- `holder` starts as `human` on every daemon start.
- `POST /control {"holder": "agent"|"human"}` requires header `X-Arm-Human-Key: <key>`. The key
  is a random string written to `arm-daemon/human.key` on first start (mode 600); the dashboard
  reads it from disk, an agent never does. Any request with that header counts as human.
- A write without the human header is an agent write. It is refused with
  `{"refused": "human has control"}` while holder is `human`.
- Switching holder to `human` while a motion is in progress freezes it in place (torque on) and
  the motion result is `frozen` with reason `control taken by human`.
- A human write while holder is `agent` is allowed and does not change the holder.

### STOP

- `POST /stop {"reason": str}` by anyone sets it; freezes any motion; all writes refused with
  `{"refused": "stop is set"}` until `POST /resume` with the human header.
- The file `arm-agent/STOP` is honoured too (checked each loop, reported as `by: file`) so the
  existing `STOP-ARM.command` keeps working; `resume` deletes it.

### Writes (all need control; all return a motion result)

| Endpoint | Body | Meaning |
|---|---|---|
| `POST /servo/<name>` | `{"deg": x}` or `{"pct": x}` for gripper, optional `speed`, `ignore_table` | absolute |
| `POST /servo/<name>/nudge` | `{"delta": d, ...}` | relative, clamped to ±15 per call |
| `POST /move` | `{"joints": {name: value, ...}, "speed", "ignore_table"}` | several joints, one watched move |
| `POST /gripper` | `{"pct": x}` | alias of servo gripper |
| `POST /pose` | `{"name": "ready"}` | named pose via the library's routing (arm-go.route); honours the library's `hold_after`: reaching `rest` releases torque, for any holder, because its lowest link is 3.4 cm (same rule as `/torque`) |
| `POST /torque` | `{"on": bool}` | torque off makes the arm limp; refused when `tool.lowest_link_z_m > 0.05` unless `"confirm": true` (the metric is the lowest link, not the tool point: at `rest` the links lie at 3.4 cm while the tool points up at 11.8 cm) |

`<name>` accepts the joint name, bus id 1 to 6, or alias pan/lift/elbow/wrist/roll/grip.

Motion result object (returned by every write and kept in `state.motion.result`):
```
{"id": 18, "outcome": "ok" | "frozen" | "refused",
 "reason": "" | "load 470 on elbow_flex" | "lag 7.2 deg on shoulder_lift" | "human has control" |
           "stop is set" | "path within 0.8 cm of table" | "unknown servo wrist" | "value out of range",
 "requested": {...}, "clamped": {...}, "table_min_z_m": 0.008, "table_ignored": false,
 "final": {<servos deg/pct>}, "peak_load": 210, "duration_s": 1.4}
```

Guards, in order: unknown servo or non-numeric value → refused; clamp to limits (reported in
`clamped`, not a refusal); table check via `path_min_z(cur, target)` against 1.5 cm → refused
unless `ignore_table`; then `move_watch` with `load_max` 350 (650 for named-pose hub legs) and
`lag_max` 6 (10 for hub legs) → `frozen` on trip. Gripper moves skip the table check and use
plain `move`.

A `frozen` outcome does not latch: the guard re-arms on the next write, so a client may retry (each retry is guarded again). Only STOP and a human takeover need `/resume` or the key.

Joint limits use `kin._limits(margin=0.8)` as arm-go and control do (pan ±118.8), not grasp's default margin.

Writes are executed one at a time from the queue; a second write arriving while one runs is
refused with `busy` rather than queued, so a stalled client cannot stack up motions.

### Reads (no control needed)

| Endpoint | Returns |
|---|---|
| `GET /state` | the state object above |
| `GET /servo/<name>` | that servo's entry plus `limits` and `aliases` |
| `GET /poses` | names, aliases and joints from the pose library |
| `GET /camera/<cam>/frame.jpg` | newest JPEG, `X-Frame-Age` header; 503 if none |
| `GET /camera/<cam>/stream` | MJPEG multipart proxy |
| `GET /cameras` | per-camera health |
| `GET /health` | `{"ok": true, "bus": ..., "uptime_s": ...}` |
| `WS /ws` | state object at 20 Hz, plus every motion result as it lands |

### API.md

One page in `arm-daemon/API.md`: start/stop instructions, the control model in five lines, every
endpoint with one `curl` example and one example response, the full outcome/reason vocabulary,
the WebSocket message shape, and a "building a client" section (how an MCP maps tools to calls;
how a dashboard uses the key and the socket). `smoke.sh` runs health, state, a 2-degree nudge and
back, and a refused agent write, printing pass/fail.

## Left to the builder

- HTTP/WebSocket library choice within the venv (stdlib `http.server` plus a small WS lib, or
  `aiohttp`); keep the dependency footprint small and pinned in `tools/arm-lerobot/pyproject`.
- Exact poll rates if 20 Hz starves the serial bus during moves; state staleness must be reported
  in `t`.
- How `move_watch` is sub-stepped so the state loop updates mid-move (it currently runs to
  completion inside one call; wrap or copy its loop with a per-tick callback).
- Log format and location (`arm-daemon/logs/`), one line per command and per result.

## Migration notes (not part of this build)

- `scripts/servo-server.py` and `arm-agent/servo_mcp.py` are superseded; remove after the daemon
  lands.
- `arm-agent/run_runtime.py --backend real --i-own-the-bus` and `keep-arm-up.sh` must not be
  started while the daemon runs; a follow-up makes the runtime talk to :8800 instead.
- `scripts/control.py` (dashboard back end) becomes a thin client of :8800.
