# arm-daemon API

The one process that owns the SO-101 serial bus (`/dev/cu.usbmodem5B790812561`) and proxies the
camera server. Everything else (dashboard, MCP servers, arm-agent) is a client of this API.

- Base URL: `http://127.0.0.1:8800` (loopback only). Shell sets `HTTP(S)_PROXY`, so every local
  call needs `--noproxy '*'` (curl) or a `ProxyHandler({})` opener (Python).
- Units: joints in **degrees**, gripper in **percent open** `0..95`. Servo names:
  `shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper`, or aliases
  `pan lift|shoulder elbow wrist roll grip`, or bus ids `1..6`.

## Start / stop

- Start: double-click `arm-daemon/start.command` (Terminal). It refuses to start when another
  process holds the serial port and prints that pid. `start.command --fake` runs a dry daemon
  with a fake bus for client development.
- Stop: Ctrl-C in that Terminal. Torque is left as it was (the arm does not drop).
- **Never** start it from a Claude shell or a watchdog. If it is down, clients report that and
  a human restarts it. Before starting, nothing else may hold the port: no
  `run_runtime.py --backend real`, no `keep-arm-up.sh`, no `servo-server.py`, no arm scripts.
- Key: `arm-daemon/human.key` is created on first start (mode 600). Only the human dashboard reads it.
- Logs: `arm-daemon/logs/YYYYMMDD.jsonl`, one line per command and per result.

## Dashboard and MCP

- **Dashboard** (human): `http://127.0.0.1:8800/` served by the daemon with the human key injected.
  Live gripper stream, a 3D model of the arm (`static/arm3d.js`: the real STL meshes from `GET /meshes/`
  placed by the URDF visual origins in `GET /kinematics`, driven by the same FK as the daemon; each joint labelled with angle, load and
  temperature, servo colour = temperature, ring = load, blue ghost = goal pose while moving,
  dashed drop line = lowest link height), servo sliders and ±2° nudges, load bars, tool pose, holder switch
  (human / agent / leader / released), STOP and Resume, demo start/stop, named poses, torque, and
  a log of every motion result from every client (over `/ws`).
- **MCP** (agent): `arm-daemon/mcp_server.py`, registered as `arm-daemon` in `.mcp.json`. Tools:
  `state`, `look`, `poses`, `set_servo`, `nudge`, `move`, `gripper`, `pose`, `stop`, `resume`,
  `find_demos`, `demo_summary`, `list_demos`, `read_demo`, `replay_demo`, `demo_start`, `demo_stop`. It never sends the key, so
  its writes work only while the holder is `agent`. `uv run python mcp_server.py --list`.


## Agent skills (MCP only, the daemon stays plain)

`arm-daemon/skills.py` holds the agent's higher-level behaviour; the daemon never sees it. These
follow `reports/robot-ai-practices`: observations synchronized and age-checked, a stable base frame,
contact-first descent, and outcomes verified by evidence.

| Tool | What it does | Evidence it returns |
|---|---|---|
| `observe()` | state + gripper frame read together | `stale` (frame > 0.5 s old, missing, or arm moving), `frame_age_s`, joints, `tool_xyz_m` |
| `goto_xyz(x, y, z)` / `move_tool(dx, dy, dz)` | top-down IK offline, then `POST /move` | motion result + measured `tool_xyz_m`; `refused: unreachable ...` when IK fails |
| `project(u, v)` | raw-frame pixel -> base-frame point on the calibrated table plane | `x_m, y_m, z_m` |
| `descend_to_contact()` | lower in 0.5 cm steps until the load/lag guard freezes, back off 1 cm | `contact`, `z_contact_m`, guard reason |
| `grasp()` | close, read jaws, lift 2 cm, read again | `verdict` held / empty / slipped / unknown, both jaw readings, frame after the lift |
| `release()` | open, retreat 3 cm, observe | frame after the retreat |
| `demo_stop(reported_outcome, notes)` | the agent's own verdict stored in `meta.json` | `reported_outcome`, `notes` (distinct from any inferred `outcome`) |


### Grasp protocol tools (analyze -> grab -> evaluate -> replan)

Added after the 23 Oct lesson (four empty closes on a carton that had never been looked at whole):

| Tool | Does | Returns |
|---|---|---|
| `analyze_object(label)` | rise to >= 10 cm, find the object (pans if needed), measure with the vision overlay | `card` (table_xy_m, size_cm, short/long side, long_axis, est_height_cm) and `plan` (roll_deg so the jaws close across the short side, open_pct, approach_z_m, grasp_z_m, feasible + reason) |
| `grab(plan)` | approach with the planned roll, align on vision offsets (<= 1 cm, <= 3 corrections), open, descend, close, lift | corrections, aligned_xy_m, grasp verdict + jaw readings, `evaluation`, frame |
| `evaluate_grasp(grasp, plan)` | compare the jaw stop to the opening expected for the planned width | quality good / edge / empty / slipped, expected_jaw_pct |
| `replan(plan, evaluation)` | empty -> lower and re-survey; edge -> shift along the closing axis; slipped -> lower, shorter lift; attempt > 3 -> infeasible | the next plan |

Constants in `skills.py`: `JAW_MAX_CM` 7.5, `SURVEY_Z` 0.11, `APPROACH_Z` 0.06, `MODEL_LOW_CM` 2.0
(the kinematic z reads about 2 cm below the real table at the working area), `GRIP_DEPTH_CM` 1.5.

A pick, the intended way (manual tools): `observe` -> `project` the target pixel -> `goto_xyz` 6 cm above it ->
`observe` and correct with `move_tool` -> `descend_to_contact` -> `move_tool(dz=-0.005)` as needed ->
`grasp` -> act on the verdict (`empty`: open, re-observe, retry once; `slipped`: stop and report) ->
`demo_stop(reported_outcome=...)`.


### Grab rules learned 23 Oct (skills.py)

`grab(plan)` returns `outcome: not_aligned` with the jaws untouched when the object is not within
1 cm of the jaw target after 3 corrections at approach height or 2 at grasp height (it backs up to
approach height). It descends in 1.5 cm steps re-checking the offset, reports `measured_gain`
(object cm moved per commanded cm, from the first correction), `low_corrections`,
`aligned_at_grasp_height` and `descent_steps`. `analyze_object` returns `card.full_view` and no
plan when the object's box touches the frame edge. `grasp(label=...)` lets the vision overlay
decide held/empty when the jaws read fully closed. Skill cards carry `attempts` (every jaw close
with z, xy, settle %, lifted cm, held, descent speed, lateral motion, keyframe), `closes`,
`reported_outcome` and `notes`.

## Control model

1. `control.holder` is `human`, `agent`, `leader` or `released`. It starts as `human` every time the daemon starts.
2. A request is **human** when it carries header `X-Arm-Human-Key: <contents of human.key>`.
   Any other request is an **agent** request. A wrong key is simply an agent request.
3. Only a human request can change the holder (`POST /control`). While holder is `human`, agent
   writes get `409 {"outcome":"refused","reason":"human has control"}`. Human writes always work.
4. Switching holder to `human` while an agent move is running freezes it in place (torque on);
   that move returns `frozen` / `control taken by human`.
5. `POST /stop` works for anyone and freezes any motion; only a human `POST /resume` clears it.
   The file `arm-agent/STOP` is honoured too (`stop.by = "file"`); resume deletes it.
6. Holder `leader` is **teleop through the daemon**: the daemon opens the leader arm
   (`/dev/cu.usbmodem5B901010331`, calibration `hackaiwoo_leader`) and mirrors it onto the
   follower at 30 Hz with a 15°-per-tick step cap and the joint-limit clamp. State, tool pose,
   camera health and demo recording all keep working. Every write is refused with
   `leader has control`. STOP pauses mirroring. `state.leader` reports `{"configured", "active"}`;
   `bus` becomes `error:no leader arm configured` or `error:leader <text>` when the leader cannot
   be opened (the follower stays connected).
7. Holder `released` hands the serial bus to an **external** process (LeRobot's own
   `lerobot-teleoperate` / `lerobot-record`): the daemon freezes any move, disconnects (torque
   untouched), stops polling and reports `bus: "released"`; writes are refused with `bus released`.
   Setting the holder back reconnects. `arm-daemon/teleop.command` wraps LeRobot teleop this way.

Reads never need control. Writes run one at a time: a write arriving while one runs is refused
with `busy` (not queued).

## Guards on every write

| Guard | Behaviour |
|---|---|
| unknown servo / wrong unit / non-numeric | `refused`: `unknown servo <name>`, `wrong unit: gripper takes pct`, `wrong unit: <joint> takes deg`, `value out of range` |
| joint limits | clamped silently, reported in `clamped` (never a refusal) |
| table clearance (model, 1.5 cm) | `refused`: `path within <z> cm of table` unless the body has `"ignore_table": true` |
| load / lag (servo readings, always on) | `frozen`: `load <n> on <joint>` or `lag <deg> deg on <joint>`; the arm holds where it stopped |
| gripper moves | no table or load guard (it is meant to squeeze); STOP and takeover still apply |

A `frozen` result does not latch: the next write is allowed and guarded again.

## Motion result (returned by every write; also `state.motion.result`)

```json
{"id": 4, "outcome": "ok", "reason": "",
 "requested": {"elbow_flex": 2.0}, "clamped": {"elbow_flex": 2.0},
 "table_min_z_m": 0.2325, "table_ignored": false,
 "final": {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 2.0, "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 0.0},
 "peak_load": 50, "duration_s": 0.37}
```

`outcome` is `ok`, `frozen` or `refused`. HTTP status is 200 for `ok`/`frozen`, 409 for `refused`,
404 for an unknown servo or camera, 403 when a human key is required, 400 for bad JSON or an
unknown holder. `peak_load` is the highest absolute load seen on any arm joint (gripper excluded).

Full `reason` vocabulary: `busy`, `human has control`, `stop is set`, `control taken by human`,
`bus released`, `leader has control`, `path within <z> cm of table[ on the way to <leg>]`, `load <n> on <joint>`, `lag <d> deg on <joint>`,
`unknown servo <name>`, `unknown pose <name>: ...`, `unknown op <op>`, `wrong unit: ...`,
`value out of range`, `arm is raised; pass confirm`, `stopped on the way to <leg>: <reason>`,
`error: <exception>` (the daemon froze the arm after an unexpected error).

## Reads

| Endpoint | Returns |
|---|---|
| `GET /health` | `{"ok": true, "bus": "ok", "uptime_s": 17.6, "holder": "human", "stop": false}` |
| `GET /state` | the state object below |
| `GET /servo/<name>` | that servo's entry plus `name`, `limits`, `aliases` |
| `GET /poses` | `{"ready": {"joints": {...}, "aliases": [...], "via": null, "hold_after": true}, ...}` |
| `GET /kinematics` | the URDF chain for drawing the arm: `chain` (per link: `joint, parent, type, xyz, rpy, axis, lower_rad, upper_rad`), `jaw` (the moving-jaw joint), `offset_deg` and `sign` (LeRobot degrees → URDF radians, as `scripts/kin.py`), `limits_deg`, `table_z_m`, `links` |
| `GET /meshes/<file>` | the SO-101 STL meshes the URDF visuals reference (`vendor/so101/assets/`, from TheRobotStudio/SO-ARM100); `kinematics.visuals` maps each link to its meshes and visual origins |
| `GET /static/<file>` | vendored three.js (`three.module.min.js`, `three.core.min.js`, `OrbitControls.js`) and `arm3d.js` for the dashboard's 3D view |
| `GET /cameras` | `{"gripper": {"age_s": 0.056, "ok": true, "camera": "USB HD Camera"}, ...}` |
| `GET /camera/<cam>/frame.jpg` | newest JPEG, header `X-Frame-Age` (s); 503 `{"refused":"no frame"}` when the camera is down or stale |
| `GET /camera/<cam>/stream` | MJPEG, `multipart/x-mixed-replace; boundary=f` |
| `WS /ws` | see WebSocket |

Cameras: `gripper` (the only one that moves with the arm), `mac`, `phone`, `ipad`.

```bash
curl -s --noproxy '*' http://127.0.0.1:8800/state
```
```json
{"t": 1792693270.17, "bus": "ok",
 "control": {"holder": "human", "since": 1792693252.5},
 "stop": {"set": false, "reason": "", "by": ""},
 "servos": {
   "shoulder_pan": {"deg": 0.0, "goal_deg": 0.0, "load": 50, "torque": true, "temp_c": 35, "min_deg": -118.8, "max_deg": 118.8},
   "shoulder_lift": {...}, "elbow_flex": {...}, "wrist_flex": {...}, "wrist_roll": {...},
   "gripper": {"pct": 0.0, "goal_pct": 0.0, "load": 50, "torque": true, "temp_c": 35, "min_pct": 0.0, "max_pct": 95.0}},
 "tool": {"xyz_m": [0.2446, 0.0, 0.2451], "tilt_from_down_deg": 85.4, "lowest_link_z_m": 0.2325},
 "motion": {"id": 4, "state": "idle", "result": {...motion result...}},
 "cameras": {"gripper": {"age_s": 0.06, "ok": true, "camera": "USB HD Camera"}, "mac": {...}, "phone": {...}, "ipad": {...}}}
```

`bus` is `ok`, `starting`, `released` or `error:<text>`. `leader` is `{"configured": bool, "active": bool}`;
`demo` is `null` or `{"id", "task", "rows", "images"}` while a demonstration is recording. `motion.state` is `idle`, `moving` or `frozen`.
`t` is the time of the last servo read; during a move it refreshes every tick (50 Hz).
`temp_c` is `null` when the servo firmware does not report temperature.

## Writes

All need control (see above). Body is JSON. Optional fields on moves: `speed` (deg/s, default
30 for joints, 60 for the gripper, 40 for poses), `ignore_table` (bool).

| Endpoint | Body | Meaning |
|---|---|---|
| `POST /servo/<name>` | `{"deg": 12.5}` or `{"pct": 40}` for the gripper | absolute position |
| `POST /servo/<name>/nudge` | `{"delta": 2}` | relative, clipped to ±15 per call |
| `POST /move` | `{"joints": {"elbow": 10, "wrist": 60}}` | several joints in one watched move |
| `POST /gripper` | `{"pct": 40}` | alias of `POST /servo/gripper` |
| `POST /pose` | `{"name": "ready"}` | named pose from the library, routed through its `via` chain; `rest` releases torque on arrival |
| `POST /torque` | `{"on": false, "confirm": true}` | torque off makes the arm limp; refused with `arm is raised; pass confirm` when the lowest link is above 5 cm and `confirm` is missing |

```bash
K=$(cat arm-daemon/human.key)
curl -s --noproxy '*' -X POST -H "X-Arm-Human-Key: $K" -H 'Content-Type: application/json' \
     -d '{"delta": 2}' http://127.0.0.1:8800/servo/elbow/nudge
# -> {"id": 4, "outcome": "ok", ... "final": {..., "elbow_flex": 2.0, ...}, "duration_s": 0.37}

curl -s --noproxy '*' -X POST -H 'Content-Type: application/json' -d '{"deg": 5}' http://127.0.0.1:8800/servo/elbow
# -> 409 {"id": 5, "outcome": "refused", "reason": "human has control", ...}

curl -s --noproxy '*' -X POST -H "X-Arm-Human-Key: $K" -H 'Content-Type: application/json' \
     -d '{"joints": {"lift": 60, "wrist": 90}}' http://127.0.0.1:8800/move
# -> 409 {"outcome": "refused", "reason": "path within -4.1 cm of table", "table_min_z_m": -0.0408, ...}
```

## Vision (annotated gripper view, for humans and agents)

The daemon annotates the newest gripper frame with the current joints at `--vision-hz` (default 5):
rotated 90° so **up = forward (+x), left = +y**; a **1 cm grid** on the table plane (bold every 5 cm)
around the **jaw target** ✛ (where an object must appear to be between the fingers), a 5 cm scale
bar, the tool height, and **YOLO-World** boxes (`tools/models/yolov8s-worldv2.pt`, ~70 ms) labelled
with estimated **size** and **offset from the jaw target** in cm. All metric numbers assume the
object lies on the table plane of the hand-eye calibration (`lab/arm-vision/gripper-camera-cal.json`);
tall objects, or objects sitting on something, read larger and farther than they are: every
detection carries `assumes_table: true` and the labels say "(on table)". Without the table in
view (tilt > 60° from down) there is no grid.

| Endpoint | Returns |
|---|---|
| `GET /vision` | `{"t", "ms", "detector", "prompts", "tool": {"z_cm", "camera_z_cm", "looking_down", "px_per_cm"}, "grid": {"drawn", "spacing_cm", "px_per_cm", "origin_display_px"}, "jaw_target": {"raw_px", "display_px", "table_xy_cm"}, "detections": [{"label", "conf", "bbox_raw", "center_display_px", "size_cm", "offset_cm" (fwd, left), "table_xy_cm", "dist_cm"}], "frame_age_s", "error"?}` |
| `GET /vision/frame.jpg` | annotated JPEG (503 while there is no camera frame) |
| `GET /vision/stream` | MJPEG of the annotated view |
| `POST /vision/prompts` | `{"prompts": ["box", "cup"]}` or `"box, cup"`; empty = detector defaults. Anyone may set them. With explicit prompts every detection is shown; with defaults, detections under `min_conf` (0.15) are hidden (`hidden_low_conf` counts them). |
| `POST /vision/settings` | `{"grid_cm": 5 or 1, "min_conf": 0.15}`; returns the settings in force. Default grid is 5 cm lines. |

`state.vision` carries the detections, tool numbers, prompts and any error, so WebSocket clients
get them for free. The dashboard shows the annotated stream by default (toggle raw) with a
detection table; the MCP `look()` returns the annotated frame plus `vision`.

## Demonstrations (recording what a human or an agent did, for the agent to learn from)

A demo is one episode under `arm-daemon/demos/<stamp>-<task-slug>/`: `meta.json`
(`id, task, started, ended, rows, images, fps_teleop`), `frames.jsonl` (one row per motion tick,
from leader teleop **and** from guarded agent/human writes while the demo is open), and
`gripper/NNNNNN.jpg`. Row: `{"t", "holder", "state": {joint: deg|pct}, "action": {joint: goal},
"tool": {xyz_m, tilt_from_down_deg, lowest_link_z_m}, "image": "gripper/000012.jpg"}`; `image`
is present only on rows where the gripper frame changed (reuse the previous one otherwise).

| Endpoint | Body | Key | Returns |
|---|---|---|---|
| `POST /demo/start` | `{"task": "pick up the carton"}` | no | `{"id": "20261023-031500-pick-up-the-carton", "task": ...}`; 409 if one is recording |
| `POST /demo/stop` | none | no | `{"id", "task", "rows", "images", "dir"}`; 409 if none |
| `GET /demos` | | | list of `meta.json` objects, oldest first |
| `GET /demos?q=pick+box&limit=3` | | | keyword search over task, tags, phases, outcome; hits are skill cards (see below) ranked by score |
| `GET /demo/<id>/summary` | | | the skill card: `{id, task, outcome, holders, duration_s, rows, images, tags, key, phases}` (`?refresh=1` recomputes) |
| `GET /demo/<id>` | | | `{"meta": {...}, "rows": [...]}` |
| `GET /demo/<id>/image/gripper/000012.jpg` | | | that JPEG |
| `POST /demo/<id>/replay` | `{"every": 10, "speed": 30, "ignore_table": false}` | control | a motion result; waypoints are every N-th recorded action, each a guarded move. `reason` on `ok` is `"<n> waypoints"`, on a stop `stopped at waypoint k/n: <why>` |

**Skill card** (`summary.json` + `README.md` written next to every demo when it stops, also
`python demolib.py <demo dir>` for old ones): phases `approach, align, descend, grasp, lift,
carry, release, retreat` detected from gripper opening and tool height, each with row range,
times, start/end tool xyz and joints, and a keyframe image; `key` holds `approach_tool_z_cm`,
`gripper_open_pct`, `grasp_tool_z_cm`, `grasp_xy_cm`, `grasp_state` (joints at the grasp),
`gripper_closed_pct`, `lift_height_cm`, `carry_dx_cm/dy_cm`, `release_tool_z_cm`,
`release_xy_cm`; `outcome` is `grasped_and_released`, `grasped_and_held`, `closed_empty`
(gripper shut fully: nothing held), `never_closed` or `gripper_never_opened`; `tags` = task
words + synonyms + holders + phase names + `success`/`failed`.

Typical flow for teaching: `POST /demo/start` → `POST /control {"holder":"leader"}` (human key) →
drive with the leader arm → `POST /control {"holder":"human"}` → `POST /demo/stop`. The agent
later reads `GET /demos`, picks a matching task, studies its rows and images, and either replays
it or adapts it with its own servo commands.

## Control, stop, resume

| Endpoint | Body | Human key | Returns |
|---|---|---|---|
| `POST /control` | `{"holder": "agent"}`, `"human"`, `"leader"` or `"released"` | required | `{"holder": "agent", "since": 1792693270.6}`; 403 `{"refused": "human key required"}`; 400 `{"refused": "unknown holder"}` |
| `POST /stop` | `{"reason": "text"}` | no | `{"stop": {"set": true, "reason": "text", "by": "agent"}, "motion": {...}}` |
| `POST /resume` | none | required | `{"stop": {"set": false, "reason": "", "by": ""}}` |

## WebSocket `/ws`

On connect the daemon sends one `state` message, then a `state` message 20 times a second and a
`result` message for every finished write (from any client):

```json
{"type": "state", "t": ..., "bus": "ok", "control": {...}, "stop": {...}, "servos": {...}, "tool": {...}, "motion": {...}, "cameras": {...}}
{"type": "result", "id": 4, "outcome": "ok", "reason": "", ...}
```

The socket sends only; nothing a client sends on it is read. Use HTTP for commands.

## Building a client

**MCP for an agent** (no key): one tool per write endpoint (`set_servo`, `nudge_servo`, `move`,
`gripper`, `pose`, `stop`, `replay_demo`) plus `state`, `frame`, `list_demos`, `read_demo` reads. Pass every motion result through to
the model unchanged: `outcome` + `reason` is the whole story. On `human has control` tell the
user the human holds the arm; never retry in a loop. Check `cameras.gripper.ok` before trusting a
frame; `X-Frame-Age` says how old it is.

**Dashboard for the human**: read `arm-daemon/human.key` from disk and send it on every request;
open `/ws` for live joints, loads, tool pose and camera health; show `/camera/gripper/stream`;
offer a holder toggle (`POST /control`) and a big STOP (`POST /stop`) with a human-only Resume.
Taking control (`holder: human`) is the "take over from the agent" button: it also freezes the
current move.

**arm-agent runtime** (follow-up): replace its direct bus access with `POST /move` and `GET
/camera/gripper/frame.jpg`; it runs as an agent client and must handle `human has control`.

## Smoke test

`arm-daemon/smoke.sh` (optionally `ARM_URL=http://127.0.0.1:8801`) checks health, state, an
agent refusal, a human elbow nudge of +2° and back, and a camera frame. It prints `PASS`/`FAIL`
per line and exits non-zero on any failure.
