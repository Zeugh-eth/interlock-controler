---
name: driving-the-arm
description: Use when the user asks the HackAIWoo SO-101 arm to do something through the arm MCP tools (mcp__arm__*) - look, find or describe objects, scan the table, take a posture, open or close the gripper, pick or place - when the user says stop or wait while the arm is working, or when an arm tool result says refused, frozen, an outcome other than grasped/placed/locked, or shows a fixture or synthetic frame.
---

# Driving the arm (arm MCP)

The `arm` MCP server (arm-agent/) is the only way to move or see. **Decide from the gripper camera only.**

## Every mission
1. `status()`: `backend` (sim: say so in the reply), `stop.set`, `holding`, `skills` (can select/pick/place run?).
2. `begin_task("<the user's words, verbatim>")`.
3. See: `look()` here, `scan(area)` to search. Name things by `object_id`.
4. Act: `go_pose`, `gripper`, or `select(object_id)` → `pick(target_id)` → `select(place object)` → `place(target_id)`.
5. End in `go_pose("standby")` unless the user asked for `rest`. `scan` ends in `ready`, so this applies after it too.

## Reading images
- `camera_matches_pose: false` or `frame_source: fixture:…`: not a view from this pose. With a live
  camera and a sim arm, **every scan view is the same frame**, so its candidates are one scene seen N
  times. Describe it once and say the views do not follow the arm; never plan motion from it.
- `frame_source: synthetic`: a rendered sim scene, not the real table.
- `scan.objects` counts candidates per view. Overlapping boxes (`red box` + `pink/red object`) and the
  same thing in neighbouring views are usually one object: check the image.

## Skill outcomes
| Result | Do |
|---|---|
| `skills.pick` "needs a camera that follows the arm", or select/pick/place `refused` | say it cannot pick with this setup; **never fake a pick with go_pose + gripper** |
| `grasped` / `placed` (+ `on_target`) | report; continue the plan |
| `target_lost` | `look()`/`scan()` again, re-`select`, try once more |
| `missed` / `not_ready` (jaws stayed open: view did not match the grasp signature) | re-`select` and retry once, then ask |
| `touch` | `look()`, report what it hit; do not retry the same approach |
| `timeout` / `unreachable` | report with `error_trace_z_cm_ex_ey`; ask before another try |
| `refused: true` (motion) | report `error`; never retry a variant to get around it |

## STOP
- User says stop / wait / no / careful → `stop()` first, then talk.
- A tool call blocks until it returns: a "stop!" typed during `go_pose`, `scan` or `pick` reaches you
  only after that motion finished. To stop **during** a motion the user runs `touch arm-agent/STOP`.
- `stop()` writes `arm-agent/STOP`: it survives restarts and blocks every runtime until `resume()`.
  After a stop (or `frozen`): `status()`, report the pose, `resume()` only when the user says it is clear.

## Postures
compact/standby/waiting = `crouch-tight`, home/neutral = `ready`, straight/candle = `straight`,
sleep/rest = `rest` (torque off), look at the table = `look-down`. Other shapes: **REQUIRED SUB-SKILL:**
arm-posture for the meaning only; its `arm-go.py`/`arm-explore.py` runs open the bus, so propose them
to the user instead of running them while a runtime is up.

## Never
- Run arm-moving `scripts/*.py` or open `/dev/cu.usbmodem*` while a runtime is up.
- Start `--backend real` yourself; the user switches to hardware (arm-agent/README.md).
