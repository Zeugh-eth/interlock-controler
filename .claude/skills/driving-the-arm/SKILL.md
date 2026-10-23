---
name: driving-the-arm
description: Use when the user asks the HackAIWoo SO-101 arm to do something through the arm-daemon MCP tools (mcp__arm-daemon__*) - look, find or describe objects, grab, pick or place, take a posture, open or close the gripper - when the user says stop or wait while the arm is working, or when a tool result says refused, frozen, lost, not_aligned, touch, or any outcome other than held, aligned or placed.
---

# Driving the arm (arm-daemon MCP)

The `arm-daemon` MCP (`mcp__arm-daemon__*`) is the only way to move or see: the daemon on :8800 owns
the bus. (The older `arm` MCP from arm-agent/ runs a simulator unless its own runtime is up; do not
use it for the real arm.) **Decide from the gripper camera.** `look()` shows the RAW frame with the
jaw band drawn on it: an object is between the fingers when it crosses the jaw-tip row inside the
green band, just right of the fixed jaw. Every detection in `vision.detections` says `between_jaws`
and `where`.

## Before anything
1. `state()`: `control.holder` must be `agent` (otherwise ask the user to click **agent** on the
   dashboard at http://127.0.0.1:8800/), `stop.set` false, `bus` ok.
2. `find_demos("<task words>")`: the best success card gives the grasp z, open % and jaw stop %.
3. `demo_start("<the user's words, verbatim>")`. End with `demo_stop(reported_outcome, notes)` and
   say what you saw, not what you assumed.

## Space setup: after a daemon restart, a scene change, or anything you can't explain
Run this before the first grab, and again whenever the frame shows something you can't account for.
1. **Health**: `state()` twice. A frozen `t`, `temp_c: null` or a `bus` error means the daemon is
   wedged (23 Oct: a USB dropout froze the poll loop and the gripper's ffmpeg in cam-server.py).
   `gripper.ok:false` with a fresh daemon → cam-server `/health` frame age; a hung ffmpeg is restarted
   by killing it (cam-server respawns it). Torque comes back on with the first pose.
2. **Survey** from `pose("look-down")`, then from z 11-12 cm (the top-down maximum there) with
   `look(overlay=True)`. Note the surface, the light, every object, where the target is and how it stands.
3. **Something unexplained → test it before engaging.** Pick a test that tells the hypotheses apart:
   - Dark shape: move the camera 4 cm sideways. A shadow stays put in the frame (it travels with the
     arm); a real object shifts by the move (~75 px at z 11).
   - Size or height doubtful: `measure_height(label)`; if the detector drops out, judge the base edge
     in the raw frame or `project(u, v)` it to the table.
   - Do not engage until one explanation is left. If none is, describe the frame and ask.
4. **Pick the alignment method from what the light allows**, checked on two or three frames:
   - Detector steady at ≥ 0.15 → `align_to(label)`.
   - Detector flickering (strong side light, 23 Oct: 0.06-0.70 frame to frame) or hard shadow
     edges → do not use the `u, v` blob tracker (it locks onto shadow edges and the white fixed jaw
     and walks the arm away). Use the manual servo instead: move the tool until the object crosses the
     jaw row 40-70 px right of the fixed tip, open to 60 %, descend in 1.5 cm steps with `look()`
     after each, `grasp(label)` at tool z ≈ 2.2 cm.
5. Write what you found into the demo notes (`demo_start("space setup ...")`) so the next run starts
   from it.

## Grab or pick: the decision tree
Start from `pose("look-down")` (tool z 8 cm) and `look()`.

- **A. The object is not in the frame** → `analyze_object(label)`. It surveys as high as is
  reachable above the current spot, scans ±15° and ±30° around the current pan and restores the
  pan when it finds nothing. A card with `plan.feasible` → `grab(plan)`. No card → describe what is
  in the frame and ask; do not wander.
- **B. You can see the object but the detector does not list it** (no detection with conf ≥ 0.15,
  or only one big merged box) → `align_to(label=..., u=..., v=...)` with a RAW-frame pixel on the
  object's body (give both label and pixel when unsure) and `width_cm` = the side the jaws will
  close across. On `aligned` → `grasp(label=...)`. `grab_at(...)` does both in one call.
- **C. The detector lists it with conf ≥ 0.15** → `align_to(label)` then `grasp(label)`, or
  `analyze_object(label)` → `grab(plan)` when the whole object is in frame from 8 cm or higher.
- **Place** → `move_tool` above the spot, lower to the pick height, `release()`. The frame after the
  retreat is the proof.
- End in `pose("standby")` unless the user asked for `rest`.

## Outcomes
| Result | Do |
|---|---|
| `aligned` | `grasp(label)` |
| `not_aligned` | read `final.where`; one more `align_to`; then ask |
| `lost` | `look()`; the object left the view: point at it again (u, v) and retry once |
| `touch` / `frozen` | `look()`; something was hit, say what; never repeat the same move |
| `held` | report; keep holding unless told to place |
| `empty` / `slipped` | `release()`; re-align once; then ask |
| `edge` | `replan` or shift along the closing axis; never carry an edge pinch |
| `refused` | read `reason`: holder, stop, busy, or unreachable. Unreachable names the highest reachable z: go lower. Never pass `ignore_table` unless the user agrees |

**Budget.** An attempt is a jaw close: three closes, then stop and ask. Exploration: twelve moves
without the object getting closer to the band → stop and report what you saw.

## Facts (every success kept these; every miss broke one)
- The fixed jaw is the LEFT finger in the raw view (base side). The moving jaw is the right one and
  is out of frame when the opening is 46 % or more.
- Raw axes: image-left = toward the base, image-top = world left. Object left of the band →
  `move_tool(dx<0)` (tool toward the base). Object above the jaw row → `move_tool(dy>0)` (tool left).
  These hold near pan 0. The camera turns with the pan, so off-centre (e.g. pan −25°) image-left is
  *radially* toward the base: split the correction along the arm's direction, not base x/y.
- **Frames lag ~0.5 s even when `stale: false`.** Decide from an observe pair: wait 0.6 s,
  `observe()`, wait 0.4 s, `observe()`; act only when both frames agree.
- Open the jaws (55 % for a 3 cm side, `open_pct` in `align_to`) BEFORE sliding sideways or descending.
- Keep tool z at or below 9 cm when x is under 18 cm: nearer the base a higher top-down pose is
  unreachable. 8 cm works everywhere the arm works.
- An object standing on something reads 1.5 to 2 times too large and too far in the overlay numbers.
  `measure_height(label)` or `look(height_cm=...)` fixes them; `align_to` does not need them.
- The detector often misses small objects above 6 cm. No detection is not evidence of absence; your
  eyes on the raw frame are evidence.
- The jaw percent alone cannot tell a thin hold (1.5 %) from a miss (1 %): `grasp(label)` checks
  with vision after the lift when the detector can see the object. When it cannot, the verdict can
  be wrong both ways (23 Oct: "empty" at 2.9 % on a held carton; "held" on a box that slid out, the
  opening shrinking during the lift). Confirm with a 3 cm lift and an observe pair: held means the
  object stays put in the frame while the table recedes.
- The table guard keeps the tool at ≥1.5 cm, so an object needs to stand clearly taller than that to
  be gripped from above. A box lying flat (~1 cm) can't be; ask the user to stand it on edge.
- Send any big move (a raise, a carry) alone and read its outcome before the next call: calls
  batched after it still run even when it comes back `frozen`. Carrying a load, raise at speed ≤25.
- Descend in 1.5 cm steps and look between steps: parallax moves the picture as you get low.
- Postures: compact/standby = `crouch-tight`, home = `ready`, `straight`, sleep/rest = `rest`
  (torque off), `look-down`. Other shapes: the arm-posture skill explains the words; its scripts open
  the bus, so propose them to the user instead of running them while the daemon is up.

## STOP
- User says stop / wait / no / careful → `stop()` first, then talk. A tool call blocks until its
  motion ends; to stop DURING a motion the user clicks stop on the dashboard.
- After a stop or a `frozen` result: `state()`, report the pose. Only a human clears STOP on the
  dashboard; `resume()` only reports the refusal.

## Never
- Run arm-moving `scripts/*.py`, open `/dev/cu.usbmodem*`, or start any runtime while the daemon is up.
- Close the jaws on an object that is not between them in the raw view.
- Fake a pick with open-loop `move_tool` + `gripper` calls when `align_to` says `not_aligned`.
- Plan from a detection below 0.15 confidence, or from the overlay cross alone (it was 4 cm off on 23 Oct).
