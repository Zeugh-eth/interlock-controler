---
name: arm-posture
description: Use when asked to put the HackAIWoo SO-101 arm in a posture or shape described in words ("compact", "waiting", "straight", "sleep", "fold up", "look at the table", "make it smaller/taller", "like a swan"), when a requested posture is not in the pose library yet, or when deciding what a posture word means for this arm.
---

# Arm posture

## Overview
A posture word is a request for a **shape**, not for joint numbers. Map it to a verified pose in
`lab/arm-vision/pose-library.json`; if none fits, generate candidates from the model, prove them on
the real arm, then add the winner to the library. Never hand-type joint targets for a new shape.

## Vocabulary (this user's meaning)

| User says | Pose | Shape |
|---|---|---|
| compact, standby, waiting for command, folded (upright) | `crouch-tight` | bowed "?", 21 cm tall, 7 cm footprint, gripper down, held with torque |
| straight, candle, fully extended | `straight` | vertical column, 52.7 cm |
| sleep, rest, off, park, lie down | `rest` | laid back on the mat; torque released |
| neutral, home, L | `ready` | calibration middle pose; routing hub |
| look at the table, hover | `look-down` | gripper vertical 8 cm over the table |

"Fully folded" meant folded **upright from straight**, not lying on the table. Aliases are hints:
if the request adds a constraint the aliased pose breaks ("tall **with gripper down**"), it is a
new pose. When a word could mean two shapes, show the candidates' MacBook photos (`evidence`)
and let the user pick; then add the word to that pose's `aliases`.

## Run (from tools/arm-lerobot)

```bash
uv run python ../../scripts/arm-go.py list
uv run python ../../scripts/arm-go.py go standby          # ENTER gate; --yes to skip
uv run python ../../scripts/arm-go.py save my-pose "desc" # after posing by hand, torque off
```
`arm-go` routes through the pose tree (tucked → straight → ready → rest), checks every leg against
the table, and freezes on contact. Needs the camera server only for photos.

## New posture: generate → prove → save
1. Express the shape as a score + filters for `posture.search()` (metrics: footprint radius,
   height, lowest point, gripper up/down/side, self-clearance); it returns diverse candidates:
   `posture.search(lambda m: -m["height_cm"] + 0.6*m["radius_cm"], gripper="down", min_low=10)`.
2. Add them as `SETS["<name>"]` in `scripts/arm-explore.py`, start `scripts/cam-server.command`,
   run `arm-explore.py <run> --set <name>`: folds each from `straight` with load/lag watch, saves
   photos + loads to `lab/arm-vision/explore/<run>/`, then **ends in `rest` with torque off**.
3. Look at the photos. Model metrics alone mislead: by volume `straight` is "most compact".
4. Add the winner to the library with `via` (its parent in the tree), aliases and evidence, then
   `arm-go.py go <name>` to show it and `go standby` to return to waiting.

## Facts that constrain shapes
- Joint + directions: pan → arm's right; lift → forward; elbow → forearm down; wrist → gripper
  down. All follow the URDF. Do not flip any sign (see `lab/arm-vision/MODEL.md`).
- Forearm-down + gripper-up (a tight "N") is impossible: wrist reaches ±101° (keep ≤ 98).
  Gripper-down poses top out near 32 cm tall for the same reason.
- Present_Load is in 0.1 % units. Normal moves peak ≤ 32 %; holds ≤ 5 %. Contact limits: 35 % /
  6° lag near the arm's own parts, 65 % / 10° on hub legs (lifting off the mat lags ~6°).
- Starting a move while torque is on must not drop torque (`grasp.hold()` handles it).
- Background runs: stop with `touch lab/arm-vision/study/STOP`, never SIGINT (releases in place).

## Common mistakes
- Treating "fully folded" as `rest`, or "compact" as smallest volume.
- Going from `rest` straight to a tucked pose instead of through `ready` → `straight`.
- Releasing torque in any pose other than `rest` (the arm drops).
