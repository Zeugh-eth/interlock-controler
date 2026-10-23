# Guardrail loop — 23 Oct 2026

Three MCP-only cycles on the real SO-101 via arm-daemon (holder `agent`), no scripts:
look-down → find carton → pick → raise to ~25 cm → place back → standby → one blocked move → look-down.

| Cycle | Pick | Raise peak | Place | Blocked move | Guard result |
|---|---|---|---|---|---|
| 1 | `align_to` + `grasp`, held 4.5 % | 25.4 cm | standing at pick spot | `goto_xyz(0.20,-0.05,0.0)` touch table | refused: path within -0.0 cm of table |
| 2 | `grab_at`, held 4.5 %, eval good | 25 cm | standing at pick spot | `goto_xyz(0.55,0,0.08)` reach too far | refused: unreachable top-down |
| 3 | `lost` → `refused` → reset → `align_to` + `grasp`, held 5.5 % | 25.2 cm | standing at pick spot | `move_tool(dz -0.06)` from standby, through table | refused: path within -0.9 cm of table |

One jaw close per cycle, 3/3 picks held, 3/3 blocked moves refused with nothing moved.

## Data
- `session-2026-10-23.jsonl` — one line per step (cycle, step, outcome, args, reason).
- Daemon demos (rows + gripper images, labelled `success` with notes), in the main checkout's `arm-daemon/demos/`:
  - `20261023-084412-guardrail-loop-cycle-1-find-the-box-pick`
  - `20261023-084644-guardrail-loop-cycle-2-find-the-box-pick`
  - `20261023-084806-guardrail-loop-cycle-3-find-the-box-pick`

## Findings
- Top-down `goto_xyz` tops out at 11–12 cm near x 16–20 cm (IK limit, not a guard). "Raise high" needs a joint move
  (`move` lift -20, elbow -40 → 25 cm, gripper tilted 55°); the carton stayed held every time.
- The carton is invisible to the detector at look-down every cycle; pointing with a RAW pixel worked.
- Placements drifted the carton ~1 cm toward the base per cycle; by cycle 3 the first point lost it.
- Re-pointing from a half-aligned pose let the blob tracker latch onto empty mat. Reset to look-down first.

## Run 2 — cycles 4–28 (25 cycles)

Same loop, MCP only, daemon restarted before cycle 4. **25/25 picks held on one jaw close each; 25/25 blocked moves
refused with nothing moved.** Max servo temperature 40 °C. One `cycle` row per cycle in the jsonl (pick route, grasp
xyz, jaw %, raise z, place xyz, the blocked call and its refusal reason); demos `20261023-085633` … `20261023-092618`.

| Blocked kind | n | Example call | Refusal |
|---|---|---|---|
| table_touch | 3 | `goto_xyz(0.25,0.08,0.0)` | path within -0.0 cm of table |
| push_through_table | 2 | `move_tool(dz=-0.10)` from standby | path within -4.7 cm of table |
| inside_clearance_margin | 2 | `move_tool(dz=-0.045)` → 0.6 cm | path within 0.7 cm of table |
| below_table | 3 | `goto_xyz(0.20,-0.05,-0.05)` | path within -5.0 cm of table |
| diagonal_dive | 3 | `move_tool(0.10,0.05,-0.07)` | path within -1.9 cm of table |
| reach_far | 3 | `goto_xyz(0.45,-0.10,0.06)` | unreachable top-down |
| behind_base | 3 | `goto_xyz(-0.20,-0.05,0.10)` | unreachable top-down |
| far_side | 3 | `goto_xyz(0.0,-0.45,0.08)` | unreachable top-down |
| unknown_servo | 3 | `set_servo("wrist_pitch",30)` | unknown servo wrist_pitch |

Recipe that made it repeatable (cycles 5–28 used it unchanged):
1. Place at a fixed spot `(15.5, 1.5)` instead of the pick spot: no drift.
2. Find = `goto_xyz(0.135, 0.016, 0.08)` from look-down: the carton crosses the band there.
3. `align_to(label="carton", u=378, v=300, width_cm=3.5, open_pct=60)` → `grasp(label, lift_cm=3)`.
4. Raise: `goto_xyz(0.18, 0.015, 0.115)` + `move(lift -20, elbow -40)` → ~25 cm.

Only failure-and-recovery: cycle 4 (first after the daemon restart, carton at its old drifted spot near the
fixed jaw). The blob tracker locked onto the white fixed jaw and walked 12 moves off the mat (`not_aligned`, no close,
no contact). Fixed by repositioning until the detector saw the carton. The guard clearance margin is real: a path
ending 0.5–0.7 cm above the table is refused, not just a collision.

## Run 3: cycles 29-33, after a daemon restart (5 cycles)

**What happened first:** about an hour after cycle 28, a USB dropout froze the daemon's poll loop
("Port is in use!", state stuck) and hung the gripper's ffmpeg inside cam-server.py. Fix: SIGINT the
daemon, start `arm-daemon/start.command` in a Terminal tab, then kill the hung ffmpeg (cam-server
respawns it). The scene also changed: strong side light, a pink and dark look to the mat, and a large
arm shadow beside the carton. I first read the shadow as an object (glasses).

**Space setup run** (demo `20261023-103724`), now a section of the `driving-the-arm` skill:
- Shadow test: moved the camera 4 cm sideways. The dark loop stayed at the same image spot, so it
  is the arm's shadow; the carton shifted about 75 px, so it is fixed to the table.
- `measure_height` failed (the detector dropped below 0.15 at the second height).
- The detector flickered from 0.06 to 0.30, and to "bottle" with extra prompts. The `u,v` blob tracker
  drifted on shadow edges and walked the arm toward the base until refused (no close, no contact).
  `align_to(label)` lost the carton.
- Chosen method: manual visual servo, four 1.5 cm descents with a raw `look()` after each.

**Result: 5/5 held on one close each, 5/5 blocked moves refused, nothing moved.** Max servo temperature 36 °C.

| Cycle | Blocked call | Refusal |
|---|---|---|
| 29 | `goto_xyz(0.18,0.10,0.0)` | path within -0.0 cm of table |
| 30 | `goto_xyz(0.50,0.05,0.10)` | unreachable top-down |
| 31 | `move_tool(dz=-0.07)` from standby | path within -1.9 cm of table |
| 32 | `goto_xyz(0.16,-0.08,-0.04)` | path within -4.0 cm of table |
| 33 | `set_servo("gripper2",50)` | unknown servo gripper2 |

Cycle 29 picked the carton where it stood after the restart, about (19.5, 5.5). Cycles 30-33 used the
fixed drop spot and find pose, as in run 2.

**Totals so far: 33 cycles, 33 picks held, 33 blocked moves refused.**
