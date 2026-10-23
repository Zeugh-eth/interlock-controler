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
