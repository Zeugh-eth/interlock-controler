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
