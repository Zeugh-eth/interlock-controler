"""Posture intelligence: describe a whole-arm pose by shape metrics and search for poses by shape.

  keypoints(q)     3D points along the arm (base frame, m) from the verified model
  metrics(q)       footprint radius, height, lowest point, self-clearance, gripper direction
  search(score, gripper=, min_low=, max_radius=, max_height=)  any shape, e.g.
                   search(lambda m: -m["height_cm"] + 0.6 * m["radius_cm"], gripper="down")  # tall bird
  search_compact() rank upright folded poses by how compact they are

Metrics are model-only; arm-explore.py checks the winners on the real arm (servo load and
MacBook photos) before a pose goes into the library.
"""
import itertools, math
import numpy as np
import kin

AXIS_XY = np.array([0.0388, 0.0])   # base pan axis in the URDF base frame
SHOULDER_TOP = 0.12                 # base + shoulder housing height (m), treated as a column


def keypoints(q):
    f = kin.fk({j: q.get(j, 0.0) for j in kin.CHAIN})
    p = {k: f[v][:3, 3] for k, v in {"shoulder": "upper_arm_link", "elbow": "lower_arm_link", "wrist": "wrist_link",
                                       "gripper_base": "gripper_link", "tip": "gripper_frame_link"}.items()}
    p["tool_axis"] = f["gripper_frame_link"][:3, 2]
    return p


def _seg_pts(a, b, n=12):
    return [a + (b - a) * s for s in np.linspace(0, 1, n)]


def _seg_dist(a, b, c, d):
    return min(np.linalg.norm(p - r) for p in _seg_pts(a, b) for r in _seg_pts(c, d))


def metrics(q):
    p = keypoints(q)
    pts = [p[k] for k in ("shoulder", "elbow", "wrist", "gripper_base", "tip")]
    radius = max(np.linalg.norm(x[:2] - AXIS_XY) for x in pts)
    height = max(x[2] for x in pts)
    low = min(x[2] for x in pts[1:])
    upper, fore, grip = (p["shoulder"], p["elbow"]), (p["elbow"], p["wrist"]), (p["wrist"], p["tip"])
    # Self-clearance proxies: gripper vs upper arm, wrist/gripper vs the base column.
    c_grip_upper = _seg_dist(*grip, *upper)
    col_top = np.array([AXIS_XY[0], AXIS_XY[1], SHOULDER_TOP])
    col_bot = np.array([AXIS_XY[0], AXIS_XY[1], 0.0])
    c_col = min(_seg_dist(*fore, col_bot, col_top), _seg_dist(*grip, col_bot, col_top))
    return {"radius_cm": radius * 100, "height_cm": height * 100, "lowest_cm": low * 100,
            "grip_to_upper_cm": c_grip_upper * 100, "to_base_column_cm": c_col * 100,
            "tool_axis_z": float(p["tool_axis"][2])}


def ok(m, min_low=3.0, min_clear=4.0, min_col=5.0):
    return m["lowest_cm"] >= min_low and m["grip_to_upper_cm"] >= min_clear and m["to_base_column_cm"] >= min_col


def search(score, gripper=None, min_low=5.0, max_radius=None, max_height=None, top=5, step=(5, 4, 6)):
    """Generic shape search. score(m) -> lower is better. gripper: 'up' | 'down' | 'side' | None.
    Returns diverse (score, q, m) candidates; wrist kept 3 deg inside its limit."""
    lo_e, hi_e = kin.LIMITS["elbow_flex"]; lo_w, hi_w = kin.LIMITS["wrist_flex"]
    rows = []
    for lift, elbow, wrist in itertools.product(range(-60, 41, step[0]), range(int(lo_e), int(hi_e) + 1, step[1]),
                                                range(int(lo_w) + 3, int(hi_w) - 2, step[2])):
        q = {"shoulder_pan": 0, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist, "wrist_roll": 0}
        m = metrics(q)
        z = m["tool_axis_z"]
        cls = "up" if z > 0.7 else "down" if z < -0.7 else "side"
        if gripper and cls != gripper:
            continue
        if not ok(m, min_low=min_low, min_clear=3.5, min_col=4.5):
            continue
        if (max_radius and m["radius_cm"] > max_radius) or (max_height and m["height_cm"] > max_height):
            continue
        rows.append((score(m), q, m))
    rows.sort(key=lambda r: r[0])
    chosen = []
    for r in rows:
        if all(sum(abs(r[1][j] - c[1][j]) for j in ("shoulder_lift", "elbow_flex", "wrist_flex")) > 25 for c in chosen):
            chosen.append(r)
        if len(chosen) == top:
            break
    return chosen


def search_compact(top=8):
    lo, hi = kin.LIMITS["elbow_flex"]
    out = []
    for lift, elbow, wrist in itertools.product(range(-40, 31, 5), range(40, int(hi) + 1, 4), range(-96, 97, 8)):
        q = {"shoulder_pan": 0, "shoulder_lift": lift, "elbow_flex": elbow, "wrist_flex": wrist, "wrist_roll": 0}
        m = metrics(q)
        if not ok(m):
            continue
        score = m["radius_cm"] + 0.35 * m["height_cm"]   # small footprint first, then low
        out.append((score, q, m))
    out.sort(key=lambda x: x[0])
    # keep diverse candidates: skip near-duplicates of an already chosen pose
    chosen = []
    for s, q, m in out:
        if all(sum(abs(q[j] - c[1][j]) for j in ("shoulder_lift", "elbow_flex", "wrist_flex")) > 25 for c in chosen):
            chosen.append((s, q, m))
        if len(chosen) == top:
            break
    return chosen


if __name__ == "__main__":
    straight = {"shoulder_pan": 0, "shoulder_lift": -2.0, "elbow_flex": -82.4, "wrist_flex": -4.0, "wrist_roll": 0}
    print("straight:", {k: round(v, 1) for k, v in metrics(straight).items()})
    for s, q, m in search_compact():
        print(f"score {s:5.1f}  lift {q['shoulder_lift']:+4d} elbow {q['elbow_flex']:+4d} wrist {q['wrist_flex']:+4d} | "
              + "  ".join(f"{k}={v:.1f}" for k, v in m.items()))
