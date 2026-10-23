"""Go to a named pose from lab/arm-vision/pose-library.json, or save the current pose into it.

  list                       show poses with shape metrics
  go NAME [--yes] [--release]  route current -> (ready) -> via -> NAME with path + load checks.
                             Holds the pose with torque on (waiting for a command) unless the pose
                             says otherwise (rest) or --release is given.
  save NAME "description"    record the arm's current pose (pose it by hand with torque off first)
  off                        torque off where it is (support the arm first)

Names also match aliases ("compact", "standby", "sleep", "candle", ...).
Usage (from tools/arm-lerobot): uv run python ../../scripts/arm-go.py go standby
"""
import json, math, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kin
kin.LIMITS = kin._limits(margin=0.8)
import grasp, posture
from grasp import bus, read, hold

LIB = Path(__file__).resolve().parents[1] / "lab/arm-vision/pose-library.json"
FOLD_KEYS = ("shoulder_lift", "elbow_flex")


def load():
    return json.loads(LIB.read_text())


def resolve(name, poses):
    key = name.lower().strip()
    if key in poses:
        return key
    hits = [n for n, p in poses.items() if key in [a.lower() for a in p.get("aliases", [])]]
    if len(hits) == 1:
        return hits[0]
    raise SystemExit(f"unknown pose {name!r}" + (f"; ambiguous: {hits}" if hits else f"; known: {sorted(poses)}"))


def path_min_z(a, b, n=40):
    lo = 1.0
    for i in range(n + 1):
        s = i / n
        f = kin.fk({j: a[j] + (b[j] - a[j]) * s for j in kin.CHAIN})
        lo = min(lo, *(f[k][2, 3] for k in ("lower_arm_link", "wrist_link", "gripper_link", "gripper_frame_link")))
    return lo


def chain(n, poses):
    """n, its via, the via's via ... up to the root (ready)."""
    out = [n]
    while poses[out[-1]].get("via"):
        out.append(poses[out[-1]]["via"])
    return out


def nearest(cur, poses, tol=8.0):
    best = min(poses, key=lambda n: max(abs(cur[j] - poses[n]["joints"][j]) for j in kin.CHAIN))
    return best if max(abs(cur[j] - poses[best]["joints"][j]) for j in kin.CHAIN) <= tol else None


def route(cur, name, poses):
    """Poses form a tree through their 'via' (rest/look-down/straight -> ready, tucked -> straight).
    Go up from where the arm is to the shared ancestor, then down to the target, so a tucked pose is
    always unfolded the same way it was folded."""
    up = chain(name, poses)
    start = nearest(cur, poses)
    if start is None:
        steps = ["ready"] + list(reversed(up))[1:] if name != "ready" else ["ready"]
    else:
        down = chain(start, poses)
        common = next(n for n in down if n in up)
        steps = down[1:down.index(common) + 1] + list(reversed(up[:up.index(common)]))
    steps = [s for i, s in enumerate(steps) if i == 0 or s != steps[i - 1]] or [name]
    out, prev = [], dict(cur)
    for s in steps:
        q = {**prev, **poses[s]["joints"]}
        z = path_min_z(prev, q)
        out.append((s, q, z))
        prev = q
    return out


def cmd_list():
    poses = load()["poses"]
    for n, p in poses.items():
        m = p["metrics"]
        print(f"{n:15s} {p['shape']:18s} h={m['height_cm']:5.1f}cm r={m['footprint_radius_cm']:5.1f}cm "
              f"grip={m['gripper_points']:8s} aka {', '.join(p.get('aliases', [])[:4])}")


def cmd_go(name):
    lib = load(); poses = lib["poses"]
    name = resolve(name, poses)
    bus.connect()
    try:
        assert bus.is_calibrated, "calibration mismatch"
        cur = read()
        plan = route(cur, name, poses)
        for s, q, z in plan:
            print(f"plan: -> {s:14s} lowest point on the way {z*100:5.1f} cm")
            if z < 0.03 and s not in ("rest",):
                raise SystemExit(f"refusing: path to {s} comes within {z*100:.1f} cm of the table")
        if "--yes" not in sys.argv:
            input("ENTER = torque ON and go ")
        hold()
        for s, q, z in plan:
            # Hub legs carry the whole arm against gravity (lifting off the mat lags ~6 deg at 28 % load);
            # tucked poses pass close to the arm's own parts, so they get the tight contact limits.
            hub = s in ("ready", "straight", "rest", "look-down")
            ok, peak, why = grasp.move_watch(q, speed=25, load_max=650 if hub else 350, lag_max=10.0 if hub else 6.0)
            print(f"  {s}: {'ok' if ok else 'STOPPED ' + why} (peak load {max(peak.values())/10:.0f}%)")
            if not ok:
                print("Arm frozen where it stopped. Check it, then run 'off' or 'go ready'.")
                return
        m = read()
        print("arrived:", {k: round(float(v), 1) for k, v in m.items()})
        release = "--release" in sys.argv or not poses[name].get("hold_after", True)
        if release:
            bus.disable_torque(num_retry=3)
            print("torque OFF (resting)")
        else:
            print("holding with torque ON, waiting for a command")
    finally:
        bus.disconnect(disable_torque=False)


def cmd_save(name, desc):
    lib = load()
    bus.connect()
    try:
        q = {k: round(float(v), 1) for k, v in read().items()}
    finally:
        bus.disconnect(disable_torque=False)
    m = posture.metrics(q)
    lib["poses"][name] = {"joints": q, "shape": "taught", "description": desc, "aliases": [], "via": "ready",
                          "hold_after": True, "verified_on_arm": True, "evidence": None, "steady_load": None,
                          "metrics": {"footprint_radius_cm": round(m["radius_cm"], 1), "height_cm": round(m["height_cm"], 1),
                                      "lowest_point_cm": round(m["lowest_cm"], 1),
                                      "cylinder_volume_l": round(math.pi * m["radius_cm"] ** 2 * m["height_cm"] / 1000, 1),
                                      "gripper_points": "up" if m["tool_axis_z"] > 0.7 else "down" if m["tool_axis_z"] < -0.7 else "sideways"}}
    LIB.write_text(json.dumps(lib, indent=1))
    print("saved", name, q)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args or args[0] == "list":
        cmd_list()
    elif args[0] == "go":
        cmd_go(args[1])
    elif args[0] == "save":
        cmd_save(args[1], args[2] if len(args) > 2 else "")
    elif args[0] == "off":
        bus.connect(); bus.disable_torque(num_retry=3); bus.disconnect(disable_torque=False); print("torque OFF")
    else:
        raise SystemExit(__doc__)
