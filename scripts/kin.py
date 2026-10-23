"""SO-101 kinematics straight from vendor/so101/so101_new_calib.urdf (TheRobotStudio/SO-ARM100).

  fk(q)            q = dict of joint angles in DEGREES (LeRobot calibrated units) -> 4x4 per link
  tool_pose(q)     4x4 of gripper_frame_link (between the jaw tips) in base frame
  ik(...)          damped-least-squares IK for a tool position + tool pointing straight down

Joint sign/zero mapping between LeRobot degrees and URDF radians is in SIGN / OFFSET_DEG and is
verified on the real arm with the camera (see lab notes), not assumed.
"""
import math, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

URDF = Path(__file__).resolve().parents[1] / "vendor/so101/so101_new_calib.urdf"
CHAIN = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SIGN = {j: 1.0 for j in CHAIN}
# Every joint turns the URDF way. Verified from the MacBook side view (study2): shoulder_lift +
# tilts forward, elbow_flex + lowers the forearm, wrist_flex + lowers the gripper.
CAL_PATH = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so_follower/hackaiwoo_follower.json"


def _offsets():
    # URDF zero = the calibration "middle pose" (raw 2047 after homing). LeRobot degrees are
    # measured from the middle of the recorded range instead, so shift by that difference.
    import json
    cal = json.loads(CAL_PATH.read_text())
    return {j: ((cal[j]["range_min"] + cal[j]["range_max"]) / 2 - 2047) * 360 / 4095 for j in CHAIN}


OFFSET_DEG = _offsets()


def _limits(margin=6.0):
    import json
    cal = json.loads(CAL_PATH.read_text())
    out = {}
    for j in CHAIN:
        half = (cal[j]["range_max"] - cal[j]["range_min"]) / 2 * 360 / 4095
        out[j] = (-half + margin, half - margin)
    return out


LIMITS = _limits()


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, a):
    x, y, z = axis / np.linalg.norm(axis)
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _load():
    root = ET.parse(URDF).getroot()
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = np.array([float(v) for v in o.get("xyz").split()])
        rpy = [float(v) for v in o.get("rpy").split()]
        T = np.eye(4); T[:3, :3] = _rpy(*rpy); T[:3, 3] = xyz
        ax = j.find("axis")
        joints[j.find("child").get("link")] = {
            "name": j.get("name"), "parent": j.find("parent").get("link"), "T": T, "type": j.get("type"),
            "axis": np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None}
    return joints


JOINTS = _load()
ORDER = ["shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "gripper_frame_link"]


def to_rad(q):
    return {j: math.radians(SIGN[j] * (q.get(j, 0.0) + OFFSET_DEG[j])) for j in CHAIN}


def fk(q):
    r = to_rad(q)
    T = np.eye(4)
    out = {"base_link": T.copy()}
    for link in ORDER:
        jt = JOINTS[link]
        T = T @ jt["T"]
        if jt["type"] == "revolute":
            R = np.eye(4); R[:3, :3] = _axis_angle(jt["axis"], r[jt["name"]])
            T = T @ R
        out[link] = T.copy()
    return out


def tool_pose(q):
    return fk(q)["gripper_frame_link"]


def ik(target_xyz, q0, down=True, iters=300, tol=0.002, w_down=0.05):
    """Solve pan/lift/elbow/wrist_flex so the tool point reaches target_xyz (m, base frame) and,
    if down, the tool approach axis points straight down. Returns (q, pos_err_m)."""
    names = CHAIN[:4]
    q = {j: q0.get(j, 0.0) for j in CHAIN}
    lam = 1e-5
    for _ in range(iters):
        def resid(qq):
            T = tool_pose(qq)
            e = list(np.asarray(target_xyz) - T[:3, 3])
            if down:
                e += list(w_down * (np.array([0, 0, -1.0]) - T[:3, 2]))  # tool z axis -> world -z
            return np.array(e)
        e = resid(q)
        if np.linalg.norm(e[:3]) < tol and (not down or np.linalg.norm(e[3:]) < w_down * 0.05):
            break
        J = np.zeros((len(e), len(names)))
        for i, n in enumerate(names):
            dq = dict(q); dq[n] += 0.5
            J[:, i] = (e - resid(dq)) / 0.5
        step = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(len(e)), e)
        step = np.clip(step, -5, 5)
        for i, n in enumerate(names):
            lo, hi = LIMITS[n]
            q[n] = min(max(q[n] + step[i], lo), hi)
    return q, float(np.linalg.norm(np.asarray(target_xyz) - tool_pose(q)[:3, 3]))


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    print("offsets deg:", {k: round(v, 1) for k, v in OFFSET_DEG.items()})
    rest = {"shoulder_pan": -94.6, "shoulder_lift": -92.9, "elbow_flex": -85.3, "wrist_flex": 12.2, "wrist_roll": -52.2}
    print("rest pose tool xyz (m):", tool_pose(rest)[:3, 3], " wrist:", fk(rest)["wrist_link"][:3, 3])
    for xyz in ([0.22, 0, 0.15], [0.25, 0.1, 0.10], [0.20, -0.08, 0.05]):
        q, err = ik(xyz, {})
        print("ik", xyz, "->", {k: round(v, 1) for k, v in q.items()}, f"err={err*1000:.1f}mm tool_z={tool_pose(q)[:3, 2]}")
    for label, q in [("zero", {}), ("lift+30", {"shoulder_lift": 30}), ("elbow+30", {"elbow_flex": 30}),
                     ("wrist+30", {"wrist_flex": 30}), ("pan+30", {"shoulder_pan": 30})]:
        f = fk(q)
        pts = {k: f[k][:3, 3] for k in ["upper_arm_link", "lower_arm_link", "wrist_link", "gripper_frame_link"]}
        print(f"{label:9s}", "  ".join(f"{k.split('_')[0]}={v}" for k, v in pts.items()), " tool_z=", f["gripper_frame_link"][:3, 2])
