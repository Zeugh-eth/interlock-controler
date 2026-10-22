"""Pure guards for arm-daemon: servo names, units, limits, table clearance, kinematics info,
and the MotionResult every write returns. Depends on scripts/kin.py and scripts/arm-go.py only."""
import importlib.util
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.append(str(SCRIPTS))   # append: scripts/control.py must not shadow our modules
import kin  # noqa: E402

kin.LIMITS = kin._limits(margin=0.8)          # what arm-go and the dashboard use (pan +-118.8)
_spec = importlib.util.spec_from_file_location("arm_go", SCRIPTS / "arm-go.py")
arm_go = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(arm_go)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ALIASES = {"pan": "shoulder_pan", "lift": "shoulder_lift", "shoulder": "shoulder_lift", "elbow": "elbow_flex",
           "wrist": "wrist_flex", "roll": "wrist_roll", "grip": "gripper"}
ALIASES.update({str(i + 1): j for i, j in enumerate(JOINTS)})
LIMITS = {**kin.LIMITS, "gripper": (0.0, 95.0)}
TABLE_Z = 0.015
HUBS = {"ready", "straight", "rest", "look-down"}
LINKS = ("lower_arm_link", "wrist_link", "gripper_link", "gripper_frame_link")


def resolve_servo(name: str):
    key = str(name).strip().lower()
    if key in JOINTS:
        return key
    return ALIASES.get(key)


def parse_target(servo: str, body: dict):
    unit = "pct" if servo == "gripper" else "deg"
    other = "deg" if unit == "pct" else "pct"
    if other in body and unit not in body:
        return None, f"wrong unit: {servo} takes {unit}"
    try:
        v = float(body[unit])
    except (KeyError, TypeError, ValueError):
        return None, "value out of range"
    if math.isnan(v) or math.isinf(v):
        return None, "value out of range"
    return v, ""


def clamp(q: dict) -> dict:
    out = dict(q)
    for j, v in q.items():
        if j in LIMITS:
            lo, hi = LIMITS[j]
            out[j] = min(max(float(v), lo), hi)
    return out


def _chain(q):
    return {j: float(q[j]) for j in kin.CHAIN}


def table_min_z(cur: dict, tgt: dict) -> float:
    return float(arm_go.path_min_z(_chain(cur), _chain(tgt)))


def kinematics_info() -> dict:
    """The URDF chain the dashboard needs to draw the arm with its own FK: joint origins (xyz, rpy)
    and axes straight from the URDF, plus the LeRobot-degrees -> URDF-radians offsets kin.py uses."""
    import xml.etree.ElementTree as ET
    root = ET.parse(kin.URDF).getroot()
    raw = {}
    for j in root.findall("joint"):
        o, ax, lim = j.find("origin"), j.find("axis"), j.find("limit")
        raw[j.find("child").get("link")] = {
            "link": j.find("child").get("link"), "parent": j.find("parent").get("link"),
            "joint": j.get("name"), "type": j.get("type"),
            "xyz": [float(v) for v in o.get("xyz").split()], "rpy": [float(v) for v in o.get("rpy").split()],
            "axis": [float(v) for v in ax.get("xyz").split()] if ax is not None else [0.0, 0.0, 0.0],
            "lower_rad": float(lim.get("lower")) if lim is not None else None,
            "upper_rad": float(lim.get("upper")) if lim is not None else None}
    visuals = {}
    for link in root.findall("link"):
        vis = []
        for v in link.findall("visual"):
            o, m = v.find("origin"), v.find("geometry/mesh")
            if m is None:
                continue
            vis.append({"mesh": m.get("filename").split("/")[-1],
                        "xyz": [float(x) for x in (o.get("xyz").split() if o is not None else "0 0 0".split())],
                        "rpy": [float(x) for x in (o.get("rpy").split() if o is not None else "0 0 0".split())]})
        if vis:
            visuals[link.get("name")] = vis
    return {"chain": [raw[l] for l in kin.ORDER], "visuals": visuals,
            "jaw": raw["moving_jaw_so101_v1_link"],
            "offset_deg": {j: round(float(v), 4) for j, v in kin.OFFSET_DEG.items()},
            "sign": {j: float(v) for j, v in kin.SIGN.items()},
            "limits_deg": {j: [round(v, 1) for v in LIMITS[j]] for j in JOINTS},
            "table_z_m": TABLE_Z, "links": list(LINKS)}


def tool_info(q: dict) -> dict:
    f = kin.fk(_chain(q))
    T = f["gripper_frame_link"]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(T[2, 2])))))
    return {"xyz_m": [round(float(v), 4) for v in T[:3, 3]],
            "tilt_from_down_deg": round(tilt, 1),
            "lowest_link_z_m": round(min(float(f[k][2, 3]) for k in LINKS), 4)}


@dataclass
class MotionResult:
    id: int
    outcome: str            # ok | frozen | refused
    reason: str
    requested: dict
    clamped: dict
    table_min_z_m: float | None
    table_ignored: bool
    final: dict
    peak_load: int
    duration_s: float

    def to_dict(self) -> dict:
        return asdict(self)
