"""Camera-guided pick of the pink/red box with the SO-101 + gripper camera.

  look  [r z]   go to a top-down hover pose and save a camera frame (geometry check)
  teach         hover; you center the box under the open jaws (ENTER); it lowers until you
                ENTER at grasp height; closes, lifts, puts it back; saves lab/arm-vision/teach.json
  grab          search by turning the base, center the box under the gripper with camera feedback,
                lower, close, lift, verify with the camera, put it back down

Every run: starts from and returns to the pose the arm is in (normally folded rest), holds with
no jump, clamps all targets to the calibrated range, moves slowly. Ctrl-C freezes in place and
asks before torque-off. Needs scripts/cam-server.command running.

Usage (from tools/arm-lerobot): uv run python ../../scripts/grasp.py look
"""
import json, math, os, sys, threading, time
from pathlib import Path
import cv2
import numpy as np
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kin, vision

PORT = os.environ.get("ARM_FOLLOWER_PORT", "/dev/cu.usbmodem5B790812561")
OUT = Path(__file__).resolve().parents[1] / "lab/arm-vision"
TEACH = OUT / "teach.json"
HOVER_R, HOVER_Z = 0.20, 0.08      # m, tool point (between jaw tips) in base frame
SPEED = 40.0                       # deg/s for the fastest joint
HZ = 50
GRIP_OPEN = 90.0                   # % open while approaching
READY = {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 0, "wrist_flex": 0, "wrist_roll": 0}
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

cal = {k: MotorCalibration(**v) for k, v in json.loads(kin.CAL_PATH.read_text()).items()}
D = MotorNormMode.DEGREES
bus = FeetechMotorsBus(port=PORT, calibration=cal, motors={
    "shoulder_pan": Motor(1, "sts3215", D), "shoulder_lift": Motor(2, "sts3215", D),
    "elbow_flex": Motor(3, "sts3215", D), "wrist_flex": Motor(4, "sts3215", D),
    "wrist_roll": Motor(5, "sts3215", D), "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100)})


# ---------- arm primitives ----------
def read():
    return bus.sync_read("Present_Position")


def ensure(reg, motor, value):
    # Gains/limits live in EEPROM: write only when different (slow writes can miss a status
    # packet; one did on the elbow), and retry.
    if bus.read(reg, motor, normalize=False, num_retry=3) != value:
        bus.write(reg, motor, value, num_retry=3)


def hold():
    if torque_on():
        # Already holding (e.g. standby): never drop torque under a raised arm. Gains were set when
        # torque was first enabled; just re-anchor the goals to where the arm is.
        bus.sync_write("Goal_Position", bus.sync_read("Present_Position", normalize=False, num_retry=3), normalize=False)
        bus.enable_torque(num_retry=3)
        return
    bus.disable_torque(num_retry=3)
    for m in JOINTS:
        ensure("P_Coefficient", m, 16); ensure("I_Coefficient", m, 0); ensure("D_Coefficient", m, 32)
    ensure("Max_Torque_Limit", "gripper", 500)
    ensure("Protection_Current", "gripper", 250)
    ensure("Overload_Torque", "gripper", 25)
    bus.sync_write("Goal_Position", bus.sync_read("Present_Position", normalize=False, num_retry=3), normalize=False)
    bus.enable_torque(num_retry=3)


def torque_on():
    try:
        return any(bus.sync_read("Torque_Enable", normalize=False, num_retry=3).values())
    except Exception:
        return True  # unknown -> assume powered


def clamp(q):
    out = dict(q)
    for j in kin.CHAIN:
        if j in out:
            lo, hi = kin.LIMITS[j]
            out[j] = min(max(out[j], lo), hi)
    if "gripper" in out:
        out["gripper"] = min(max(out["gripper"], 0.0), 95.0)
    return out


def move(target, speed=SPEED, label=None):
    start = read()
    goal = clamp({**start, **target})
    dist = max(abs(goal[j] - start[j]) for j in JOINTS)
    n = max(int(max(dist / speed, 0.3) * HZ), 1)
    if label:
        print(f"  -> {label} ({n / HZ:.1f}s)")
    for i in range(1, n + 1):
        s = i / n; s = s * s * (3 - 2 * s)
        bus.sync_write("Goal_Position", {j: start[j] + (goal[j] - start[j]) * s for j in JOINTS})
        time.sleep(1 / HZ)
    time.sleep(0.2)
    out = settle(goal)
    vision.mark_motion()
    return out


def move_watch(target, speed=20.0, load_max=350, lag_max=6.0):
    """Eased move that samples servo load and lag; freezes on contact/strain.
    Returns (ok, peak_loads, reason). Normal moves peak at <= 12 % (120); contact shows as a jump."""
    start = read()
    goal = clamp({**start, **target})
    dist = max(abs(goal[j] - start[j]) for j in kin.CHAIN)
    n = max(int(max(dist / speed, 0.3) * HZ), 1)   # was 0.8 s minimum: small steps crawled
    peak = {j: 0 for j in start}
    for i in range(1, n + 1):
        s = i / n; s = s * s * (3 - 2 * s)
        cmd = {j: start[j] + (goal[j] - start[j]) * s for j in start}
        bus.sync_write("Goal_Position", cmd)
        time.sleep(1 / HZ)
        if i % 4 == 0 or i == n:
            ld = {k: int(v) for k, v in bus.sync_read("Present_Load", num_retry=2).items()}
            for j, v in ld.items():
                peak[j] = max(peak[j], abs(v))
            meas = read()
            lag = max(abs(cmd[j] - meas[j]) for j in kin.CHAIN)
            hot = {j: v for j, v in ld.items() if abs(v) > load_max and j != "gripper"}
            if hot or lag > lag_max:
                bus.sync_write("Goal_Position", bus.sync_read("Present_Position", normalize=False), normalize=False)
                vision.mark_motion()
                return False, peak, (f"load {hot} " if hot else "") + (f"lag {lag:.1f} deg" if lag > lag_max else "")
    settle(goal)
    vision.mark_motion()
    return True, peak, ""


def settle(goal, rounds=2, tol=0.6, max_fix=5.0):
    """Gravity sag correction. With P=16 the shoulder/elbow stop short under load (study3:
    shoulder +1.5 deg mean / +4 max, elbow +1.2 / +2.6 -> ~1 cm at 25 cm reach). Re-command
    each arm joint by its remaining error until the MEASURED angle matches the goal."""
    cmd = {j: goal[j] for j in kin.CHAIN}
    meas = read()
    for _ in range(rounds):
        err = {j: goal[j] - meas[j] for j in kin.CHAIN}
        if max(abs(e) for e in err.values()) < tol:
            break
        cmd = clamp({j: min(max(cmd[j] + err[j], goal[j] - max_fix), goal[j] + max_fix) for j in kin.CHAIN})
        bus.sync_write("Goal_Position", cmd)
        time.sleep(0.15)
        meas = read()
    return meas


def arm_q(q):
    return {j: q[j] for j in kin.CHAIN}


def tool_xyz(q=None):
    return kin.tool_pose(arm_q(q or read()))[:3, 3]


def ik_or_fail(xyz, seed, roll=0.0):
    q, err = kin.ik(xyz, arm_q(seed))
    tilt = math.degrees(math.acos(min(1.0, -kin.tool_pose(q)[2, 2])))
    if err > 0.006 or tilt > 12:
        raise RuntimeError(f"target {np.round(xyz, 3)} unreachable top-down (err {err*1000:.0f} mm, tilt {tilt:.0f} deg)")
    q["wrist_roll"] = roll
    return q


def move_xyz(xyz, roll=None, speed=SPEED, label=None):
    cur = read()
    q = ik_or_fail(xyz, cur, cur["wrist_roll"] if roll is None else roll)
    return move(q, speed, label)


def line_xyz(xyz, step=0.01, speed=20.0, label=None, stop=None):
    """Straight-ish line in small IK steps (used for vertical descents). stop() -> True aborts."""
    a = tool_xyz(); b = np.asarray(xyz, float)
    n = max(int(np.linalg.norm(b - a) / step), 1)
    if label:
        print(f"  -> {label}")
    for i in range(1, n + 1):
        if stop and stop():
            break
        move_xyz(a + (b - a) * i / n, speed=speed)
    return read()


def hover_xyz(phi_deg, r=HOVER_R, z=HOVER_Z):
    # pan + turns the arm toward -y in the URDF base frame
    p = math.radians(phi_deg)
    return np.array([r * math.cos(p), -r * math.sin(p), z])


def snap(name, target=None):
    f = vision.get_frame()
    d = vision.detect(f)
    cv2.imwrite(str(OUT / f"{name}.jpg"), vision.overlay(f, d, target))
    return f, d


# ---------- modes ----------
def do_look(r=HOVER_R, z=HOVER_Z):
    move(READY, label="ready")
    move_xyz(hover_xyz(0, r, z), roll=0, label=f"hover r={r} z={z}")
    q = read()
    print("  tool xyz (model):", np.round(tool_xyz(q), 3))
    _, d = snap("look")
    print("  saved lab/arm-vision/look.jpg; detection:", None if not d else {k: round(v, 1) if isinstance(v, float) else v for k, v in d.items()})
    input("  Look at the arm: is the gripper pointing straight down at the table? ENTER to return ")


def wait_enter_flag():
    flag = threading.Event()
    threading.Thread(target=lambda: (input(), flag.set()), daemon=True).start()
    return flag


def do_teach():
    move(READY, label="ready")
    move({"gripper": GRIP_OPEN}, label="open gripper")
    move_xyz(hover_xyz(0), roll=0, label="hover")
    input("\n  Slide the box under the gripper: centered between the open jaws, jaws across the box's\n"
          "  NARROW side. Keep hands clear after. ENTER when placed ")
    f, d = snap("teach-hover")
    if not d:
        raise RuntimeError("box not visible from hover; check lab/arm-vision/teach-hover.jpg")
    print(f"  box seen at ({d['cx']:.0f},{d['cy']:.0f}) size {d['frac']*100:.1f}% angle {d['angle']:.0f}")
    print("\n  ENTER to start lowering 5 mm at a time; press ENTER AGAIN when the jaw tips are at\n"
          "  about half the box height (or Ctrl-C if anything touches wrongly).")
    input()
    flag = wait_enter_flag()
    hover = tool_xyz()
    z = hover[2]
    while not flag.is_set() and z > 0.0:
        z -= 0.005
        move_xyz([hover[0], hover[1], z], speed=15)
        time.sleep(0.25)
    zg = tool_xyz()[2]
    print(f"  grasp height = {zg*1000:.0f} mm (tool point above table plane)")
    _, dl = snap("teach-low")
    input("  ENTER = close gripper ")
    q = move({"gripper": 0.0}, speed=25, label="close")
    closed = q["gripper"]
    print(f"  gripper stopped at {closed:.1f}% ({'holding something' if closed > 5 else 'closed fully: nothing between jaws?'})")
    line_xyz([hover[0], hover[1], zg + 0.05], label="lift 5 cm", speed=15)
    _, du = snap("teach-lifted")
    input("  Box lifted? ENTER = put it back down and release ")
    line_xyz([hover[0], hover[1], zg + 0.003], label="lower", speed=15)
    move({"gripper": GRIP_OPEN}, label="open")
    line_xyz(hover, label="back up", speed=15)
    TEACH.write_text(json.dumps({
        "hover_r": HOVER_R, "hover_z": HOVER_Z, "target_px": [d["cx"], d["cy"]], "target_frac": d["frac"],
        "target_angle": d["angle"], "grasp_z": zg, "closed_on_box": closed,
        "lifted_view": None if not du else [du["cx"], du["cy"], du["frac"]]}, indent=1))
    print(f"  saved {TEACH}")


def estimate_jacobian(base_xyz):
    """Pixel motion per meter of tool motion at hover, measured by two small probe moves."""
    J = np.zeros((2, 2))
    _, d0 = snap("probe-0")
    for i, dv in enumerate(([0.02, 0, 0], [0, 0.02, 0])):
        move_xyz(base_xyz + np.array(dv), speed=20)
        _, d = snap(f"probe-{i+1}")
        if not (d0 and d):
            raise RuntimeError("lost the box while probing")
        J[:, i] = [(d["cx"] - d0["cx"]) / 0.02, (d["cy"] - d0["cy"]) / 0.02]
        move_xyz(base_xyz, speed=20)
    print("  image Jacobian (px/m):\n", np.round(J))
    return J


def do_grab():
    t = json.loads(TEACH.read_text())
    target = np.array(t["target_px"])
    move(READY, label="ready")
    move({"gripper": GRIP_OPEN}, label="open gripper")
    # 1. search
    found = None
    for phi in [0, -15, 15, -30, 30, -45, 45, -60, 60]:
        for r in (0.20, 0.25, 0.16):
            try:
                move_xyz(hover_xyz(phi, r), roll=0, label=f"search pan {phi:+d} r={r}")
            except RuntimeError:
                continue
            _, d = snap("search", target)
            if d and not d["touches_edge"]:
                found = (phi, r, d); break
            if d:
                found = found or (phi, r, d)
        if found and not found[2]["touches_edge"]:
            break
    if not found:
        raise RuntimeError("box not found in the search area")
    print(f"  box seen at pan {found[0]} r {found[1]}")
    # 2. center under the gripper
    xyz = tool_xyz()
    J = estimate_jacobian(xyz)
    Jinv = np.linalg.pinv(J)
    for it in range(8):
        _, d = snap(f"servo-{it}", target)
        if not d:
            raise RuntimeError("lost the box while centering")
        err = np.array([d["cx"], d["cy"]]) - target
        print(f"  centering {it}: pixel error {np.round(err)}")
        if np.linalg.norm(err) < 10:
            break
        dxy = -0.7 * (Jinv @ err)
        dxy = np.clip(dxy, -0.03, 0.03)
        xyz = xyz + np.array([dxy[0], dxy[1], 0])
        move_xyz(xyz, speed=20)
    # 3. descend, close, lift, verify
    line_xyz([xyz[0], xyz[1], t["grasp_z"]], label="lower to grasp height", speed=15)
    q = move({"gripper": 0.0}, speed=25, label="close")
    holding = q["gripper"] > max(5.0, t["closed_on_box"] - 10)
    line_xyz([xyz[0], xyz[1], t["grasp_z"] + 0.06], label="lift", speed=15)
    _, d = snap("grab-lifted")
    print(f"  gripper at {q['gripper']:.1f}% -> {'HOLDING' if holding else 'empty?'}; camera sees box: {bool(d)}")
    input("  ENTER = put it down and go home ")
    line_xyz([xyz[0], xyz[1], t["grasp_z"] + 0.003], label="lower", speed=15)
    move({"gripper": GRIP_OPEN}, label="release")
    line_xyz([xyz[0], xyz[1], HOVER_Z], label="up", speed=15)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "look"
    bus.connect()
    home = None
    try:
        assert bus.is_calibrated, "calibration mismatch"
        home = read()
        print("home:", {k: round(v, 1) for k, v in home.items()})
        input("Clear ~40 cm around the arm, hand near power. ENTER = torque ON ")
        hold()
        if mode == "look":
            do_look(*(float(x) for x in sys.argv[2:4])) if len(sys.argv) >= 4 else do_look()
        elif mode == "teach":
            do_teach()
        elif mode == "grab":
            do_grab()
        move(READY, label="ready")
        move(home, label="home")
    except KeyboardInterrupt:
        bus.sync_write("Goal_Position", bus.sync_read("Present_Position", normalize=False), normalize=False)
        print("\nFROZEN in place.")
        try:
            input("Hand under the arm, then ENTER = torque OFF ")
        except KeyboardInterrupt:
            pass
    except Exception as e:
        print(f"\nSTOPPED: {e}")
        if torque_on():
            bus.sync_write("Goal_Position", bus.sync_read("Present_Position", normalize=False, num_retry=3), normalize=False)
            print("Arm frozen in place.")
            try:
                ans = input("ENTER = slowly return home then torque OFF; type 'off' + ENTER = torque OFF here ")
                if ans.strip() != "off" and home:
                    move(READY, speed=25, label="ready"); move(home, speed=25, label="home")
            except KeyboardInterrupt:
                pass
        else:
            print("Torque was never on; the arm did not move.")
    finally:
        bus.disable_torque()
        print("torque OFF")
        bus.disconnect(disable_torque=False)
