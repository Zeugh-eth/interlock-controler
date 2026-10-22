"""arm-daemon entry point: the single owner of the SO-101 serial bus, serving the API on
127.0.0.1:8800. Start it from Terminal via start.command. Never from a Claude shell or a watchdog."""
import argparse
import secrets
import signal
import subprocess
import sys
from pathlib import Path

from aiohttp import web

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.append(str(ROOT / "scripts"))

PORT_DEV = "/dev/cu.usbmodem5B790812561"
LEADER_DEV = "/dev/cu.usbmodem5B901010331"
DEMO_DIR = HERE / "demos"
KEY_FILE = HERE / "human.key"
STOP_FILE = ROOT / "arm-agent/STOP"
POSES = ROOT / "lab/arm-vision/pose-library.json"
LOG_DIR = HERE / "logs"


def other_bus_owner():
    try:
        out = subprocess.run(["lsof", "-t", PORT_DEV], capture_output=True, text=True, timeout=3).stdout.split()
    except Exception:
        return []
    return [p for p in out if p.strip()]


def human_key() -> str:
    if not KEY_FILE.exists():
        KEY_FILE.write_text(secrets.token_hex(16))
        KEY_FILE.chmod(0o600)
    return KEY_FILE.read_text().strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8800)
    ap.add_argument("--cam-url", default="http://127.0.0.1:8765")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--fake", action="store_true", help="fake servo bus (no serial port): dry runs and API work")
    ap.add_argument("--leader-port", default=LEADER_DEV, help="leader arm port for holder=leader teleop ('' = none)")
    ap.add_argument("--no-yolo", action="store_true", help="annotated view without the YOLO-World detector")
    ap.add_argument("--vision-hz", type=float, default=5.0)
    a = ap.parse_args()

    from api import build_app
    from bus import BusWorker
    from cameras import CameraWorker
    from ctrl import ControlState

    if a.fake:
        sys.path.insert(0, str(HERE / "tests"))
        from conftest import FakeBackend, FakeLeader
        backend = FakeBackend()
        leader = FakeLeader()
    else:
        owners = other_bus_owner()
        if owners:
            print(f"refusing to start: {PORT_DEV} is held by pid(s) {' '.join(owners)}", file=sys.stderr)
            sys.exit(2)
        from bus import RealBackend, RealLeader
        backend = RealBackend()
        leader = RealLeader(a.leader_port) if a.leader_port else None

    key = human_key()
    control = ControlState(key, STOP_FILE)
    cams = CameraWorker(a.cam_url)
    bus = BusWorker(backend, control, POSES, hz=a.hz, log_dir=LOG_DIR, leader=leader,
                    frame_fn=lambda: cams.frame("gripper"), demo_dir=DEMO_DIR)
    cams.start()
    bus.start()
    from vision_worker import VisionWorker
    detector = None
    if not a.no_yolo:
        try:
            from camview import YoloWorldDetector
            detector = YoloWorldDetector()
        except Exception as e:
            print(f"vision: YOLO-World unavailable ({e}); grid only", file=sys.stderr)
    JOINTS5 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    def _vision_joints():
        servos = bus.snapshot()["servos"]
        q = {j: servos.get(j, {}).get("deg", 0.0) for j in JOINTS5}
        q["gripper"] = servos.get("gripper", {}).get("pct", 30.0)        # the raw view draws the moving jaw for this opening
        return q
    vis = VisionWorker(frame_fn=lambda: cams.frame("gripper"), joints_fn=_vision_joints,
                       holder_fn=lambda: control.holder, detector=detector, hz=a.vision_hz)
    vis.start()
    app = build_app(control, bus, cams, human_key=key, vision=vis)

    async def on_shutdown(app_):
        vis.stop()
        bus.stop()
        cams.stop()

    app.on_shutdown.append(on_shutdown)
    loop_sig = {signal.SIGTERM, signal.SIGINT}
    for s in loop_sig:
        signal.signal(s, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    print(f"arm-daemon on http://127.0.0.1:{a.port}  backend={'fake' if a.fake else 'real'}  key={KEY_FILE}")
    web.run_app(app, host="127.0.0.1", port=a.port, print=None, handle_signals=False)


if __name__ == "__main__":
    main()
