import sys
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
ROOT = HERE.parents[1]
POSE_LIB = ROOT / "lab/arm-vision/pose-library.json"

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class FakeBackend:
    """Instant-tracking servo bus. block_joint keeps one joint still (lag); load is settable;
    slow makes each goal write take 20 ms so a move spans many ticks."""

    def __init__(self):
        self.q = {j: 0.0 for j in JOINTS}
        self.load = {j: 50 for j in JOINTS}
        self.temp = {j: 35 for j in JOINTS}
        self.torque = True
        self.block_joint = None
        self.slow = False
        self.sag = 0.0          # deg each arm joint sits short of its goal (gravity)
        self.fail_connect = None
        self.calls = []
        self.connected = False

    def connect(self):
        if self.fail_connect:
            raise RuntimeError(self.fail_connect)
        self.connected = True

    def disconnect(self):
        self.connected = False

    def read(self):
        return dict(self.q)

    def read_load(self):
        return dict(self.load)

    def read_temp(self):
        return dict(self.temp)

    def torque_on(self):
        return self.torque

    def hold(self):
        self.calls.append("hold")
        self.torque = True

    def write_goal(self, q):
        if self.slow:
            time.sleep(0.02)
        self.goals = dict(q)
        for j, v in q.items():
            if j != self.block_joint:
                self.q[j] = float(v) - (self.sag if j in ("shoulder_lift", "elbow_flex") else 0.0)

    def freeze(self):
        self.calls.append("freeze")
        self.torque = True

    def disable_torque(self):
        self.calls.append("disable_torque")
        self.torque = False

    def settle(self, goal):
        return dict(self.q)


@pytest.fixture
def backend():
    return FakeBackend()


@pytest.fixture
def control(tmp_path):
    from ctrl import ControlState
    return ControlState("k", tmp_path / "STOP")


@pytest.fixture
def worker(backend, control):
    from bus import BusWorker
    w = BusWorker(backend, control, POSE_LIB, hz=200)
    w.start()
    time.sleep(0.02)
    yield w
    w.stop()


@pytest.fixture
def fake_cam():
    from tests.fake_cam import FakeCam
    c = FakeCam()
    yield c
    c.close()


@pytest.fixture
def key():
    return "k"


@pytest.fixture
def cams(fake_cam):
    from cameras import CameraWorker
    w = CameraWorker(fake_cam.url, names=("gripper",), hz=50)
    w.start()
    time.sleep(0.1)
    yield w
    w.stop()


@pytest.fixture
async def client(aiohttp_client, worker, control, cams):
    from api import build_app
    return await aiohttp_client(build_app(control, worker, cams))


class FakeLeader:
    def __init__(self):
        self.q = {j: 0.0 for j in JOINTS}
        self.connected = False

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def read(self):
        return dict(self.q)


class FakeFrames:
    def __init__(self):
        self.n = 0

    def bump(self):
        self.n += 1

    def __call__(self):
        return (b"\xff\xd8frame%d" % self.n, 0.01)


@pytest.fixture
def leader():
    return FakeLeader()


@pytest.fixture
def frames():
    return FakeFrames()


@pytest.fixture
def worker_t(backend, control, leader, frames, tmp_path):
    """Worker with a leader arm, a frame source and a demo directory."""
    from bus import BusWorker
    w = BusWorker(backend, control, POSE_LIB, hz=200, leader=leader, frame_fn=frames, demo_dir=tmp_path / "demos")
    w.start()
    time.sleep(0.02)
    yield w
    w.stop()


@pytest.fixture
async def client_t(aiohttp_client, worker_t, control, cams):
    from api import build_app
    return await aiohttp_client(build_app(control, worker_t, cams))


class FakeDet:
    name = "fake"
    DEFAULT = ("box",)

    def __init__(self):
        self.prompts_seen = []

    def detect(self, frame, prompts=None, conf=0.05):
        self.prompts_seen.append(list(prompts or self.DEFAULT))
        return [{"label": "box", "conf": 0.9, "bbox": [300, 320, 100, 100]}]


class JpegFrames:
    """frame_fn returning a real JPEG of a blank 640x480 frame."""

    def __init__(self):
        import cv2
        import numpy as np
        f = np.full((480, 640, 3), 40, np.uint8)
        self.jpg = cv2.imencode(".jpg", f)[1].tobytes()

    def __call__(self):
        return (self.jpg, 0.01)


@pytest.fixture
def vis(worker):
    from vision_worker import VisionWorker
    v = VisionWorker(frame_fn=JpegFrames(), joints_fn=lambda: {j: worker.snapshot()["servos"][j]["deg"] for j in JOINTS if j != "gripper"},
                     holder_fn=lambda: "human", detector=FakeDet(), hz=50)
    v.start()
    time.sleep(0.2)
    yield v
    v.stop()


@pytest.fixture
async def client_v(aiohttp_client, worker, control, cams, vis):
    from api import build_app
    return await aiohttp_client(build_app(control, worker, cams, vision=vis))
