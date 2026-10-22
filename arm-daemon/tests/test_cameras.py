import time

from cameras import CameraWorker


def _worker(fake_cam):
    w = CameraWorker(fake_cam.url, names=("gripper",), hz=50)
    w.start()
    time.sleep(0.15)
    return w


def test_frame_and_health(fake_cam):
    w = _worker(fake_cam)
    jpg, age = w.frame("gripper")
    assert jpg.startswith(b"\xff\xd8") and age < 1
    h = w.health()["gripper"]
    assert h["ok"] is True and h["camera"] == "USB HD Camera" and h["age_s"] < 1
    w.stop()


def test_camera_down_reports_not_ok(fake_cam):
    fake_cam.down = True
    w = _worker(fake_cam)
    assert w.frame("gripper") is None
    assert w.health()["gripper"]["ok"] is False
    w.stop()


def test_stale_frame_not_ok(fake_cam):
    fake_cam.frame_age = 3000.0
    w = _worker(fake_cam)
    assert w.health()["gripper"]["ok"] is False
    w.stop()


def test_unknown_camera_and_server_gone():
    w = CameraWorker("http://127.0.0.1:1", names=("gripper",), hz=50)
    w.start()
    time.sleep(0.1)
    assert w.frame("nope") is None and w.frame("gripper") is None
    assert w.health()["gripper"] == {"age_s": None, "ok": False, "camera": None}
    w.stop()
