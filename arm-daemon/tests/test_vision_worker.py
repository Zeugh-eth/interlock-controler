import time


def test_worker_produces_annotated_frames_and_info(vis):
    f = vis.frame()
    assert f is not None and f[0][:2] == b"\xff\xd8"
    info = vis.info()
    assert info["detections"][0]["label"] == "box" and "tool" in info and info["detector"] == "fake"


def test_worker_prompts_change(vis):
    vis.set_prompts(["cup", "pen"])
    time.sleep(0.15)
    assert vis.detector.prompts_seen[-1] == ["cup", "pen"] and vis.info()["prompts"] == ["cup", "pen"]
    vis.set_prompts([])
    time.sleep(0.15)
    assert vis.detector.prompts_seen[-1] == ["box"]           # empty -> detector default


def test_worker_survives_missing_frames():
    from vision_worker import VisionWorker
    v = VisionWorker(frame_fn=lambda: None, joints_fn=lambda: {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 0, "wrist_flex": 0, "wrist_roll": 0},
                     holder_fn=lambda: "human", detector=None, hz=50)
    v.start(); time.sleep(0.1)
    assert v.frame() is None and v.info()["detections"] == [] and v.info().get("error") == "no camera frame"
    v.stop()


def test_worker_settings(vis):
    st = vis.set_settings({"grid_cm": 1, "min_conf": 0.5})
    assert st["grid_cm"] == 1 and st["min_conf"] == 0.5 and st["heights"] == {}
    time.sleep(0.15)
    assert vis.info()["grid"]["spacing_cm"] == 1 and vis.info()["min_conf"] == 0.5
    st = vis.set_settings({"grid_cm": 3})
    assert st["grid_cm"] == 1 and st["min_conf"] == 0.5       # invalid spacing ignored


def test_worker_settings_accept_heights_and_serves_raw_view():
    import time
    import cv2, numpy as np
    from vision_worker import VisionWorker
    f = np.full((480, 640, 3), 40, np.uint8)
    ok, buf = cv2.imencode(".jpg", f)
    q = {"shoulder_pan": 0.4, "shoulder_lift": -4.4, "elbow_flex": 7.5, "wrist_flex": 88.6, "wrist_roll": 0.0, "gripper": 55.0}
    w = VisionWorker(frame_fn=lambda: (buf.tobytes(), 0.01), joints_fn=lambda: q, detector=None, hz=20)
    assert w.set_settings({"heights": {"Carton": 7, "bad": "x"}})["heights"] == {"carton": 7.0}
    assert w.set_settings({"heights": {"carton": 0}})["heights"] == {}
    w.start()
    try:
        for _ in range(50):
            if w.raw_frame():
                break
            time.sleep(0.05)
        rf = w.raw_frame()
        assert rf and rf[0][:2] == b"\xff\xd8"
        assert w.info()["jaw_band"]["open_pct"] == 55.0
    finally:
        w.stop()
