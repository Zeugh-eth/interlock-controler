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
    assert vis.set_settings({"grid_cm": 1, "min_conf": 0.5}) == {"grid_cm": 1, "min_conf": 0.5}
    time.sleep(0.15)
    assert vis.info()["grid"]["spacing_cm"] == 1 and vis.info()["min_conf"] == 0.5
    assert vis.set_settings({"grid_cm": 3}) == {"grid_cm": 1, "min_conf": 0.5}       # invalid spacing ignored
