import json

import cv2
import numpy as np

import camview

LOOK_DOWN = {"shoulder_pan": 0.4, "shoulder_lift": -4.4, "elbow_flex": 7.5, "wrist_flex": 88.6, "wrist_roll": 0.0, "gripper": 30.0}
READY = {j: 0.0 for j in LOOK_DOWN}


def blank():
    f = np.full((480, 640, 3), 40, np.uint8)
    cv2.rectangle(f, (300, 320), (400, 420), (200, 200, 255), -1)
    return f


class FakeDet:
    name = "fake"

    def detect(self, frame, prompts=None, conf=0.05):
        return [{"label": "box", "conf": 0.9, "bbox": [300, 320, 100, 100]},
                {"label": "bottle", "conf": 0.06, "bbox": [50, 50, 80, 80]}]


def test_table_scale_at_look_down():
    g = camview.table_geometry(LOOK_DOWN)
    assert 15 < g["px_per_cm"] < 70          # ~465 px focal / ~18 cm camera height (camera sits 6.6 cm behind the tool)
    assert g["tool_z_cm"] > 7 and g["looking_down"] is True
    assert len(g["jaw_target_px"]) == 2


def test_no_grid_when_not_looking_down():
    g = camview.table_geometry(READY)
    assert g["looking_down"] is False and g["px_per_cm"] is None


def test_annotate_returns_jpeg_and_metric_info():
    jpg, info = camview.annotate(blank(), LOOK_DOWN, FakeDet(), prompts=["box"])
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (640, 480)       # rotated 90 degrees (CCW) for display
    d = info["detections"][0]
    assert d["label"] == "box" and 1.0 < d["size_cm"][0] < 8.0 and 1.0 < d["size_cm"][1] < 8.0   # 100 px at ~23 px/cm, oblique view
    assert "offset_cm" in d and len(d["offset_cm"]) == 2 and "center_display_px" in d
    assert info["grid"]["px_per_cm"] == info["tool"]["px_per_cm"] if "px_per_cm" in info["tool"] else True
    assert info["tool"]["z_cm"] > 7 and info["grid"]["spacing_cm"] == 5     # 5 cm lines by default
    assert d["assumes_table"] is True and "on table" in info["note"]
    assert json.dumps(info)                  # serialisable


def test_annotate_without_detector_and_not_looking_down():
    jpg, info = camview.annotate(blank(), READY, None)
    assert jpg[:2] == b"\xff\xd8" and info["detections"] == [] and info["grid"]["drawn"] is False


def test_display_point_mapping():
    # raw (x, y) in a 640x480 frame -> rotated 90 CCW: x' = y, y' = W-1-x
    assert camview.display_pt((10, 20), (480, 640)) == (20, 629)


def test_forward_axis_points_up_in_display():
    # project a point 5 cm forward of the jaw target on the table; in the display it must be above (smaller y)
    import camcal, numpy as np
    q = {j: float(LOOK_DOWN[j]) for j in camview.kin.CHAIN}
    P0 = camcal.locate_on_plane(camview.CAL, q, camview.JAWS_30["mid"], camview.PLANE_Z)
    o = camview.display_pt(camcal.project(camview.CAL, q, P0), (480, 640))
    fwd = camview.display_pt(camcal.project(camview.CAL, q, P0 + np.array([0.05, 0, 0])), (480, 640))
    left = camview.display_pt(camcal.project(camview.CAL, q, P0 + np.array([0, 0.05, 0])), (480, 640))
    assert fwd[1] < o[1] - 20 and abs(fwd[0] - o[0]) < 40      # up
    assert left[0] < o[0] - 20 and abs(left[1] - o[1]) < 40    # left


def test_low_confidence_hidden_unless_prompted():
    _, info = camview.annotate(blank(), LOOK_DOWN, FakeDet())             # default prompts
    assert [d["label"] for d in info["detections"]] == ["box"]
    assert info["hidden_low_conf"] == 1 and info["min_conf"] == 0.15
    _, info = camview.annotate(blank(), LOOK_DOWN, FakeDet(), prompts=["bottle"])
    assert [d["label"] for d in info["detections"]] == ["box", "bottle"]  # explicit prompts: show all


def test_grid_spacing_option():
    _, info = camview.annotate(blank(), LOOK_DOWN, None, grid_cm=1)
    assert info["grid"]["spacing_cm"] == 1
    _, info = camview.annotate(blank(), LOOK_DOWN, None, grid_cm=5)
    assert info["grid"]["spacing_cm"] == 5
