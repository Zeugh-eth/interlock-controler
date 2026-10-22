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


# --- 23 Oct: height-aware offsets and the raw view with the jaw band ------------------------------
def test_declared_height_shrinks_size_and_offset():
    """An object standing on something reads too large and too far on the table plane; with its
    top height declared the overlay measures on that plane instead."""
    _, flat = camview.annotate(blank(), LOOK_DOWN, FakeDet(), prompts=["box"])
    _, tall = camview.annotate(blank(), LOOK_DOWN, FakeDet(), prompts=["box"], heights={"box": 6.0})
    f, t = flat["detections"][0], tall["detections"][0]
    assert f["assumes_table"] is True and t["assumes_table"] is False and t["assumed_height_cm"] == 6.0
    assert t["size_cm"][0] < f["size_cm"][0] and t["size_cm"][1] < f["size_cm"][1]
    assert t["dist_cm"] < f["dist_cm"]
    assert tall["heights_cm"] == {"box": 6.0}


def test_raw_view_draws_band_and_judges_detections():
    import jawband
    _, info = camview.annotate(blank(), LOOK_DOWN, FakeDet(), prompts=["box"])
    jpg, info = camview.annotate_raw(blank(), LOOK_DOWN, info, 55.0)
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (480, 640)                     # raw orientation, not rotated
    b = info["jaw_band"]
    assert b["open_pct"] == 55.0 and b["moving_tip_in_frame"] is False and b["x0"] < b["x1"]
    d = info["detections"][0]                              # box at x 300..400, y 320..420: left edge 50 px right of the fixed tip, crosses the row? no: starts below it
    assert "between_jaws" in d and "where" in d and "dx_px" in d
    assert "raw_axes" in info and "toward the base" in info["raw_axes"]
    # a box that crosses the row with its left edge ~70 px right of the fixed jaw tip is between the jaws
    good = {"detections": [{"label": "box", "conf": 0.5, "bbox_raw": [jawband.FIXED_TIP[0] + 70, jawband.ROW_Y - 150, 120, 330]}]}
    _, good = camview.annotate_raw(blank(), LOOK_DOWN, good, 55.0)
    assert good["detections"][0]["between_jaws"] is True and good["detections"][0]["where"] == "between the jaws"
    under = {"detections": [{"label": "box", "conf": 0.5, "bbox_raw": [jawband.FIXED_TIP[0] - 40, jawband.ROW_Y - 150, 120, 330]}]}
    _, under = camview.annotate_raw(blank(), LOOK_DOWN, under, 55.0)
    assert under["detections"][0]["between_jaws"] is False and "under the fixed jaw" in under["detections"][0]["where"]
    assert under["detections"][0]["dx_px"] > 0                # must move right in the frame


def test_jawband_geometry_and_tool_move_sign():
    import jawband
    assert jawband.moving_tip_x(30) == jawband.MOVING_TIP_30[0]
    assert jawband.moving_tip_x(0) == jawband.FIXED_TIP[0]
    b30, b55 = jawband.band(30), jawband.band(55)
    assert b30["moving_tip_in_frame"] is True and b30["x1"] < b55["x1"]
    # object must move 100 px right and 50 px down -> tool toward the base and to the left (23 Oct 06:28 grab)
    dx, dy = jawband.tool_move_for(100, 50, 25.0)
    assert dx < 0 and dy > 0 and abs(dx) == 0.04 and dy == 0.02
    w = jawband.where([jawband.FIXED_TIP[0] + 200, jawband.ROW_Y - 150, 120, 330], 55)
    assert w["between_jaws"] is False and "too far from the fixed jaw" in w["where"] and w["dx_px"] < 0
    w = jawband.where([jawband.FIXED_TIP[0] + 70, jawband.ROW_Y + 60, 120, 200], 55)
    assert w["between_jaws"] is False and "below the jaw row" in w["where"]
