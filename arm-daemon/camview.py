"""Annotated gripper-camera view for humans and agents.

Pure functions over a raw BGR frame and the current joints:
  table_geometry(q)                 metric facts from the hand-eye calibration (lab/arm-vision/gripper-camera-cal.json)
  annotate(frame, q, detector, …)   -> (jpeg bytes, info dict)

The display is the raw frame rotated 90 degrees counter-clockwise so that world "forward" (away
from the base, +x) is image-up and world-left (+y) is image-left (checked against the calibrated
camera model's projected axes on 2026-10-23; clockwise put forward at the bottom). Everything metric assumes the object lies on the
table plane (cal plane_z): sizes and offsets are estimates, exact for flat things, a little large for
tall ones. Detections come from any object with .detect(frame, prompts, conf) -> [{label, conf,
bbox[x, y, w, h]}] (YoloWorldDetector below, or a test fake)."""
import json
import time
from pathlib import Path

import cv2
import numpy as np

import guards                      # puts scripts/ on sys.path and pins kin.LIMITS
from guards import kin, ROOT
import camcal                      # scripts/camcal.py

CAL = camcal.load()
JAWS_30 = json.loads((ROOT / "lab/arm-vision/jaws-30.json").read_text())
PAST_TIPS = 40                     # px below the jaw-tip row where a gripped object sits (vpick)
PLANE_Z = float(CAL.get("plane_z", 0.0))
GRID_CM = 5                        # default line spacing (1 cm is available via grid_cm=1)
GRID_HALF_CM = 15
MIN_CONF = 0.15                    # detections below this are hidden unless the client set prompts
MODELS = ROOT / "tools/models"
W, H = 640, 480
# colours (BGR)
C_GRID, C_GRID5, C_AXIS_X, C_AXIS_Y = (70, 70, 70), (120, 120, 120), (60, 60, 230), (60, 200, 60)
C_TARGET, C_DET, C_TEXT = (0, 220, 255), (255, 170, 0), (240, 240, 240)


class YoloWorldDetector:
    """Open-vocabulary boxes; the 'green pad' prompt is answered by arm-agent's colour rule if importable."""
    name = "yolo-world"
    DEFAULT = ("box", "carton", "cup", "bottle", "pen", "paper", "tray")

    def __init__(self, weights=None):
        from ultralytics import YOLOWorld
        self.model = YOLOWorld(str(weights or MODELS / "yolov8s-worldv2.pt"))
        self.classes = None

    def detect(self, frame, prompts=None, conf=0.05):
        classes = [p for p in (prompts or self.DEFAULT) if p]
        if not classes:
            return []
        if classes != self.classes:
            self.model.set_classes(classes)
            self.classes = classes
        r = self.model.predict(frame, conf=conf, verbose=False)[0]
        out = []
        for c, p, b in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist()):
            x0, y0, x1, y1 = b
            frac = (x1 - x0) * (y1 - y0) / (W * H)
            if 0.005 < frac < 0.6:
                out.append({"label": classes[int(c)], "conf": round(float(p), 3), "bbox": [round(x0), round(y0), round(x1 - x0), round(y1 - y0)]})
        return sorted(out, key=lambda d: -d["conf"])[:8]


# ---------- geometry ----------------------------------------------------------------------------
def _q(q):
    return {j: float(q[j]) for j in kin.CHAIN}


def display_pt(pt, shape):
    """Raw pixel -> pixel after a 90 degree counter-clockwise rotation (x' = y, y' = W-1-x)."""
    w = shape[1]
    return (float(pt[1]), w - 1 - float(pt[0]))


def _on_plane(q, px):
    return camcal.locate_on_plane(CAL, q, px, PLANE_Z)


def table_geometry(q):
    q = _q(q)
    Tbt = kin.tool_pose(q)
    Tbc = Tbt @ np.array(CAL["X"])
    axis = Tbc[:3, :3] @ np.array([0, 0, 1.0])
    looking_down = bool(axis[2] < -0.5)                     # within 60 degrees of straight down
    jt = [JAWS_30["mid"][0], JAWS_30["mid"][1] + PAST_TIPS]
    g = {"tool_z_cm": round(float(Tbt[2, 3]) * 100, 1), "camera_z_cm": round(float(Tbc[2, 3]) * 100, 1),
         "looking_down": looking_down, "jaw_target_px": [round(v, 1) for v in jt], "px_per_cm": None,
         "jaw_target_table_cm": None}
    if looking_down:
        P0 = _on_plane(q, jt)
        px = camcal.project(CAL, q, P0 + np.array([0.01, 0, 0]))
        py = camcal.project(CAL, q, P0 + np.array([0, 0.01, 0]))
        p0 = camcal.project(CAL, q, P0)
        g["px_per_cm"] = round(float((np.linalg.norm(px - p0) + np.linalg.norm(py - p0)) / 2), 1)
        g["jaw_target_table_cm"] = [round(float(v) * 100, 1) for v in P0[:2]]
    return g


# ---------- drawing -----------------------------------------------------------------------------
def _clip_poly(pts):
    return [p for p in pts if -2000 < p[0] < 3000 and -2000 < p[1] < 3000]


def _draw_grid(img, q, P0, grid_cm=GRID_CM):
    """Grid on the table plane around the jaw-target point (grid_cm spacing, bold every 5 cm), axes at the origin."""
    for k in range(-GRID_HALF_CM, GRID_HALF_CM + 1, grid_cm):
        for axis in (0, 1):
            pts = []
            for m in np.linspace(-GRID_HALF_CM, GRID_HALF_CM, 2 * GRID_HALF_CM + 1):
                d = np.zeros(3)
                d[axis] = k * 0.01
                d[1 - axis] = m * 0.01
                pts.append(camcal.project(CAL, q, P0 + d))
            pts = _clip_poly(pts)
            if len(pts) < 2:
                continue
            arr = np.array(pts, np.int32).reshape(-1, 1, 2)
            bold = k % 5 == 0
            cv2.polylines(img, [arr], False, C_GRID5 if bold else C_GRID, 2 if bold else 1, cv2.LINE_AA)
    o = camcal.project(CAL, q, P0)
    ax = camcal.project(CAL, q, P0 + np.array([0.05, 0, 0]))
    ay = camcal.project(CAL, q, P0 + np.array([0, 0.05, 0]))
    for tip, col in ((ax, C_AXIS_X), (ay, C_AXIS_Y)):
        if _clip_poly([tip]):
            cv2.arrowedLine(img, tuple(np.int32(o)), tuple(np.int32(tip)), col, 2, cv2.LINE_AA, tipLength=0.2)


def _label(img, text, org, color, scale=0.5):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = int(org[0]), int(org[1])
    cv2.rectangle(img, (x - 2, y - th - 4), (x + tw + 2, y + 3), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def annotate(frame, q, detector, prompts=None, conf=0.05, holder=None, grid_cm=GRID_CM, min_conf=MIN_CONF):
    """prompts=None -> detector defaults and detections under min_conf are hidden; explicit prompts show all."""
    q = _q(q)
    t0 = time.time()
    raw = frame.copy()
    geo = table_geometry(q)
    dets, hidden = [], 0
    if detector is not None:
        try:
            dets = detector.detect(frame, prompts, conf) or []
        except Exception as e:
            dets = []
            geo["detector_error"] = str(e)
        if not prompts:
            kept = [d for d in dets if d["conf"] >= min_conf]
            hidden = len(dets) - len(kept)
            dets = kept
    grid_cm = grid_cm if grid_cm in (1, 5) else GRID_CM
    P0 = _on_plane(q, geo["jaw_target_px"]) if geo["looking_down"] else None
    if P0 is not None:
        _draw_grid(raw, q, P0, grid_cm)
    # jaw target crosshair (raw frame)
    jt = tuple(int(v) for v in geo["jaw_target_px"])
    cv2.drawMarker(raw, jt, C_TARGET, cv2.MARKER_CROSS, 26, 2, cv2.LINE_AA)
    cv2.circle(raw, jt, int(JAWS_30["gap_px"] / 2), C_TARGET, 1, cv2.LINE_AA)
    # detections (metric on the table plane)
    out = []
    for d in dets:
        x, y, w, h = d["bbox"]
        cx, cy = x + w / 2, y + h / 2
        e = {"label": d["label"], "conf": d["conf"], "bbox_raw": [x, y, w, h],
             "center_display_px": [round(v) for v in display_pt((cx, cy), frame.shape)], "assumes_table": P0 is not None}
        if P0 is not None:
            Pc = _on_plane(q, (cx, cy))
            l, r = _on_plane(q, (x, cy)), _on_plane(q, (x + w, cy))
            t, b = _on_plane(q, (cx, y)), _on_plane(q, (cx, y + h))
            e["size_cm"] = [round(float(np.linalg.norm(r - l)) * 100, 1), round(float(np.linalg.norm(b - t)) * 100, 1)]
            off = (Pc - P0)[:2] * 100
            e["offset_cm"] = [round(float(off[0]), 1), round(float(off[1]), 1)]        # +forward, +left of the jaw target
            e["table_xy_cm"] = [round(float(v) * 100, 1) for v in Pc[:2]]
            e["dist_cm"] = round(float(np.linalg.norm(off)), 1)
        out.append(e)
        cv2.rectangle(raw, (int(x), int(y)), (int(x + w), int(y + h)), C_DET, 2)
    disp = cv2.rotate(raw, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if P0 is not None:                                     # centimetre labels on the bold lines, upright
        for k in range(-GRID_HALF_CM, GRID_HALF_CM + 1, 5):
            if k == 0:
                continue
            for d in (np.array([k * 0.01, 0, 0]), np.array([0, k * 0.01, 0])):
                px = camcal.project(CAL, q, P0 + d)
                if 0 <= px[0] < W and 0 <= px[1] < H:
                    dp = display_pt(px, frame.shape)
                    _label(disp, f"{k:+d}", (dp[0] + 3, dp[1] - 3), C_GRID5, 0.4)
    # upright text after rotation
    for e in out:
        px = e["center_display_px"]
        txt = f'{e["label"]} {e["conf"]:.2f}'
        if "size_cm" in e:
            txt += f'  {e["size_cm"][0]}x{e["size_cm"][1]}cm  fwd {e["offset_cm"][0]:+}  left {e["offset_cm"][1]:+} (on table)'
        _label(disp, txt, (max(2, px[0] - 60), max(14, px[1] - 8)), C_DET, 0.45)
    hud = f'tool z {geo["tool_z_cm"]} cm'
    if geo["px_per_cm"]:
        hud += f'   {geo["px_per_cm"]} px/cm on table'
        n = int(5 * geo["px_per_cm"])
        cv2.line(disp, (12, disp.shape[0] - 14), (12 + n, disp.shape[0] - 14), C_TEXT, 3)
        _label(disp, "5 cm", (12, disp.shape[0] - 20), C_TEXT, 0.45)
    else:
        hud += "   (not looking at the table: no grid)"
    if holder:
        hud += f"   holder {holder}"
    _label(disp, hud, (8, 18), C_TEXT, 0.5)
    _label(disp, f"up = forward (+x)   left = +y   grid {grid_cm} cm", (8, 36), C_TEXT, 0.42)
    ok, buf = cv2.imencode(".jpg", disp, [cv2.IMWRITE_JPEG_QUALITY, 80])
    jtd = display_pt(geo["jaw_target_px"], frame.shape)
    info = {"t": time.time(), "display_rotation_deg": -90, "prompts": list(prompts or (getattr(detector, "DEFAULT", ()) if detector else [])),
            "detector": getattr(detector, "name", None), "ms": round((time.time() - t0) * 1000),
            "tool": {"z_cm": geo["tool_z_cm"], "camera_z_cm": geo["camera_z_cm"], "looking_down": geo["looking_down"], "px_per_cm": geo["px_per_cm"]},
            "grid": {"drawn": P0 is not None, "spacing_cm": grid_cm, "px_per_cm": geo["px_per_cm"], "origin_display_px": [round(v) for v in jtd]},
            "min_conf": min_conf, "hidden_low_conf": hidden,
            "note": "(on table) sizes and offsets assume the object lies on the table plane; objects on top of something read larger and farther than they are",
            "jaw_target": {"raw_px": geo["jaw_target_px"], "display_px": [round(v) for v in jtd], "table_xy_cm": geo["jaw_target_table_cm"]},
            "detections": out}
    if "detector_error" in geo:
        info["detector_error"] = geo["detector_error"]
    return buf.tobytes(), info
