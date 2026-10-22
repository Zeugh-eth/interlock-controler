"""Where an object must sit in the RAW gripper frame to end up between the fingers.

Pure geometry, shared by camview (drawing) and skills (align_to). Everything is in raw 640x480
camera pixels: the camera is bolted to the gripper, so the jaw tips are at fixed pixels and only
the moving jaw's tip slides with the opening percent.

Measured facts (lab/arm-vision/jaws-30.json, 23 Oct 2026 frames):
  fixed jaw  = raw-LEFT finger, base side; tip at FIXED_TIP, it never moves
  moving jaw = raw-RIGHT finger; tip x grows ~8.4 px per percent open and leaves the frame above ~46 %
  jaw-tip row = ROW_Y; an object is gripped where it crosses this row
Every 23 Oct grasp that held had the object's left edge 50-90 px right of the fixed tip at the row
when the jaws closed; every empty close had the object under the fixed jaw or beyond the band.

Raw-frame axes (checked against the 06:28 grab): raw image-left = toward the base (-x), raw
image-top = world left (+y). To move an object RIGHT in the raw frame move the tool toward the base
(dx < 0); to move it DOWN move the tool left (dy > 0)."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
JAWS = json.loads((ROOT / "lab/arm-vision/jaws-30.json").read_text())
W, H = 640, 480
FIXED_TIP = (float(JAWS["left_tip"][0]), float(JAWS["left_tip"][1]))        # (250.7, 270.0)
MOVING_TIP_30 = (float(JAWS["right_tip"][0]), float(JAWS["right_tip"][1]))  # (501.9, 274.0) at 30 % open
ROW_Y = float(JAWS["mid"][1])                                              # 272
PX_PER_PCT = (MOVING_TIP_30[0] - FIXED_TIP[0]) / 30.0                      # ~8.4 px per percent open
GAP_PX = 70            # target: object's left edge this far right of the fixed tip (about 1 cm at grasp depth)
GAP_TOL_PX = 35        # |left edge - target| within this = aligned along the closing axis
UNDER_JAW_PX = 15      # left edge closer than this to the fixed tip (or left of it) = under the fixed jaw
ROW_MARGIN_PX = 25     # the object's vertical span must reach past the row by this on both sides
ROW_TARGET_DY = 40     # ideal: object centre this far below the row (where PAST_TIPS puts the jaw target)
EDGE_MARGIN_PX = 20


def moving_tip_x(open_pct: float) -> float:
    return FIXED_TIP[0] + PX_PER_PCT * max(0.0, float(open_pct))


def band(open_pct: float) -> dict:
    """The rectangle an object must occupy (crossing the row) to be between the fingers at this opening."""
    mx = moving_tip_x(open_pct)
    x0 = FIXED_TIP[0] + GAP_PX - GAP_TOL_PX
    x1 = min(W - EDGE_MARGIN_PX, mx - EDGE_MARGIN_PX)
    return {"x0": round(x0, 1), "x1": round(x1, 1), "y0": round(ROW_Y - ROW_MARGIN_PX, 1), "y1": round(ROW_Y + 2 * ROW_TARGET_DY, 1),
            "row_y": ROW_Y, "target_left_x": round(FIXED_TIP[0] + GAP_PX, 1), "target_center_y": round(ROW_Y + ROW_TARGET_DY, 1),
            "fixed_tip_px": [round(v, 1) for v in FIXED_TIP], "moving_tip_px": [round(mx, 1), round(MOVING_TIP_30[1], 1)],
            "moving_tip_in_frame": mx < W, "open_pct": round(float(open_pct), 1),
            "usable": x1 - x0 >= 40,                      # False: the jaws are too closed for anything to fit; open them first
            "min_open_pct_for_3cm": round((GAP_PX - GAP_TOL_PX + 40 + EDGE_MARGIN_PX) / PX_PER_PCT, 0)}


def where(bbox, open_pct: float) -> dict:
    """Judge a raw-frame bbox [x, y, w, h] against the band. Returns between_jaws, a short `where`
    phrase, and the pixel shift (dx_px right, dy_px down) that would put it in the ideal spot."""
    x, y, w, h = (float(v) for v in bbox)
    b = band(open_pct)
    left, right, top, bottom = x, x + w, y, y + h
    cy = y + h / 2
    dx_px = b["target_left_x"] - left
    spans_row = top <= ROW_Y - ROW_MARGIN_PX and bottom >= ROW_Y + ROW_MARGIN_PX
    dy_px = 0.0 if spans_row and abs(cy - b["target_center_y"]) < h / 2 else b["target_center_y"] - cy
    problems = []
    if left < FIXED_TIP[0] + UNDER_JAW_PX:
        problems.append("under the fixed jaw (move the tool toward the base)")
    elif left > b["target_left_x"] + GAP_TOL_PX:
        problems.append("too far from the fixed jaw (move the tool away from the base)")
    if b["moving_tip_in_frame"] and right > b["x1"]:
        problems.append("reaches past the moving jaw (open wider or shift toward the base)")
    if not spans_row:
        problems.append("above the jaw row (move the tool left)" if bottom < ROW_Y + ROW_MARGIN_PX else "below the jaw row (move the tool right)")
    between = not problems
    return {"between_jaws": between, "where": "between the jaws" if between else "; ".join(problems),
            "dx_px": round(dx_px, 1), "dy_px": round(dy_px, 1), "spans_row": spans_row, "band": b}


def tool_move_for(dx_px: float, dy_px: float, px_per_cm: float) -> tuple[float, float]:
    """Tool move (dx_m, dy_m) that shifts an object by (dx_px right, dy_px down) in the raw frame."""
    s = max(1.0, float(px_per_cm)) * 100.0
    return (-dx_px / s, dy_px / s)
