"""Pink/red object detection on gripper-camera frames from cam-server.

  get_frame()        -> BGR image from http://127.0.0.1:8765/frame.jpg
  detect(frame)      -> dict(cx, cy, area, frac, w, h, angle, bbox) of the largest pink/red blob, or None
  overlay(frame, d)  -> annotated copy for inspection

Run directly: grabs a frame, saves lab/arm-vision/detect.jpg (overlay) and detect-mask.png.
"""
import time, urllib.request
from pathlib import Path
import cv2
import numpy as np

URL = "http://127.0.0.1:8765/frame.jpg"
OUT = Path(__file__).resolve().parents[1] / "lab/arm-vision"

# Red wraps around hue 0/180 in OpenCV HSV; pink sits at ~150-175. Saturation floor rejects
# skin and wood; value floor rejects shadows.
HUE_RANGES = [(0, 8), (150, 180)]
S_MIN, V_MIN = 110, 70
MIN_FRAC = 0.002  # ignore blobs under 0.2 % of the image


# go1: the gripper camera shows a move ~0.5 s after it happens (measured: 0.49-0.56 s). Any frame read
# sooner after the arm stops is stale, which silently broke steering once moves got faster.
CAMERA_LAG_S = 0.40   # re-measured 0.31 s median after the event-driven cam-server stream (was 0.49-0.56)
LAST_MOTION = [0.0]


def mark_motion():
    LAST_MOTION[0] = time.time()


def get_frame(retries=3):
    wait = LAST_MOTION[0] + CAMERA_LAG_S - time.time()
    if wait > 0:
        time.sleep(wait)
    for i in range(retries):
        try:
            data = urllib.request.urlopen(URL, timeout=3).read()
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                return img
        except OSError:
            time.sleep(0.3)
    raise RuntimeError("no frame from cam-server; is scripts/cam-server.command running?")


def mask_of(frame):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    m = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in HUE_RANGES:
        m |= cv2.inRange(hsv, (lo, S_MIN, V_MIN), (hi, 255, 255))
    k = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(cv2.morphologyEx(m, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)


def detect(frame):
    m = mask_of(frame)
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    h, w = frame.shape[:2]
    if area < MIN_FRAC * w * h:
        return None
    mo = cv2.moments(c)
    (rx, ry), (rw, rh), ang = cv2.minAreaRect(c)
    x, y, bw, bh = cv2.boundingRect(c)
    return {"cx": mo["m10"] / mo["m00"], "cy": mo["m01"] / mo["m00"], "area": area, "frac": area / (w * h),
            "w": rw, "h": rh, "angle": ang, "bbox": (x, y, bw, bh),
            "touches_edge": x <= 2 or y <= 2 or x + bw >= w - 2 or y + bh >= h - 2}


def overlay(frame, d, target=None):
    img = frame.copy()
    h, w = img.shape[:2]
    cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
    if target:
        cv2.drawMarker(img, (int(target[0]), int(target[1])), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 24, 2)
    if d:
        x, y, bw, bh = d["bbox"]
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
        cv2.circle(img, (int(d["cx"]), int(d["cy"])), 6, (0, 255, 255), -1)
        cv2.putText(img, f"{d['frac']*100:.1f}% ({d['cx']:.0f},{d['cy']:.0f})", (x, max(y - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return img


if __name__ == "__main__":
    import sys
    f = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else get_frame()
    d = detect(f)
    print(d)
    cv2.imwrite(str(OUT / "detect.jpg"), overlay(f, d))
    cv2.imwrite(str(OUT / "detect-mask.png"), mask_of(f))
