"""Stateful fake of arm-daemon for skill tests: joints move on /move and /gripper; an object at
`contact_z` (m) stops descents with a load freeze; `object_width_pct` is where the jaws stop when
closing on it; `frame_age` controls staleness."""
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import kin  # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class StatefulFakeDaemon:
    def __init__(self):
        self.q = {"shoulder_pan": 0.4, "shoulder_lift": -4.4, "elbow_flex": 7.5, "wrist_flex": 88.6, "wrist_roll": 0.0, "gripper": 30.0}
        self.contact_z = None          # m: moves whose tool z would go below this freeze (load)
        self.object_width_pct = None   # jaws stop here when closing (object between them)
        self.slip_after_lift = False
        self.frame_age = 0.03
        self.cam_ok = True
        # fake vision: one object on the table at (ox, oy) m with size (w_y, h_x) cm; offsets are
        # reported like the real overlay: fwd = ox - tool_x, left = oy - tool_y (cm)
        self.object = None            # {"label","x","y","w_cm","h_cm","conf","min_pan":None,"top_z":m}
        self.prompts = []
        self.heights = {}
        self.render_frames = False    # True: frame.jpg is a rendered 640x480 scene (bright object on dark) for tracker tests
        self.calls = []
        self.demo_stop_body = None
        outer = self

        def tool_z(q):
            return float(kin.tool_pose({j: q[j] for j in kin.CHAIN})[2, 3])

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype="application/json"):
                self.send_response(code); self.send_header("Content-Type", ctype); self.end_headers()
                self.wfile.write(body if isinstance(body, bytes) else json.dumps(body).encode())

            def _state(self):
                servos = {j: ({"pct": outer.q[j]} if j == "gripper" else {"deg": outer.q[j]}) for j in JOINTS}
                T = kin.tool_pose({j: outer.q[j] for j in kin.CHAIN})
                return {"t": 0, "bus": "ok", "control": {"holder": "agent"}, "stop": {"set": False}, "servos": servos,
                        "tool": {"xyz_m": [float(v) for v in T[:3, 3]], "tilt_from_down_deg": 0.0, "lowest_link_z_m": tool_z(outer.q)},
                        "motion": {"state": "idle", "id": 1, "result": None},
                        "cameras": {"gripper": {"ok": outer.cam_ok, "age_s": outer.frame_age}}}

            def do_GET(self):
                outer.calls.append(("GET", self.path, None))
                if self.path == "/state":
                    self._send(200, self._state())
                elif self.path == "/vision":
                    T = kin.tool_pose({j: outer.q[j] for j in kin.CHAIN})
                    tx, ty, tz = (float(v) for v in T[:3, 3])
                    dets = []
                    o = outer.object
                    if o and (o.get("min_pan") is None or outer.q["shoulder_pan"] >= o["min_pan"]):
                        gain = float(o.get("gain", 1.0))
                        if o.get("held"):
                            fwd, left = 0.0, 0.0
                        elif o.get("sticky_offset_cm"):
                            fwd, left = o["sticky_offset_cm"]
                        else:
                            fwd, left = (o["x"] - tx) * 100 * gain, (o["y"] - ty) * 100 * gain
                            if tz < 0.04:
                                fwd += float(o.get("shift_when_low_cm", 0.0))
                        w, h = o["w_cm"], o["h_cm"]
                        cam_z = tz + 0.065                                    # camera sits ~6.5 cm above the tool point
                        top_z = float(o.get("top_z", 0.0))                   # object-top height (m): apparent size grows as the camera nears it
                        ppc = 20.0 * 0.145 / max(0.02, cam_z - top_z)
                        cx, cy = 240 - left * ppc, 320 - fwd * ppc           # display frame 480x640: up = forward, left = +y
                        bbox = [round(cx - w * ppc / 2), round(cy - h * ppc / 2), round(w * ppc), round(h * ppc)]
                        if o.get("clipped"):
                            bbox = [0, round(cy - h * ppc / 2), round(w * ppc), round(h * ppc)]     # touches the left edge
                        rx, ry = 639 - cy, cx                                 # raw frame: x_raw = W-1-y_disp, y_raw = x_disp
                        bbox_raw = [round(rx - h * ppc / 2), round(ry - w * ppc / 2), round(h * ppc), round(w * ppc)]
                        dets.append({"label": o["label"], "conf": o.get("conf", 0.3), "size_cm": [w, h],
                                     "offset_cm": [round(fwd, 1), round(left, 1)], "bbox_display": bbox, "frame_wh": [480, 640],
                                     "bbox_raw": bbox_raw,
                                     "table_xy_cm": [round(o["x"] * 100, 1), round(o["y"] * 100, 1)], "assumes_table": True})
                    self._send(200, {"detections": dets, "prompts": outer.prompts,
                                     "tool": {"z_cm": round(tz * 100, 1), "camera_z_cm": round((tz + 0.065) * 100, 1), "looking_down": True, "px_per_cm": 20.0},
                                     "jaw_target": {"table_xy_cm": [round(tx * 100, 1), round(ty * 100, 1)], "raw_px": [376.3, 312]},
                                     "heights_cm": dict(outer.heights)})
                elif self.path == "/camera/gripper/frame.jpg" or self.path == "/vision/raw.jpg":
                    if outer.cam_ok:
                        self._send(200, outer.render() if outer.render_frames else b"\xff\xd8fake", "image/jpeg")
                    else:
                        self._send(503, {"refused": "no frame"})
                else:
                    self._send(404, {"refused": "nope"})

            def _result(self, outcome, reason=""):
                return {"id": len(outer.calls), "outcome": outcome, "reason": reason, "final": dict(outer.q), "peak_load": 50}

            def do_POST(self):
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                outer.calls.append(("POST", self.path, body))
                if self.path == "/move":
                    tgt = {**outer.q, **{k: float(v) for k, v in body["joints"].items()}}
                    if outer.contact_z is not None and tool_z(tgt) < outer.contact_z and tool_z(tgt) < tool_z(outer.q):
                        self._send(200, self._result("frozen", "load 420 on wrist_flex"))      # stays put
                        return
                    outer.q = tgt
                    if outer.slip_after_lift and outer.object_width_pct == 0.0:
                        outer.q["gripper"] = 0.0       # the object dropped: jaws close the rest of the way
                    self._send(200, self._result("ok"))
                elif self.path == "/gripper":
                    pct = float(body["pct"])
                    if outer.object_width_pct is not None and pct < outer.object_width_pct:
                        pct = outer.object_width_pct
                        if outer.slip_after_lift:
                            outer.object_width_pct = 0.0   # next move: object gone, jaws close fully
                    outer.q["gripper"] = pct
                    self._send(200, self._result("ok"))
                elif self.path == "/vision/prompts":
                    p = body.get("prompts", [])
                    outer.prompts = p.split(",") if isinstance(p, str) else list(p)
                    self._send(200, {"prompts": outer.prompts})
                elif self.path == "/vision/settings":
                    h = body.get("heights")
                    if isinstance(h, dict):
                        for k, v in h.items():
                            if v:
                                outer.heights[str(k).lower()] = float(v)
                            else:
                                outer.heights.pop(str(k).lower(), None)
                    self._send(200, {"grid_cm": 5, "min_conf": 0.15, "heights": dict(outer.heights)})
                elif self.path.startswith("/servo/") and self.path.endswith("shoulder_pan"):
                    outer.q["shoulder_pan"] = float(body["deg"])
                    self._send(200, self._result("ok"))
                elif self.path == "/demo/stop":
                    outer.demo_stop_body = body
                    self._send(200, {"id": "d", "rows": 1, "images": 1, **body})
                else:
                    self._send(200, self._result("ok"))

        self.srv = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_port}"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def close(self):
        self.srv.shutdown()

    def raw_bbox(self):
        """The object's raw-frame bbox as /vision would report it now (None when not visible)."""
        import urllib.request
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(self.url + "/vision", timeout=5) as r:
            dets = json.loads(r.read()).get("detections", [])
        return dets[0]["bbox_raw"] if dets else None

    def render(self) -> bytes:
        """A raw frame: dark table, a bright rectangle where /vision says the object is (its raw bbox)."""
        import cv2
        import numpy as np
        img = np.full((480, 640, 3), 40, np.uint8)
        cv2.putText(img, "seed", (30, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 90, 90), 1)
        o = self.object
        if o:
            T = kin.tool_pose({j: self.q[j] for j in kin.CHAIN})
            tx, ty, tz = (float(v) for v in T[:3, 3])
            fwd, left = (o["x"] - tx) * 100, (o["y"] - ty) * 100
            ppc = 20.0 * 0.145 / max(0.02, (tz + 0.065) - float(o.get("top_z", 0.0)))
            cx, cy = 240 - left * ppc, 320 - fwd * ppc
            rx, ry = 639 - cy, cx
            w, h = o["w_cm"], o["h_cm"]
            x0, y0 = int(rx - h * ppc / 2), int(ry - w * ppc / 2)
            cv2.rectangle(img, (x0, y0), (int(x0 + h * ppc), int(y0 + w * ppc)), (235, 235, 235), -1)
            cv2.rectangle(img, (x0 + 6, y0 + 6), (x0 + 26, y0 + 26), (60, 60, 200), -1)       # a logo so the patch is not uniform
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return buf.tobytes()
