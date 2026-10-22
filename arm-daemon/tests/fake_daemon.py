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
        self.object = None            # {"label","x","y","w_cm","h_cm","conf","min_pan":None}
        self.prompts = []
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
                        ppc = 20.0
                        cx, cy = 240 - left * ppc, 320 - fwd * ppc           # display frame 480x640: up = forward, left = +y
                        bbox = [round(cx - w * ppc / 2), round(cy - h * ppc / 2), round(w * ppc), round(h * ppc)]
                        if o.get("clipped"):
                            bbox = [0, round(cy - h * ppc / 2), round(w * ppc), round(h * ppc)]     # touches the left edge
                        dets.append({"label": o["label"], "conf": o.get("conf", 0.3), "size_cm": [w, h],
                                     "offset_cm": [round(fwd, 1), round(left, 1)], "bbox_display": bbox, "frame_wh": [480, 640],
                                     "table_xy_cm": [round(o["x"] * 100, 1), round(o["y"] * 100, 1)], "assumes_table": True})
                    self._send(200, {"detections": dets, "prompts": outer.prompts, "tool": {"z_cm": round(tz * 100, 1), "looking_down": True, "px_per_cm": 20.0},
                                     "jaw_target": {"table_xy_cm": [round(tx * 100, 1), round(ty * 100, 1)]}})
                elif self.path == "/camera/gripper/frame.jpg":
                    if outer.cam_ok:
                        self._send(200, b"\xff\xd8fake", "image/jpeg")
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
