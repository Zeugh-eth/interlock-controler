"""Agent-side skills over arm-daemon's plain servo API. The daemon stays a thin bus owner; the
practices from reports/robot-ai-practices live here: observations synchronized and age-checked,
a stable base frame (IK and pixel->table projection offline), contact-first descent, and grasp /
release outcomes that are verified by evidence instead of assumed."""
import json
import math
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / "scripts"))
import camcal  # noqa: E402
import kin  # noqa: E402

kin.LIMITS = kin._limits(margin=0.8)

STALE_S = 0.5          # a gripper frame older than this is not a view of the current pose
TILT_MAX = 12.0        # deg from straight down the top-down IK may lean
IK_ERR_MAX = 0.006     # m
SPEED = 20.0
EMPTY_PCT = 3.0        # jaws closed to <= this: nothing between them
SLIP_PCT = 2.0         # jaw opening change after a lift that means the object moved
IK_SEED = {"shoulder_pan": 0.4, "shoulder_lift": -4.4, "elbow_flex": 7.5, "wrist_flex": 88.6, "wrist_roll": 0.0}   # look-down
REACH_PROBE_Z = (0.12, 0.11, 0.10, 0.09, 0.08, 0.07, 0.06, 0.05)   # m, highest first: what a refusal reports as reachable


class Skills:
    def __init__(self, base_url: str = "http://127.0.0.1:8800", timeout: float = 120):
        self.base = base_url.rstrip("/")
        self.timeout = timeout
        self._open = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._cal = None

    # --- transport ------------------------------------------------------------------------------
    def _http(self, method, path, body=None):
        data = json.dumps(body or {}).encode() if method == "POST" else None
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"} if data is not None else {})
        try:
            with self._open.open(req, timeout=self.timeout) as r:
                raw, ctype = r.read(), r.headers.get("Content-Type", "")
        except urllib.error.HTTPError as e:
            raw, ctype = e.read(), e.headers.get("Content-Type", "")
        except Exception as e:
            raise ConnectionError(f"arm-daemon not reachable at {self.base}: {e}")
        if ctype.startswith("image/"):
            return raw
        try:
            return json.loads(raw)
        except Exception:
            return {"refused": raw.decode(errors="replace")[:200]}

    def state(self):
        return self._http("GET", "/state")

    @staticmethod
    def _joints(state):
        return {j: float(e.get("deg", e.get("pct"))) for j, e in state["servos"].items()}

    # --- observation ----------------------------------------------------------------------------
    def observe(self) -> dict:
        """State and gripper frame read back to back; stale=True when the frame is older than
        STALE_S, missing, or the arm is still moving. Never decide from a stale frame."""
        s = self.state()
        frame = None
        try:
            f = self._http("GET", "/camera/gripper/frame.jpg")
            frame = f if isinstance(f, bytes) else None
        except Exception:
            frame = None
        cam = (s.get("cameras") or {}).get("gripper") or {}
        age = cam.get("age_s")
        stale = frame is None or not cam.get("ok") or age is None or age > STALE_S or s["motion"]["state"] == "moving"
        return {"t": time.time(), "joints": self._joints(s), "tool_xyz_m": s["tool"]["xyz_m"],
                "tilt_from_down_deg": s["tool"].get("tilt_from_down_deg"), "frame": frame, "frame_age_s": age,
                "stale": stale, "holder": s["control"]["holder"], "stop": s["stop"]["set"], "motion": s["motion"]["state"]}

    # --- base frame -----------------------------------------------------------------------------
    def _ik(self, xyz, cur, roll=None):
        """Top-down IK seeded from the current joints, then from the look-down pose when that fails:
        a folded start pose (crouch-tight) can leave the solver in a bad local minimum for a point
        that is reachable from look-down."""
        best = None
        for seed in ({j: cur[j] for j in kin.CHAIN}, {**IK_SEED, "wrist_roll": cur["wrist_roll"]}):
            q, err = kin.ik(list(xyz), seed)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(kin.tool_pose(q)[2, 2])))))
            if err <= IK_ERR_MAX and tilt <= TILT_MAX:
                q["wrist_roll"] = cur["wrist_roll"] if roll is None else float(roll)
                return {j: round(float(v), 2) for j, v in q.items()}, ""
            if best is None or err < best[0]:
                best = (err, tilt)
        return None, f"unreachable top-down: err {best[0]*1000:.0f} mm, tilt {best[1]:.0f} deg"

    def reachable_z(self, x, y, cur, zs=REACH_PROBE_Z) -> float | None:
        """Highest top-down reachable z (m) above (x, y) among zs, or None."""
        for z in zs:
            q, _ = self._ik([x, y, z], cur)
            if q is not None:
                return z
        return None

    def goto_xyz(self, xyz, roll=None, speed=SPEED, ignore_table=False) -> dict:
        """Move the tool point (fingertips) to xyz in the base frame, gripper pointing down.
        A refusal says what IS reachable: the highest top-down z above the same (x, y)."""
        s = self.state()
        cur = self._joints(s)
        q, why = self._ik(xyz, cur, roll)
        if q is None:
            rz = self.reachable_z(xyz[0], xyz[1], cur)
            if rz is not None and rz < float(xyz[2]):
                why += f"; at x {xyz[0]*100:.1f} y {xyz[1]*100:.1f} cm the highest top-down reachable z is {rz*100:.1f} cm (near the base keep z at or below that)"
            elif rz is None:
                why += f"; nothing top-down reachable above x {xyz[0]*100:.1f} y {xyz[1]*100:.1f} cm between {REACH_PROBE_Z[-1]*100:.0f} and {REACH_PROBE_Z[0]*100:.0f} cm"
            return {"outcome": "refused", "reason": why, "tool_xyz_m": s["tool"]["xyz_m"], "reachable_z_m": rz}
        body = {"joints": q, "speed": speed}
        if ignore_table:
            body["ignore_table"] = True
        r = self._http("POST", "/move", body)
        s2 = self.state()
        return {**r, "tool_xyz_m": s2["tool"]["xyz_m"], "joints": self._joints(s2)}

    def move_tool(self, dx=0.0, dy=0.0, dz=0.0, speed=SPEED, ignore_table=False) -> dict:
        """Relative tool move in the base frame (x forward, y left, z up), metres."""
        s = self.state()
        p = [a + b for a, b in zip(s["tool"]["xyz_m"], (dx, dy, dz))]
        return self.goto_xyz(p, speed=speed, ignore_table=ignore_table)

    def project(self, u: float, v: float, z: float | None = None, joints=None) -> dict:
        """Pixel (u, v) of the RAW gripper frame -> base-frame point on the table plane (or on the
        plane z). Uses the calibrated camera-to-tool transform and the CURRENT joints unless given."""
        if self._cal is None:
            self._cal = camcal.load()
        q = joints or self._joints(self.state())
        zz = float(self._cal.get("plane_z", 0.0)) if z is None else float(z)
        p = camcal.locate_on_plane(self._cal, q, (float(u), float(v)), zz)
        return {"x_m": round(float(p[0]), 4), "y_m": round(float(p[1]), 4), "z_m": zz,
                "note": "raw frame: image top = world left (+y), image left = toward the base (-x); verify with a small move before trusting"}

    # --- contact and grasp ----------------------------------------------------------------------
    def descend_to_contact(self, max_cm=8.0, step_cm=0.5, back_off_cm=1.0, floor_cm=0.3) -> dict:
        """Lower the tool in steps until the daemon's load/lag guard reports contact (frozen), then
        back off. Height comes from touch, not from the model. Uses ignore_table because the
        contact guard is the real floor here; never goes below floor_cm."""
        s = self.state()
        z0 = float(s["tool"]["xyz_m"][2])
        steps, zc, reason = 0, None, ""
        while steps * step_cm < max_cm:
            z = float(self.state()["tool"]["xyz_m"][2])
            if z - step_cm / 100 < floor_cm / 100:
                break
            r = self.move_tool(dz=-step_cm / 100, ignore_table=True)
            steps += 1
            if r["outcome"] == "frozen":
                zc, reason = float(r["tool_xyz_m"][2]), r.get("reason", "")
                break
            if r["outcome"] != "ok":
                return {"contact": False, "steps": steps, "outcome": r["outcome"], "reason": r.get("reason", ""), "tool_xyz_m": r["tool_xyz_m"]}
        if zc is None:
            s = self.state()
            return {"contact": False, "steps": steps, "z_start_m": z0, "tool_xyz_m": s["tool"]["xyz_m"], "reason": "no contact"}
        back = self.move_tool(dz=back_off_cm / 100, ignore_table=True)
        return {"contact": True, "steps": steps, "z_start_m": z0, "z_contact_m": zc, "reason": reason,
                "outcome": back["outcome"], "tool_xyz_m": back["tool_xyz_m"]}

    def _object_at_jaws(self, label):
        """Vision check: is `label` detected within VISION_HOLD_CM of the jaw target? None = the
        detector sees no such object at all (no evidence either way, the jaw reading decides);
        False = it sees the object somewhere else."""
        try:
            v = self._vision(label)
        except Exception:
            return None
        det = self._best_detection(v, label)
        if det is None or not det.get("offset_cm"):
            return None
        return abs(float(det["offset_cm"][0])) <= VISION_HOLD_CM and abs(float(det["offset_cm"][1])) <= VISION_HOLD_CM

    def grasp(self, close_pct=0.0, lift_cm=2.0, label: str | None = None) -> dict:
        """Close, read the jaw opening, lift a little, read again. verdict: held (jaws stopped on
        something and kept it through the lift), empty (jaws closed fully), slipped (opening
        changed after the lift), or unknown (a move was refused/frozen).
        A thin object can read as fully closed (1-2 %); with a label the vision overlay decides:
        the object riding with the jaws after the lift means held, the object left behind means empty."""
        r1 = self._http("POST", "/gripper", {"pct": float(close_pct)})
        g1 = float(r1.get("final", {}).get("gripper", self._joints(self.state())["gripper"]))
        seen_before = self._object_at_jaws(label) if label else None
        if g1 <= EMPTY_PCT and not seen_before:
            o = self.observe()
            ev = "jaws closed fully: nothing between them" + (f"; vision: no {label} at the jaw target" if label else "")
            return {"verdict": "empty", "jaw_pct_after_close": g1, "jaw_pct_after_lift": None, "frame": o["frame"], "evidence": ev}
        lift = self.move_tool(dz=lift_cm / 100, ignore_table=True)
        if lift["outcome"] != "ok":
            return {"verdict": "unknown", "jaw_pct_after_close": g1, "jaw_pct_after_lift": None, "frame": None, "evidence": f"lift {lift['outcome']}: {lift.get('reason','')}"}
        g2 = float(lift["joints"]["gripper"])
        o = self.observe()
        seen_after = self._object_at_jaws(label) if label else None
        if label and seen_after is not None:
            if seen_after:
                v, ev = "held", f"vision: {label} still at the jaw target after a {lift_cm} cm lift (jaws {g1} -> {g2} %)"
            elif g1 <= EMPTY_PCT:
                v, ev = "empty", f"vision: {label} not at the jaw target after the lift; jaws read {g1} %"
            else:
                v, ev = "slipped", f"vision: {label} left behind after the lift; jaws {g1} -> {g2} %"
        elif g2 <= EMPTY_PCT or abs(g2 - g1) > SLIP_PCT:
            v, ev = "slipped", f"jaw opening changed from {g1} to {g2} % after a {lift_cm} cm lift"
        else:
            v, ev = "held", f"jaws stopped at {g1} % and stayed at {g2} % through a {lift_cm} cm lift"
        return {"verdict": v, "jaw_pct_after_close": g1, "jaw_pct_after_lift": g2, "frame": o["frame"], "tool_xyz_m": o["tool_xyz_m"], "evidence": ev}

    def release(self, open_pct=50.0, retreat_cm=3.0) -> dict:
        """Open, retreat upward, observe: placement is confirmed by the frame after the retreat."""
        r = self._http("POST", "/gripper", {"pct": float(open_pct)})
        if r.get("outcome") != "ok":
            return {**r, "frame": None}
        up = self.move_tool(dz=retreat_cm / 100, ignore_table=True)
        o = self.observe()
        return {"outcome": up["outcome"], "reason": up.get("reason", ""), "tool_xyz_m": o["tool_xyz_m"], "frame": o["frame"], "stale": o["stale"]}

    # --- demos ----------------------------------------------------------------------------------
    def demo_stop(self, reported_outcome: str | None = None, notes: str | None = None) -> dict:
        """The agent's own verdict goes to meta.json as reported_outcome (the daemon's summary may
        add an inferred `outcome` of its own; the two are kept apart)."""
        body = {k: v for k, v in (("reported_outcome", reported_outcome), ("notes", notes)) if v is not None}
        return self._http("POST", "/demo/stop", body)


# ---------------------------------------------------------------------------------------------
# Analyze -> grab -> evaluate -> replan. Lessons from 23 Oct: study the whole object from a survey
# height before approaching; choose the side to close on from measured size; align with the
# vision offsets, not with a guessed pixel; judge a grasp by jaw opening vs expected width.
# ---------------------------------------------------------------------------------------------
JAW_MAX_CM = 7.5            # widest object the jaws can take at 95 %
SURVEY_Z = 0.11             # m: the whole object and its neighbours fit in the frame
APPROACH_Z = 0.06
MODEL_LOW_CM = 2.0          # the kinematic z runs about this much below the real table here
GRIP_DEPTH_CM = 1.5         # fingertips this far below the object's top
HEIGHT_BY_LABEL = {"cup": 9.0, "mug": 9.0, "bottle": 20.0, "can": 12.0, "ball": 4.0, "box": 3.0, "carton": 3.0, "package": 3.0}
ALIGN_TOL_CM = 1.0
MAX_CORRECTIONS = 3
MAX_LOW_CORRECTIONS = 2
DESCENT_STEP_M = 0.015      # paced final approach: re-check alignment after every step
VISION_HOLD_CM = 2.5        # object centre this close to the jaw target after a close = between the jaws
EDGE_PX = 3
MAX_ATTEMPTS = 3
MIN_PLAN_CONF = 0.15        # detections below this never drive a plan, an alignment or a hold verdict
PLANE_Z_CM = 0.3            # the calibrated table plane (lab/arm-vision/gripper-camera-cal.json plane_z), cm
PAN_SCAN_DEG = (15.0, -15.0, 30.0, -30.0)   # RELATIVE to the pan the survey started at; restored when nothing is found
SURVEY_FALLBACK_Z = (0.10, 0.09, 0.08)      # m: same (x, y), lower, when the requested survey height is unreachable


def open_pct_for(width_cm: float) -> float:
    """Jaw opening for an object of this width plus clearance; 3 cm -> ~55 %, 7 cm -> 95 %."""
    return float(round(min(95.0, max(30.0, 20.0 + width_cm * 10.7))))


def expected_jaw_pct(width_cm: float) -> float:
    """Where the jaws should stop when closed on an object of this width (3 cm -> ~5 %, 7 cm -> ~45 %)."""
    return float(max(2.0, (width_cm - 2.5) * 10.0))


class Skills(Skills):  # noqa: F811  (extends the class above; keeps one import for the MCP)
    # --- vision ---------------------------------------------------------------------------------
    def _vision(self, label=None):
        if label:
            try:
                self._http("POST", "/vision/prompts", {"prompts": [label]})
            except Exception:
                pass
        v = self._http("GET", "/vision")
        return v if isinstance(v, dict) else {"detections": []}

    @staticmethod
    def _label_matches(d, label) -> bool:
        return not label or label.lower() in str(d.get("label", "")).lower() or str(d.get("label", "")).lower() in label.lower()

    def _best_detection(self, v, label, min_conf: float = MIN_PLAN_CONF):
        """Best detection of `label` at or above min_conf. The overlay hides low-confidence boxes
        but used to let them drive plans: a 0.07 'carton' that was the leader arm's gripper got a
        feasible plan on 23 Oct. Below the floor the detection is reported, never acted on."""
        dets = [d for d in v.get("detections", []) if self._label_matches(d, label) and float(d.get("conf", 0)) >= min_conf]
        return max(dets, key=lambda d: d.get("conf", 0)) if dets else None

    def _low_conf_detection(self, v, label):
        """The best match below the planning floor, for the report (None when there is none)."""
        dets = [d for d in v.get("detections", []) if self._label_matches(d, label) and float(d.get("conf", 0)) < MIN_PLAN_CONF]
        return max(dets, key=lambda d: d.get("conf", 0)) if dets else None

    # --- measure --------------------------------------------------------------------------------
    @staticmethod
    def _det_px_size(det) -> float | None:
        bb = det.get("bbox_raw") or det.get("bbox_display")
        if not bb:
            return None
        return math.sqrt(max(1.0, float(bb[2])) * max(1.0, float(bb[3])))

    def measure_height(self, label: str, rise_cm: float = 3.0) -> dict:
        """How high the object's top is above the table, from parallax: look at it, rise by
        rise_cm, look again; the apparent size shrinks in proportion to the camera distance, so the
        distance follows and with it the top's height. The result is stored as the vision height
        for this label so look()/analyze offsets are measured on the object's top plane from now on.
        Needs the detector to see the object from both heights; otherwise says so (no guess)."""
        v1 = self._vision(label)
        d1 = self._best_detection(v1, label)
        if d1 is None:
            return {"label": label, "measured": False, "reason": f"{label} not detected from here (conf >= {MIN_PLAN_CONF}); lower the tool until it is, or declare the height yourself with look(height_cm=...)"}
        s1 = self._det_px_size(d1)
        tool = v1.get("tool") or {}
        cam_z1 = float(tool.get("camera_z_cm") or (float(tool.get("z_cm", 0.0)) + 6.5))
        up = self.move_tool(dz=rise_cm / 100)
        if up.get("outcome") != "ok":
            return {"label": label, "measured": False, "reason": f"could not rise {rise_cm} cm to measure: {up.get('reason', up.get('outcome'))}"}
        v2 = self._vision(label)
        d2 = self._best_detection(v2, label)
        s2 = self._det_px_size(d2) if d2 else None
        o = self.observe()
        if s1 is None or s2 is None or s2 <= 0:
            return {"label": label, "measured": False, "reason": f"{label} lost after rising {rise_cm} cm; tool now at z {o['tool_xyz_m'][2]*100:.1f} cm", "tool_xyz_m": o["tool_xyz_m"], "frame": o["frame"]}
        ratio = s1 / s2
        if ratio < 1.03:
            return {"label": label, "measured": False, "reason": f"no usable parallax (size ratio {ratio:.3f} over {rise_cm} cm): the object is far below the camera or the detector box did not change; try a larger rise_cm",
                    "size_px": [round(s1, 1), round(s2, 1)], "tool_xyz_m": o["tool_xyz_m"], "frame": o["frame"]}
        d_cm = rise_cm / (ratio - 1.0)                 # camera-to-top distance at the first look
        top_z_cm = cam_z1 - d_cm
        height_cm = max(0.0, top_z_cm - PLANE_Z_CM)
        try:
            self._http("POST", "/vision/settings", {"heights": {label: round(height_cm, 1)}})
            stored = True
        except Exception:
            stored = False
        return {"label": label, "measured": True, "top_z_cm": round(top_z_cm, 1), "height_above_table_cm": round(height_cm, 1),
                "camera_to_top_cm": round(d_cm, 1), "size_ratio": round(ratio, 3), "size_px": [round(s1, 1), round(s2, 1)],
                "stored_as_vision_height": stored, "tool_xyz_m": o["tool_xyz_m"], "frame": o["frame"],
                "note": "offsets and sizes for this label are now measured on the object's top plane; the tool stayed at the raised height"}

    # --- analyze --------------------------------------------------------------------------------
    def analyze_object(self, label: str, survey_z: float = SURVEY_Z) -> dict:
        """Study the object before touching it: rise to a survey height, find it (panning if needed),
        measure it, and derive a grasp plan: which side to close on (roll), jaw opening, heights.
        Survey height: the requested z above the CURRENT (x, y), else lower steps at the same spot
        (near the base 11 cm is not top-down reachable; moving x outward would move away from the
        object). Pan scan: small relative steps around the start pan; the start pan is restored when
        nothing is found, so the next look() shows the scene the caller had."""
        s = self.state()
        x, y = s["tool"]["xyz_m"][:2]
        start_pan = self._joints(s)["shoulder_pan"]
        survey_note = ""
        up = self.goto_xyz([x, y, survey_z])
        for fz in SURVEY_FALLBACK_Z:
            if up.get("outcome") == "ok":
                break
            if fz >= survey_z:
                continue
            up = self.goto_xyz([x, y, fz])
            if up.get("outcome") == "ok":
                survey_note = f"surveyed from {fz*100:.0f} cm: {survey_z*100:.0f} cm is not top-down reachable at x {x*100:.1f} cm"
        if up.get("outcome") != "ok":
            o = self.observe()
            return {"card": None, "plan": None, "reason": f"could not reach a survey height above x {x*100:.1f} y {y*100:.1f} cm: {up.get('reason', '')}",
                    "searched_pans": [], "survey_tool_z_m": o["tool_xyz_m"][2], "frame": o["frame"]}
        searched = []
        v = self._vision(label)
        det = self._best_detection(v, label)
        low = self._low_conf_detection(v, label)
        if det is None:
            for dpan in PAN_SCAN_DEG:
                pan = start_pan + dpan
                self._http("POST", "/servo/shoulder_pan", {"deg": pan})
                searched.append(round(pan, 1))
                v = self._vision(label)
                det = self._best_detection(v, label)
                low = low or self._low_conf_detection(v, label)
                if det:
                    break
            if det is None:
                self._http("POST", "/servo/shoulder_pan", {"deg": start_pan})       # back to the caller's view
        if det is not None and not self._full_view(det) and float(self.state()["tool"]["xyz_m"][2]) < 0.14:
            bk = self.move_tool(dz=0.03)                 # half-identified object: back up once and look again
            if bk.get("outcome") == "ok":
                v = self._vision(label)
                det = self._best_detection(v, label) or det
        o = self.observe()
        if det is None:
            why = f"{label} not found from survey height {o['tool_xyz_m'][2]*100:.1f} cm (pans tried {searched}, pan restored to {start_pan:.1f})"
            if low is not None:
                why += f"; only a low-confidence hit ({low.get('label')} {float(low.get('conf', 0)):.2f} at offset {low.get('offset_cm')} cm), below the {MIN_PLAN_CONF} planning floor"
            why += ". If you can see the object in the frame yourself, use align_to(label or a raw pixel) from here instead of surveying higher."
            return {"card": None, "plan": None, "reason": why, "low_confidence": low, "searched_pans": searched,
                    "survey_tool_z_m": o["tool_xyz_m"][2], "survey_note": survey_note, "frame": o["frame"]}
        full = self._full_view(det)
        w_cm, h_cm = (float(det["size_cm"][0]), float(det["size_cm"][1])) if det.get("size_cm") else (None, None)
        card = {"label": det.get("label", label), "conf": det.get("conf"),
                "table_xy_m": [round(det["table_xy_cm"][0] / 100, 4), round(det["table_xy_cm"][1] / 100, 4)] if det.get("table_xy_cm") else None,
                "offset_cm": det.get("offset_cm"), "size_cm": [w_cm, h_cm],
                "short_cm": min(w_cm, h_cm) if w_cm else None, "long_cm": max(w_cm, h_cm) if w_cm else None,
                "long_axis": ("x" if h_cm >= w_cm else "y") if w_cm else None,       # h = extent along +x (forward), w = along y
                "est_height_cm": HEIGHT_BY_LABEL.get(str(det.get("label", label)).lower(), 3.0),
                "seen_from_tool_xyz_m": o["tool_xyz_m"], "assumes_table": det.get("assumes_table", True), "full_view": full}
        if not full:
            return {"card": card, "plan": None, "reason": f"{card['label']} partly out of view: its box touches the frame edge, so size and centre are unknown; move the arm over it or raise it and analyze again",
                    "searched_pans": searched, "survey_tool_z_m": o["tool_xyz_m"][2], "survey_note": survey_note, "frame": o["frame"]}
        plan = self._plan_from_card(card)
        return {"card": card, "plan": plan, "searched_pans": searched, "survey_tool_z_m": o["tool_xyz_m"][2], "survey_note": survey_note, "frame": o["frame"]}

    @staticmethod
    def _full_view(det) -> bool:
        """The whole object is in frame: its box does not touch any edge (bbox_display + frame_wh
        from a test fake, or bbox_raw in the 640x480 camera frame)."""
        if det.get("bbox_display") and det.get("frame_wh"):
            x, y, w, h = det["bbox_display"]; fw, fh = det["frame_wh"]
        elif det.get("bbox_raw"):
            x, y, w, h = det["bbox_raw"]; fw, fh = 640, 480
        else:
            return True
        return x > EDGE_PX and y > EDGE_PX and x + w < fw - EDGE_PX and y + h < fh - EDGE_PX

    def _plan_from_card(self, card: dict) -> dict:
        # jaws close along x at roll 0 (fixed jaw toward the base), along y at roll 90
        short, long_ = card["short_cm"], card["long_cm"]
        roll = 90 if card["long_axis"] == "x" else 0            # close across the short side
        feasible = short is not None and short <= JAW_MAX_CM
        grasp_z = max(0.008, (card["est_height_cm"] - GRIP_DEPTH_CM - MODEL_LOW_CM) / 100 + 0.01)
        plan = {"label": card["label"], "target_xy_m": card["table_xy_m"], "roll_deg": roll,
                "across_jaws_cm": short, "open_pct": open_pct_for(short) if short else 95.0,
                "approach_z_m": APPROACH_Z, "grasp_z_m": round(grasp_z, 4), "lift_cm": 3.0,
                "attempt": 1, "shift_along_jaws_cm": 0.0, "resurvey": False, "feasible": feasible,
                "reason": "" if feasible else f"too wide: {short} cm across the narrowest side, jaws take {JAW_MAX_CM} cm; tip it onto a narrower side or push it against something first"}
        return plan

    # --- grab -----------------------------------------------------------------------------------
    def _offset(self, label):
        v = self._vision(label)
        det = self._best_detection(v, label)
        if det is None or not det.get("offset_cm"):
            return None
        return float(det["offset_cm"][0]), float(det["offset_cm"][1])

    def grab(self, plan: dict) -> dict:
        """Execute a plan: approach above the target with the chosen roll, align on the vision
        offsets (<= 1 cm, at most 3 corrections, the first one measuring how many cm the object
        really shifts per commanded cm), open, descend in steps re-checking alignment after each,
        and only close when the object is centred between the jaws at grasp height. Lift and evaluate.
        23 Oct lessons: every miss closed with the object beside a jaw; never close unaligned."""
        if not plan.get("feasible", True):
            return {"outcome": "refused", "reason": plan.get("reason", "plan infeasible"), "evaluation": {"quality": "refused"}}
        label = plan["label"]
        tx, ty = plan["target_xy_m"]
        shift = plan.get("shift_along_jaws_cm", 0.0) / 100
        if plan["roll_deg"] == 0:
            tx += shift
        else:
            ty += shift
        r = self.goto_xyz([tx, ty, plan["approach_z_m"]], roll=plan["roll_deg"])
        if r.get("outcome") != "ok":
            return {"outcome": r.get("outcome"), "reason": r.get("reason"), "stage": "approach", "evaluation": {"quality": "unknown"}}
        corrections, aligned, gain, off = 0, False, 1.0, self._offset(label)
        while True:
            if off is None:
                break
            fwd, left = off
            if abs(fwd) <= ALIGN_TOL_CM and abs(left) <= ALIGN_TOL_CM:
                aligned = True
                break
            if corrections >= MAX_CORRECTIONS:
                break
            dx = max(-0.03, min(0.03, fwd / gain / 100))
            dy = max(-0.03, min(0.03, left / gain / 100))
            self.move_tool(dx=dx, dy=dy)
            corrections += 1
            new = self._offset(label)
            if corrections == 1 and new is not None:        # measure the overlay's cm against a real move
                moved = max(abs(dx), abs(dy)) * 100
                if moved >= 0.5:
                    delta = abs(fwd - new[0]) if abs(dx) >= abs(dy) else abs(left - new[1])
                    gain = min(3.0, max(0.5, delta / moved))
            off = new
        if not aligned:
            o = self.observe()
            why = (f"offset fwd {off[0]:+} left {off[1]:+} cm after {corrections} corrections" if off else f"{label} not seen from the approach height")
            return {"outcome": "not_aligned", "reason": f"{why}; the object is not between the jaws, so the jaws were not closed",
                    "corrections": corrections, "aligned": False, "measured_gain": round(gain, 2),
                    "evaluation": {"quality": "not_aligned", "reason": why}, "frame": o["frame"]}
        s = self.state()
        axy = s["tool"]["xyz_m"][:2]
        self._http("POST", "/gripper", {"pct": float(plan["open_pct"])})
        # paced descent: steps, re-check alignment after each one (parallax moves the picture)
        z = float(s["tool"]["xyz_m"][2])
        gz = float(plan["grasp_z_m"])
        steps, low_corr, down, low_aligned = 0, 0, {"outcome": "ok"}, True
        while z - gz > 0.002:
            dz = max(-DESCENT_STEP_M, gz - z)
            down = self.move_tool(dz=dz, ignore_table=True)
            steps += 1
            if down.get("outcome") == "frozen":
                self.move_tool(dz=0.01, ignore_table=True)
                break
            if down.get("outcome") != "ok":
                break
            z = float(down["tool_xyz_m"][2])
            o2 = self._offset(label)
            if o2 is not None and (abs(o2[0]) > ALIGN_TOL_CM or abs(o2[1]) > ALIGN_TOL_CM):
                if low_corr >= MAX_LOW_CORRECTIONS:
                    low_aligned = False
                    break
                self.move_tool(dx=max(-0.015, min(0.015, o2[0] / gain / 100)), dy=max(-0.015, min(0.015, o2[1] / gain / 100)), ignore_table=True)
                low_corr += 1
        if not low_aligned:
            o2 = self._offset(label)
            self.goto_xyz([axy[0], axy[1], plan["approach_z_m"]], roll=plan["roll_deg"], ignore_table=True)
            why = f"offset fwd {o2[0]:+} left {o2[1]:+} cm at grasp height after {low_corr} corrections" if o2 else f"{label} lost from view near grasp height"
            return {"outcome": "not_aligned", "reason": f"{why}; backed up without closing", "corrections": corrections, "low_corrections": low_corr,
                    "aligned": aligned, "aligned_at_grasp_height": False, "measured_gain": round(gain, 2), "descent_steps": steps,
                    "evaluation": {"quality": "not_aligned", "reason": why}}
        g = self.grasp(close_pct=0.0, lift_cm=plan.get("lift_cm", 3.0), label=label)
        ev = self.evaluate_grasp(g, plan)
        return {"outcome": "ok", "corrections": corrections, "low_corrections": low_corr, "aligned": aligned, "aligned_at_grasp_height": True,
                "measured_gain": round(gain, 2), "descent_steps": steps, "aligned_xy_m": [round(float(v), 4) for v in axy],
                "descend": {k: down.get(k) for k in ("outcome", "reason", "tool_xyz_m")},
                "grasp": {k: v for k, v in g.items() if k != "frame"}, "evaluation": ev, "frame": g.get("frame")}

    # --- evaluate -------------------------------------------------------------------------------
    def evaluate_grasp(self, grasp: dict, plan: dict) -> dict:
        """Judge a grasp beyond held/empty: the jaws should stop near the opening expected for the
        planned width. Much narrower = an edge pinch (will drop); narrower than closed = empty."""
        exp = expected_jaw_pct(float(plan.get("across_jaws_cm") or 3.0))
        v = grasp.get("verdict")
        j1, j2 = grasp.get("jaw_pct_after_close"), grasp.get("jaw_pct_after_lift")
        if v == "empty":
            q, why = "empty", "jaws closed fully"
        elif v == "slipped":
            q, why = "slipped", f"opening went {j1} -> {j2} % after the lift"
        elif v != "held":
            q, why = "unknown", grasp.get("evidence", "")
        elif j1 is not None and j1 < 0.4 * exp:
            q, why = "edge", f"jaws stopped at {j1} %, expected about {exp:.0f} % for {plan.get('across_jaws_cm')} cm: holding an edge or corner"
        else:
            q, why = "good", f"jaws at {j1} % (expected ~{exp:.0f} %) and held through the lift"
        return {"quality": q, "reason": why, "expected_jaw_pct": round(exp, 1), "jaw_pct": j1, "jaw_pct_after_lift": j2}

    # --- replan ---------------------------------------------------------------------------------
    def replan(self, plan: dict, evaluation: dict) -> dict:
        """Adjust the plan after a failed or poor grasp. empty -> re-survey, go 0.5 cm lower;
        edge -> shift along the closing axis toward the object's middle; slipped -> lower and
        tighter; after MAX_ATTEMPTS -> infeasible, ask the human."""
        p = dict(plan)
        p["attempt"] = int(plan.get("attempt", 1)) + 1
        q = evaluation.get("quality")
        if p["attempt"] > MAX_ATTEMPTS:
            p.update(feasible=False, reason=f"{MAX_ATTEMPTS} attempts failed (last: {q}); ask the human to reposition the object or hand over control")
            return p
        if q == "empty":
            p.update(grasp_z_m=round(max(0.005, plan["grasp_z_m"] - 0.005), 4), resurvey=True)
        elif q == "edge":
            sign = 1.0 if plan.get("shift_along_jaws_cm", 0.0) >= 0 else -1.0
            step = max(1.0, (plan.get("across_jaws_cm") or 3.0) / 3)
            p.update(shift_along_jaws_cm=round(plan.get("shift_along_jaws_cm", 0.0) + sign * step, 2), resurvey=True)
        elif q == "slipped":
            p.update(grasp_z_m=round(max(0.005, plan["grasp_z_m"] - 0.005), 4), lift_cm=2.0)
        else:
            p.update(resurvey=True)
        p["feasible"] = True
        p["reason"] = f"attempt {p['attempt']} after {q}"
        return p


# ---------------------------------------------------------------------------------------------
# align_to: the five-move sequence that grabbed the carton on 23 Oct 06:28, as one closed loop on
# the RAW frame. It does not need the detector to see the object from a survey height (YOLO-World
# never saw that carton above 6 cm): a label is used when the detector sees it, otherwise a raw
# pixel the caller points at is tracked as an image patch. Geometry from jawband.py.
# ---------------------------------------------------------------------------------------------
import jawband  # noqa: E402

ALIGN_MAX_MOVES = 12
ALIGN_STEP_MAX_M = 0.03
LOW_STEP_MAX_M = 0.015
TRACK_MIN_SCORE = 0.45
TRACK_JUMP_PX = 300
CENTER_TOL_PX = 60


class _PatchTracker:
    """Follows one image patch across frames with multi-scale template matching. Seeded at a raw
    pixel the caller chose; the template is refreshed at each good match so the growing picture
    during a descent does not lose it. Fallback for _BlobTracker on objects without a clear blob."""

    def __init__(self, img, u, v, size=90):
        import cv2
        self.cv2, self.size = cv2, int(size)
        self.cx, self.cy = float(u), float(v)
        self.tpl = self._crop(self._gray(img), self.cx, self.cy)

    def _gray(self, img):
        return self.cv2.cvtColor(img, self.cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img

    def _crop(self, g, cx, cy):
        h, w = g.shape[:2]
        x0 = int(min(max(0, cx - self.size / 2), max(0, w - self.size)))
        y0 = int(min(max(0, cy - self.size / 2), max(0, h - self.size)))
        return g[y0:y0 + self.size, x0:x0 + self.size].copy()

    def locate(self, img, expect_px=None):
        cv2 = self.cv2
        g = self._gray(img)
        best = None
        for sc in (0.75, 0.87, 1.0, 1.15, 1.32, 1.5):
            t = cv2.resize(self.tpl, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA if sc < 1 else cv2.INTER_LINEAR)
            if t.shape[0] >= g.shape[0] or t.shape[1] >= g.shape[1] or min(t.shape[:2]) < 12:
                continue
            res = cv2.matchTemplate(g, t, cv2.TM_CCOEFF_NORMED)
            _, mx, _, loc = cv2.minMaxLoc(res)
            if best is None or mx > best[0]:
                best = (float(mx), loc, t.shape)
        if best is None:
            return None
        score, (x, y), (th, tw) = best
        cx, cy = x + tw / 2, y + th / 2
        if score < TRACK_MIN_SCORE or math.hypot(cx - self.cx, cy - self.cy) > TRACK_JUMP_PX:
            return None
        if score >= 0.6:
            self.tpl = self._crop(g, cx, cy)
        self.cx, self.cy = cx, cy
        return {"cx": round(cx, 1), "cy": round(cy, 1), "bbox": None, "method": "patch-tracker", "conf": round(score, 2)}


class _BlobTracker:
    """Follows the uniformly coloured region the caller pointed at (a white carton on a dark mat):
    flood-fills from the last centre (shifted by the expected pixel motion), takes the filled
    region's bbox, and rejects fills that leaked into the background or changed colour. Gives a
    real bbox, so the jaw-band test is the same as for a detector box and the growing picture
    during a descent costs nothing. Falls back to _PatchTracker when the fill is unusable."""
    MIN_FRAC, MAX_FRAC, TOL, COLOR_JUMP = 0.002, 0.45, 18, 70

    def __init__(self, img, u, v):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.cx, self.cy = float(u), float(v)
        self.mean = None
        self.patch = _PatchTracker(img, u, v)
        first = self._fill(img, self.cx, self.cy)
        if first is None:
            raise ValueError(f"no trackable region at raw pixel ({u:.0f}, {v:.0f}): point at the object's body, not its edge")
        self.mean = first["mean"]
        self.cx, self.cy = first["cx"], first["cy"]

    def _fill(self, img, sx, sy):
        cv2, np = self.cv2, self.np
        h, w = img.shape[:2]
        sx, sy = int(min(max(0, sx), w - 1)), int(min(max(0, sy), h - 1))
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        mask = np.zeros((h + 2, w + 2), np.uint8)
        flags = 4 | cv2.FLOODFILL_MASK_ONLY | (255 << 8)
        cv2.floodFill(blur, mask, (sx, sy), 0, (self.TOL,) * 3, (self.TOL,) * 3, flags)
        m = mask[1:-1, 1:-1]
        area = int(np.count_nonzero(m))
        frac = area / float(h * w)
        if frac < self.MIN_FRAC or frac > self.MAX_FRAC:
            return None
        ys, xs = np.nonzero(m)
        x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
        mean = blur[m > 0].reshape(-1, 3).mean(axis=0)
        if self.mean is not None and float(np.abs(mean - self.mean).max()) > self.COLOR_JUMP:
            return None
        return {"cx": (x0 + x1) / 2, "cy": (y0 + y1) / 2, "bbox": [float(x0), float(y0), float(x1 - x0 + 1), float(y1 - y0 + 1)],
                "mean": mean, "frac": frac}

    def locate(self, img, expect_px=None):
        ex, ey = (expect_px or (0.0, 0.0))
        for sx, sy in ((self.cx + ex, self.cy + ey), (self.cx, self.cy)):
            f = self._fill(img, sx, sy)
            if f is not None:
                self.cx, self.cy, self.mean = f["cx"], f["cy"], f["mean"]
                self.patch.cx, self.patch.cy = f["cx"], f["cy"]
                return {"cx": round(f["cx"], 1), "cy": round(f["cy"], 1), "bbox": f["bbox"], "method": "blob-tracker", "conf": round(f["frac"], 3)}
        p = self.patch.locate(img)
        if p is not None:
            self.cx, self.cy = p["cx"], p["cy"]
        return p


class Skills(Skills):  # noqa: F811
    def _raw_frame_img(self):
        import cv2
        import numpy as np
        f = self._http("GET", "/camera/gripper/frame.jpg")
        if not isinstance(f, bytes):
            return None
        img = cv2.imdecode(np.frombuffer(f, np.uint8), cv2.IMREAD_COLOR)
        return img

    def _annotated_raw(self, loc=None):
        """The daemon's raw view with the jaw band when it has one, else the plain raw frame; a
        tracked point is drawn on it."""
        f = self._http("GET", "/vision/raw.jpg")
        if not isinstance(f, bytes):
            f = self._http("GET", "/camera/gripper/frame.jpg")
        if not isinstance(f, bytes):
            return None
        if loc and loc.get("method") in ("patch-tracker", "blob-tracker"):
            try:
                import cv2
                import numpy as np
                img = cv2.imdecode(np.frombuffer(f, np.uint8), cv2.IMREAD_COLOR)
                cv2.circle(img, (int(loc["cx"]), int(loc["cy"])), 9, (0, 220, 255), 2, cv2.LINE_AA)
                cv2.putText(img, f'tracked {loc.get("conf")}', (int(loc["cx"]) + 12, int(loc["cy"]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1, cv2.LINE_AA)
                f = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            except Exception:
                pass
        return f

    def _locate(self, label, tracker, expect_px=None):
        """Where the object is in the RAW frame: detector bbox (conf >= floor) first, else the
        tracked blob/patch (expect_px: where it should have moved to). None = not seen by either."""
        v = self._vision(label) if label else self._http("GET", "/vision")
        if not isinstance(v, dict):
            v = {}
        det = self._best_detection(v, label) if label else None
        if det is not None and det.get("bbox_raw"):
            x, y, w, h = (float(t) for t in det["bbox_raw"])
            return {"cx": round(x + w / 2, 1), "cy": round(y + h / 2, 1), "bbox": [x, y, w, h], "method": "detector",
                    "conf": det.get("conf"), "px_per_cm_table": (v.get("tool") or {}).get("px_per_cm")}
        if tracker is not None:
            img = self._raw_frame_img()
            if img is not None:
                loc = tracker.locate(img, expect_px)
                if loc:
                    loc["px_per_cm_table"] = (v.get("tool") or {}).get("px_per_cm")
                    return loc
        return None

    @staticmethod
    def _band_error(loc, open_pct, width_cm, ppc):
        """Pixel shift (dx right, dy down) that puts the object in the band, and whether it is there."""
        if loc.get("bbox"):
            w = jawband.where(loc["bbox"], open_pct)
            return w["dx_px"], w["dy_px"], w["between_jaws"], w["where"]
        b = jawband.band(open_pct)
        half = max(6.0, float(width_cm) * float(ppc) / 2)
        dx = (b["target_left_x"] + half) - float(loc["cx"])
        dy = b["target_center_y"] - float(loc["cy"])
        ok = abs(dx) <= jawband.GAP_TOL_PX and abs(dy) <= CENTER_TOL_PX
        return round(dx, 1), round(dy, 1), ok, "between the jaws" if ok else f"tracked point {dx:+.0f} px right, {dy:+.0f} px down of the ideal spot"

    def align_to(self, label: str | None = None, u: float | None = None, v: float | None = None, width_cm: float = 3.0,
                 open_pct: float = 55.0, grasp_z_m: float = 0.02, descend: bool = True, max_moves: int = ALIGN_MAX_MOVES) -> dict:
        """Put the object between the fingers: open the jaws, slide the tool at the current height
        until the object sits just right of the fixed jaw across the jaw-tip row (the green band in
        the raw view), then descend in 1.5 cm steps re-centering after each until grasp_z_m. Never
        closes the jaws: call grasp(label) after an 'aligned' result. Works from any height the
        object is visible from (the 23 Oct carton: look-down, 8 cm). label: use the detector when it
        sees the object; u, v: a raw-frame pixel on the object to track when it does not (both may
        be given; the detector wins when it sees it). width_cm: the side the jaws will close across,
        used only to centre a tracked point."""
        if not label and (u is None or v is None):
            return {"outcome": "refused", "reason": "give a label the detector can see, or a raw-frame pixel (u, v) on the object"}
        s = self.state()
        if s["control"]["holder"] != "agent":
            return {"outcome": "refused", "reason": f"{s['control']['holder']} has control"}
        g = self._http("POST", "/gripper", {"pct": float(open_pct)})
        if g.get("outcome") not in ("ok", None):
            return {"outcome": g.get("outcome"), "reason": g.get("reason", ""), "stage": "open"}
        tracker = None
        if u is not None and v is not None:
            img = self._raw_frame_img()
            if img is None:
                return {"outcome": "refused", "reason": "no raw camera frame to track from"}
            try:
                tracker = _BlobTracker(img, u, v)
            except ValueError as e:
                return {"outcome": "refused", "reason": str(e)}
        loc = self._locate(label, tracker)
        if loc is None:
            return {"outcome": "lost", "reason": f"{label or 'the pixel'} not seen in the raw frame (detector conf >= {MIN_PLAN_CONF} or trackable patch)", "stage": "start", "frame": self._annotated_raw()}
        ppc = float(loc.get("px_per_cm_table") or 25.0) * 1.3      # objects stand above the table: start a little larger, then measure
        gain_measured, moves, phase = None, [], "lateral"
        cam_prev = None

        def tool_z():
            return float(self.state()["tool"]["xyz_m"][2])

        def refusal(r, stage):
            o = self.observe()
            out = {"outcome": "touch" if r.get("outcome") == "frozen" else r.get("outcome", "refused"), "reason": r.get("reason", ""), "stage": stage,
                   "moves": moves, "px_per_cm_measured": gain_measured, "tool_xyz_m": o["tool_xyz_m"], "frame": self._annotated_raw(loc)}
            if r.get("outcome") == "frozen":
                self.move_tool(dz=0.01, ignore_table=True)
                out["reason"] = f"load spike (contact) during {stage}: {r.get('reason', '')}; backed up 1 cm, look before retrying"
            return out

        # phase 1: lateral alignment at the current height
        dx_px, dy_px, ok, where = self._band_error(loc, open_pct, width_cm, ppc)
        n = 0
        while not ok and n < max_moves:
            dx_m, dy_m = jawband.tool_move_for(dx_px, dy_px, ppc)
            dx_m, dy_m = max(-ALIGN_STEP_MAX_M, min(ALIGN_STEP_MAX_M, dx_m)), max(-ALIGN_STEP_MAX_M, min(ALIGN_STEP_MAX_M, dy_m))
            r = self.move_tool(dx=dx_m, dy=dy_m)
            n += 1
            if r.get("outcome") != "ok":
                return refusal(r, "lateral")
            new = self._locate(label, tracker, expect_px=(-dx_m * 100 * ppc, dy_m * 100 * ppc))
            moves.append({"phase": phase, "dx_cm": round(dx_m * 100, 2), "dy_cm": round(dy_m * 100, 2), "before_px": [loc["cx"], loc["cy"]],
                          "after_px": [new["cx"], new["cy"]] if new else None, "tool_xyz_m": r.get("tool_xyz_m")})
            if new is None:
                return {"outcome": "lost", "reason": f"{label or 'the tracked patch'} left the view after a {dx_m*100:+.1f}, {dy_m*100:+.1f} cm move", "stage": "lateral",
                        "moves": moves, "tool_xyz_m": r.get("tool_xyz_m"), "frame": self._annotated_raw()}
            moved_px = math.hypot(new["cx"] - loc["cx"], new["cy"] - loc["cy"])
            cmd_cm = math.hypot(dx_m, dy_m) * 100
            if gain_measured is None and cmd_cm >= 0.5 and moved_px >= 5:
                ppc = max(8.0, min(150.0, moved_px / cmd_cm))          # px per cm at the object's own depth
                gain_measured = round(ppc, 1)
            loc = new
            dx_px, dy_px, ok, where = self._band_error(loc, open_pct, width_cm, ppc)
        if not ok:
            o = self.observe()
            return {"outcome": "not_aligned", "reason": f"after {n} moves the object is still {where} (dx {dx_px:+.0f} px, dy {dy_px:+.0f} px); nothing closed",
                    "moves": moves, "px_per_cm_measured": gain_measured, "final": {"where": where, "px": [loc["cx"], loc["cy"]], "method": loc["method"]},
                    "tool_xyz_m": o["tool_xyz_m"], "frame": self._annotated_raw(loc)}
        if not descend:
            o = self.observe()
            return {"outcome": "aligned", "at_grasp_height": False, "moves": moves, "px_per_cm_measured": gain_measured,
                    "final": {"where": where, "px": [loc["cx"], loc["cy"]], "method": loc["method"]}, "tool_xyz_m": o["tool_xyz_m"], "frame": self._annotated_raw(loc)}
        # phase 2: paced descent, re-centering after every step (parallax grows the picture)
        phase = "descend"
        z = tool_z()
        steps, low_moves = 0, 0
        while z - grasp_z_m > 0.002 and steps < 10:
            dz = max(-DESCENT_STEP_M, grasp_z_m - z)
            cam_prev = z
            r = self.move_tool(dz=dz, ignore_table=True)
            steps += 1
            if r.get("outcome") != "ok":
                return refusal(r, "descend")
            z = float(r["tool_xyz_m"][2])
            grow = (cam_prev + 0.065) / max(0.02, z + 0.065)              # scale grows as the camera nears the table (at least)
            ppc *= grow
            new = self._locate(label, tracker, expect_px=((loc["cx"] - 320) * (grow - 1), (loc["cy"] - 240) * (grow - 1)))
            moves.append({"phase": phase, "dz_cm": round(dz * 100, 2), "after_px": [new["cx"], new["cy"]] if new else None, "tool_xyz_m": r.get("tool_xyz_m")})
            if new is None:
                self.move_tool(dz=0.015, ignore_table=True)
                return {"outcome": "lost", "reason": f"{label or 'the tracked patch'} not seen at z {z*100:.1f} cm; backed up 1.5 cm", "stage": "descend",
                        "moves": moves, "px_per_cm_measured": gain_measured, "tool_xyz_m": self.state()["tool"]["xyz_m"], "frame": self._annotated_raw()}
            loc = new
            dx_px, dy_px, ok, where = self._band_error(loc, open_pct, width_cm, ppc)
            if not ok and low_moves < max_moves:
                dx_m, dy_m = jawband.tool_move_for(dx_px, dy_px, ppc)
                dx_m, dy_m = max(-LOW_STEP_MAX_M, min(LOW_STEP_MAX_M, dx_m)), max(-LOW_STEP_MAX_M, min(LOW_STEP_MAX_M, dy_m))
                r2 = self.move_tool(dx=dx_m, dy=dy_m, ignore_table=True)
                low_moves += 1
                if r2.get("outcome") != "ok":
                    return refusal(r2, "descend-correct")
                new = self._locate(label, tracker, expect_px=(-dx_m * 100 * ppc, dy_m * 100 * ppc))
                moves.append({"phase": "descend-correct", "dx_cm": round(dx_m * 100, 2), "dy_cm": round(dy_m * 100, 2), "after_px": [new["cx"], new["cy"]] if new else None, "tool_xyz_m": r2.get("tool_xyz_m")})
                if new is None:
                    self.move_tool(dz=0.015, ignore_table=True)
                    return {"outcome": "lost", "reason": "object not seen after a low correction; backed up 1.5 cm", "stage": "descend", "moves": moves,
                            "px_per_cm_measured": gain_measured, "tool_xyz_m": self.state()["tool"]["xyz_m"], "frame": self._annotated_raw()}
                loc = new
                dx_px, dy_px, ok, where = self._band_error(loc, open_pct, width_cm, ppc)
        o = self.observe()
        if not ok:
            self.move_tool(dz=0.03, ignore_table=True)
            return {"outcome": "not_aligned", "reason": f"at grasp height the object is {where} (dx {dx_px:+.0f} px, dy {dy_px:+.0f} px); backed up 3 cm, nothing closed",
                    "moves": moves, "px_per_cm_measured": gain_measured, "final": {"where": where, "px": [loc["cx"], loc["cy"]], "method": loc["method"]},
                    "tool_xyz_m": self.state()["tool"]["xyz_m"], "frame": self._annotated_raw(loc)}
        return {"outcome": "aligned", "at_grasp_height": True, "moves": moves, "descent_steps": steps, "px_per_cm_measured": gain_measured,
                "final": {"where": where, "px": [loc["cx"], loc["cy"]], "method": loc["method"]}, "tool_xyz_m": o["tool_xyz_m"],
                "next": "grasp(label) closes, lifts and verifies", "frame": self._annotated_raw(loc)}

    def grab_at(self, label: str | None = None, u: float | None = None, v: float | None = None, width_cm: float = 3.0,
                open_pct: float = 55.0, grasp_z_m: float = 0.02, lift_cm: float = 3.0) -> dict:
        """align_to, then grasp, then evaluate against the width: the whole pick for a caller that
        only knows what it sees. Stops without closing on anything but 'aligned'."""
        a = self.align_to(label=label, u=u, v=v, width_cm=width_cm, open_pct=open_pct, grasp_z_m=grasp_z_m, descend=True)
        if a.get("outcome") != "aligned":
            return {"outcome": a.get("outcome"), "reason": a.get("reason", ""), "align": {k: val for k, val in a.items() if k != "frame"}, "frame": a.get("frame")}
        g = self.grasp(close_pct=0.0, lift_cm=lift_cm, label=label)
        ev = self.evaluate_grasp(g, {"across_jaws_cm": width_cm, "open_pct": open_pct})
        return {"outcome": "ok", "align": {k: val for k, val in a.items() if k != "frame"}, "grasp": {k: val for k, val in g.items() if k != "frame"},
                "evaluation": ev, "frame": g.get("frame") or a.get("frame")}
