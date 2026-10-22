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
        q, err = kin.ik(list(xyz), {j: cur[j] for j in kin.CHAIN})
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -float(kin.tool_pose(q)[2, 2])))))
        if err > IK_ERR_MAX or tilt > TILT_MAX:
            return None, f"unreachable top-down: err {err*1000:.0f} mm, tilt {tilt:.0f} deg"
        q["wrist_roll"] = cur["wrist_roll"] if roll is None else float(roll)
        return {j: round(float(v), 2) for j, v in q.items()}, ""

    def goto_xyz(self, xyz, roll=None, speed=SPEED, ignore_table=False) -> dict:
        """Move the tool point (fingertips) to xyz in the base frame, gripper pointing down."""
        s = self.state()
        cur = self._joints(s)
        q, why = self._ik(xyz, cur, roll)
        if q is None:
            return {"outcome": "refused", "reason": why, "tool_xyz_m": s["tool"]["xyz_m"]}
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
        """Vision check: is `label` detected within VISION_HOLD_CM of the jaw target? (None = no view)"""
        try:
            v = self._vision(label)
        except Exception:
            return None
        det = self._best_detection(v, label)
        if det is None or not det.get("offset_cm"):
            return False
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
PAN_SCAN = (35.0, -40.0)


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

    def _best_detection(self, v, label):
        dets = [d for d in v.get("detections", []) if not label or label.lower() in str(d.get("label", "")).lower() or str(d.get("label", "")).lower() in label.lower()]
        return max(dets, key=lambda d: d.get("conf", 0)) if dets else None

    # --- analyze --------------------------------------------------------------------------------
    def analyze_object(self, label: str, survey_z: float = SURVEY_Z) -> dict:
        """Study the object before touching it: rise to a survey height, find it (panning if needed),
        measure it, and derive a grasp plan: which side to close on (roll), jaw opening, heights."""
        s = self.state()
        x, y = s["tool"]["xyz_m"][:2]
        up = self.goto_xyz([x, y, survey_z])
        for fx, fz in ((0.24, survey_z), (0.22, 0.09), (0.20, 0.08)):      # near the base a high top-down pose is unreachable
            if up.get("outcome") == "ok":
                break
            up = self.goto_xyz([fx, y, fz])
        searched = []
        det, v = None, self._vision(label)
        det = self._best_detection(v, label)
        if det is None:
            for pan in PAN_SCAN:
                self._http("POST", "/servo/shoulder_pan", {"deg": pan})
                searched.append(pan)
                v = self._vision(label)
                det = self._best_detection(v, label)
                if det:
                    break
        if det is not None and not self._full_view(det) and float(self.state()["tool"]["xyz_m"][2]) < 0.14:
            self.move_tool(dz=0.03)                      # half-identified object: back up once and look again
            v = self._vision(label)
            det = self._best_detection(v, label) or det
        o = self.observe()
        if det is None:
            return {"card": None, "plan": None, "reason": f"{label} not found from survey height (pans tried {searched})",
                    "searched_pans": searched, "survey_tool_z_m": o["tool_xyz_m"][2], "frame": o["frame"]}
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
                    "searched_pans": searched, "survey_tool_z_m": o["tool_xyz_m"][2], "frame": o["frame"]}
        plan = self._plan_from_card(card)
        return {"card": card, "plan": plan, "searched_pans": searched, "survey_tool_z_m": o["tool_xyz_m"][2], "frame": o["frame"]}

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
