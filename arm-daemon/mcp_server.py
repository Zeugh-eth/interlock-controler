"""MCP server for an agent driving the SO-101 through arm-daemon (http://127.0.0.1:8800).

It is an AGENT client: it never sends the human key. Writes are refused with `human has control`
while the human holds the arm (ask the user to switch the holder on the dashboard), with
`leader has control` during teleop, and with `bus released` while LeRobot owns the bus.

Register:  claude mcp add arm-daemon -s project -- uv --directory <repo>/tools/arm-lerobot run python <repo>/arm-daemon/mcp_server.py
Tool list: uv run python mcp_server.py --list
"""
import argparse
import json
import sys
import urllib.error
import urllib.request

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.utilities.types import Image
from mcp_types import CallToolResult, TextContent, ToolAnnotations

BASE = "http://127.0.0.1:8800"
TIMEOUT = 120

INSTRUCTIONS = """Drives a Seeed SO-101 arm through arm-daemon, as the AGENT client. Decide from the gripper
camera: look() returns the annotated view, already rotated so image-up = forward (+x, away from the
base) and image-left = +y (left), with a 1 cm table grid and object sizes/offsets in cm. The raw
camera frame (look(raw=True), /camera/gripper/frame.jpg) is rolled: raw image-top = world left.
Every write returns a motion result: outcome ok | frozen | refused, with a reason. refused means
nothing moved: 'human has control' (ask the user to hand control to the agent on the dashboard),
'leader has control' (teleop running), 'bus released', 'stop is set', 'busy', 'path within X cm of
table' (pass ignore_table only when the user agrees), unit or range errors. frozen means a load or lag
spike (contact) stopped it where it is: look() and think before retrying.
Guards: joint limits clamp, load/lag freeze, table clearance (advisory, refusable). Units: joints in
degrees, gripper in percent open 0-95. Prefer nudge() steps <= 15 deg with look() between steps.
Named poses: standby/crouch-tight, ready, straight, look-down, rest (torque off).
Demonstrations: before a task call find_demos("<task words>"): each hit is a skill card with the
outcome, the grasp height/joints, gripper percents, lift height, release point, and phases with
keyframes. read_demo(id) gives sampled rows plus images for the details; replay_demo(id) re-executes
one under the guards. Record your own work with demo_start(task)/demo_stop(); a card is written
automatically when it stops, failures included (tag 'failed').
GRAB DECISION TREE (23 Oct 2026; every miss broke one of the facts below, every success kept them).
Start at pose('look-down') and look(): the default view is the RAW frame with the jaw band. An
object is between the fingers when it crosses the jaw-tip row inside the green band, just right of
the fixed jaw (vision.detections[i].between_jaws / where say so).
  A. Object not in the frame -> analyze_object(label): surveys as high as reachable above the
     current spot, scans +-15/30 deg around the current pan and restores it. plan.feasible -> grab(plan).
     No card -> describe the frame and ask; do not wander.
  B. You see it but the detector does not list it (no conf >= 0.15) -> align_to(label, u, v) with a
     raw pixel on its body and width_cm = the side the jaws close across; then grasp(label).
     grab_at(...) does both.
  C. Detector lists it (conf >= 0.15) -> align_to(label) then grasp(label), or analyze_object -> grab.
  Place -> move_tool above the spot, lower to the pick height, release(); the frame after the retreat
  is the proof. End at pose('standby').
OUTCOMES: aligned -> grasp. not_aligned -> read final.where, one more align_to, then ask. lost ->
look(), re-point, retry once. touch/frozen -> look(), say what was hit, never repeat the move. held ->
report and keep holding. empty/slipped -> release, re-align once, then ask. edge -> replan, never
carry. refused -> read the reason (holder, stop, busy, unreachable: it names the reachable z, go
lower; never pass ignore_table on your own).
BUDGET: an attempt is a jaw close; three closes then stop and ask. Twelve moves without the object
getting closer to the band -> stop and report what you saw.
FACTS: fixed jaw = LEFT finger in the raw view (base side), moving jaw = right, off frame at >= 46 %
open. Raw axes: image-left = toward the base, image-top = world left; object left of the band ->
move_tool(dx<0); object above the row -> move_tool(dy>0). Open the jaws (55 % for a 3 cm side)
BEFORE sliding or descending. Keep z <= 9 cm when x < 18 cm (higher is unreachable near the base;
8 cm works everywhere). Objects standing on something read 1.5-2x too large/far in the overlay:
measure_height(label) or look(height_cm=...) fixes it; align_to does not need it. The detector often
misses small objects above 6 cm: no detection is not absence, your eyes on the raw frame are
evidence. Jaw percent alone cannot tell a thin hold (1.5 %) from a miss (1 %): grasp(label) checks
with vision after the lift. Descend in 1.5 cm steps and look between steps (parallax). Never close
on something that is not between the jaws in the raw view; never plan from a detection under 0.15."""

server = MCPServer("arm-daemon", instructions=INSTRUCTIONS, version="0.1.0")
READ = ToolAnnotations(read_only_hint=True, destructive_hint=False, open_world_hint=False)
MOVE = ToolAnnotations(read_only_hint=False, destructive_hint=True, idempotent_hint=False, open_world_hint=False)
SAFE = ToolAnnotations(read_only_hint=False, destructive_hint=False, idempotent_hint=True, open_world_hint=False)
_open = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def _http(method, path, body=None):
    data = json.dumps(body or {}).encode() if method == "POST" else None
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"} if data is not None else {})
    try:
        with _open.open(req, timeout=TIMEOUT) as r:
            raw = r.read()
            ctype = r.headers.get("Content-Type", "")
    except urllib.error.HTTPError as e:
        raw, ctype = e.read(), e.headers.get("Content-Type", "")
    except Exception as e:
        raise ConnectionError(f"arm-daemon not reachable at {BASE}: {e}. Ask the user to start arm-daemon/start.command.")
    if ctype.startswith("image/"):
        return raw
    try:
        return json.loads(raw)
    except Exception:
        return {"refused": raw.decode(errors="replace")[:200]}


def _result(res, images=()):
    content = [TextContent(type="text", text=json.dumps(res, indent=1))]
    for jpg in images:
        content.append(Image(data=jpg, format="jpeg").to_image_content())
    return CallToolResult(content=content, structured_content=res if isinstance(res, dict) else {"items": res}, is_error=False)


def _error(e):
    return CallToolResult(content=[TextContent(type="text", text=str(e))], is_error=True)


def _frame():
    try:
        r = _http("GET", "/camera/gripper/frame.jpg")
        return r if isinstance(r, bytes) else None
    except Exception:
        return None


def _call(method, path, body=None, with_frame=False):
    try:
        res = _http(method, path, body)
    except ConnectionError as e:
        return _error(e)
    imgs = []
    if with_frame:
        f = _frame()
        if f:
            imgs.append(f)
    return _result(res, imgs)


# --- reads --------------------------------------------------------------------------------------
@server.tool(annotations=READ)
def state() -> CallToolResult:
    """Live daemon state: bus, control.holder (human | agent | leader | released), stop, all six servos
    (deg or pct, goal, load, torque, temp, limits), tool pose (xyz_m, tilt, lowest link z), last motion
    result, cameras health, leader/demo status."""
    return _call("GET", "/state")


def _vision_frame():
    try:
        r = _http("GET", "/vision/frame.jpg")
        return r if isinstance(r, bytes) else None
    except Exception:
        return None


def _raw_band_frame():
    try:
        r = _http("GET", "/vision/raw.jpg")
        return r if isinstance(r, bytes) else None
    except Exception:
        return None


@server.tool(annotations=READ)
def look(raw: bool = False, prompts: str | None = None, overlay: bool = False, height_cm: float | None = None) -> CallToolResult:
    """Gripper-camera view plus vision info and state. DEFAULT = the RAW frame with the jaw geometry
    drawn on it: the fixed jaw tip (left finger, base side), the moving jaw tip for the current
    opening, the jaw-tip row and a green band. An object is between the fingers when it crosses the
    row inside the band, just right of the fixed jaw; each detection is judged (between_jaws, where,
    dx_px/dy_px to the ideal spot) in vision.detections. Raw axes: image-left = toward the base,
    image-top = world left; to move an object RIGHT in this view move the tool toward the base
    (dx<0), to move it DOWN move the tool left (dy>0); align_to does this for you.
    overlay=True: the rotated table view instead (up = forward, left = +y, 1 cm grid, sizes and
    offsets in cm measured on the table plane unless a height is declared; objects standing on
    something read 1.5-2x too large and too far there). raw=True: the plain camera frame.
    prompts: comma-separated object names to detect (persist until changed). height_cm: declare
    the object-top height above the table for those prompts (or the current ones) so sizes and
    offsets are measured on the object's top plane; measure_height() finds it for you."""
    try:
        if prompts is not None:
            _http("POST", "/vision/prompts", {"prompts": prompts})
        if height_cm is not None:
            labels = [p.strip() for p in (prompts or "").replace(";", ",").split(",") if p.strip()]
            if not labels:
                cur = _http("GET", "/vision")
                labels = list((cur.get("prompts") or []) if isinstance(cur, dict) else [])
            if labels:
                _http("POST", "/vision/settings", {"heights": {lab: float(height_cm) for lab in labels}})
        st = _http("GET", "/state")
        vi = _http("GET", "/vision")
    except ConnectionError as e:
        return _error(e)
    out = {"state": st, "vision": vi if isinstance(vi, dict) else {"error": "vision not configured"}}
    if raw:
        img, out["view"] = _frame(), "raw camera frame, no annotations"
    elif overlay:
        img, out["view"] = (_vision_frame() or _frame()), "rotated table overlay (up = forward, left = +y)"
    else:
        img = _raw_band_frame()
        if img is not None:
            out["view"] = "raw frame with the jaw band (left = toward the base, top = world left)"
        else:
            img, out["view"] = _frame(), "raw camera frame (this daemon has no /vision/raw.jpg yet: restart it for the jaw band)"
    return _result(out, [img] if img else [])


@server.tool(annotations=READ)
def poses() -> CallToolResult:
    """Named poses in the library: joints, aliases, via-chain, hold_after."""
    return _call("GET", "/poses")


# --- writes -------------------------------------------------------------------------------------
@server.tool(annotations=MOVE)
def set_servo(joint: str, value: float, speed: float | None = None, ignore_table: bool = False) -> CallToolResult:
    """Move ONE servo to an absolute value (deg; gripper = percent open). joint: name, bus id 1-6 or
    alias pan/lift/elbow/wrist/roll/grip. Other servos keep their goals. Out-of-range values are
    clamped (see clamped). Returns the motion result and a fresh frame."""
    unit = "pct" if str(joint).lower() in ("gripper", "grip", "6") else "deg"
    body = {unit: float(value)}
    if speed is not None:
        body["speed"] = float(speed)
    if ignore_table:
        body["ignore_table"] = True
    return _call("POST", f"/servo/{joint}", body, with_frame=True)


@server.tool(annotations=MOVE)
def nudge(joint: str, delta: float, ignore_table: bool = False) -> CallToolResult:
    """Move ONE servo by a relative amount (clipped to +-15 deg per call). Returns result + frame."""
    body = {"delta": float(delta)}
    if ignore_table:
        body["ignore_table"] = True
    return _call("POST", f"/servo/{joint}/nudge", body, with_frame=True)


@server.tool(annotations=MOVE)
def move(joints: dict[str, float], speed: float | None = None, ignore_table: bool = False) -> CallToolResult:
    """Move several servos in one watched move: {joint: absolute value}. Returns result + frame."""
    body = {"joints": joints}
    if speed is not None:
        body["speed"] = float(speed)
    if ignore_table:
        body["ignore_table"] = True
    return _call("POST", "/move", body, with_frame=True)


@server.tool(annotations=MOVE)
def gripper(pct: float) -> CallToolResult:
    """Open (95) or close (0) the gripper, percent open. Closing on an object stops where it grips."""
    return _call("POST", "/gripper", {"pct": float(pct)}, with_frame=True)


@server.tool(annotations=MOVE)
def pose(name: str, ignore_table: bool = False) -> CallToolResult:
    """Go to a named pose through its via-chain (ready, straight, standby/crouch-tight, look-down,
    rest = torque off, ...). Note: poses also set the library's gripper value."""
    body = {"name": name}
    if ignore_table:
        body["ignore_table"] = True
    return _call("POST", "/pose", body, with_frame=True)


@server.tool(annotations=SAFE)
def stop(reason: str = "stop() tool") -> CallToolResult:
    """Emergency stop: freezes any motion with torque on; every write is refused until a human resumes."""
    return _call("POST", "/stop", {"reason": reason})


@server.tool(annotations=SAFE)
def resume() -> CallToolResult:
    """Ask to clear STOP. Only a human (dashboard) can actually clear it; this reports the refusal."""
    return _call("POST", "/resume")


# --- demonstrations -----------------------------------------------------------------------------
@server.tool(annotations=READ)
def list_demos() -> CallToolResult:
    """Recorded demonstrations (human teleop and agent runs): id, task text, rows, images, times."""
    return _call("GET", "/demos")


@server.tool(annotations=READ)
def find_demos(query: str, limit: int = 3) -> CallToolResult:
    """Search past demonstrations by words (task text, tags, phase names, outcome). Each hit is a
    skill card: outcome, key numbers (grasp_tool_z_cm, grasp_xy_cm, grasp_state joints,
    gripper_open_pct, gripper_closed_pct, lift_height_cm, release_tool_z_cm), phases with times and
    keyframe names. Use it before a task: 'pick box' -> how a human grasped the box and at what height."""
    import urllib.parse
    return _call("GET", f"/demos?q={urllib.parse.quote(query)}&limit={int(limit)}")


@server.tool(annotations=READ)
def demo_summary(demo_id: str) -> CallToolResult:
    """The skill card of one demonstration (phases, key numbers, keyframes, tags, outcome)."""
    return _call("GET", f"/demo/{demo_id}/summary")


@server.tool(annotations=READ)
def read_demo(demo_id: str, every: int = 10, with_images: bool = True, max_images: int = 6) -> CallToolResult:
    """One demonstration: meta plus every N-th row (t, holder, state, action, tool, image name).
    with_images attaches up to max_images of the referenced gripper frames, spread over the demo."""
    try:
        d = _http("GET", f"/demo/{demo_id}")
    except ConnectionError as e:
        return _error(e)
    if not isinstance(d, dict) or "rows" not in d:
        return _result(d)
    rows = d["rows"]
    sampled = rows[::max(1, int(every))]
    out = {"meta": d["meta"], "rows_total": len(rows), "every": every, "rows": sampled}
    try:
        summ = _http("GET", f"/demo/{demo_id}/summary")
        if isinstance(summ, dict) and "phases" in summ:
            out["summary"] = {k: v for k, v in summ.items() if k != "phases"}
            out["summary"]["phases"] = [{k: v for k, v in p.items() if k not in ("start_state", "end_state")} for p in summ["phases"]]
    except Exception:
        pass
    imgs = []
    if with_images:
        named = [r["image"] for r in rows if r.get("image")]
        step = max(1, len(named) // max(1, max_images))
        for name in named[::step][:max_images]:
            try:
                b = _http("GET", f"/demo/{demo_id}/image/{name}")
                if isinstance(b, bytes):
                    imgs.append(b)
            except Exception:
                pass
        out["images_attached"] = [n for n in named[::step][:max_images]]
    return _result(out, imgs)


@server.tool(annotations=MOVE)
def replay_demo(demo_id: str, every: int = 10, speed: float | None = None, ignore_table: bool = False) -> CallToolResult:
    """Re-execute a demonstration: every N-th recorded action becomes a guarded waypoint move.
    Stops at the first frozen/refused waypoint and says which."""
    body = {"every": int(every)}
    if speed is not None:
        body["speed"] = float(speed)
    if ignore_table:
        body["ignore_table"] = True
    return _call("POST", f"/demo/{demo_id}/replay", body, with_frame=True)


@server.tool(annotations=SAFE)
def demo_start(task: str) -> CallToolResult:
    """Start recording a demonstration under this task text (your own moves are recorded too)."""
    return _call("POST", "/demo/start", {"task": task})


@server.tool(annotations=SAFE)
def demo_stop(reported_outcome: str | None = None, notes: str | None = None) -> CallToolResult:
    """Stop the current demonstration recording; returns id, rows, images. Give your own verdict as
    reported_outcome (e.g. success | failed | aborted) and short notes on why: failures are training
    data too, and the label is what makes them usable."""
    body = {k: v for k, v in (("reported_outcome", reported_outcome), ("notes", notes)) if v is not None}
    return _call("POST", "/demo/stop", body)



# --- agent skills (skills.py): synchronized observation, base-frame moves, contact-first grasping ---
_SKILLS = None


def _skills():
    global _SKILLS
    if _SKILLS is None or _SKILLS.base != BASE:
        import skills
        _SKILLS = skills.Skills(BASE)
    return _SKILLS


def _skill_result(res):
    """Skill dict (may carry a raw JPEG under 'frame') -> MCP result with that image attached."""
    res = dict(res)
    frame = res.pop("frame", None)
    return _result(res, [frame] if frame else [])


def _skill(fn, *a, **kw):
    try:
        return _skill_result(fn(*a, **kw))
    except ConnectionError as e:
        return _error(e)


@server.tool(annotations=READ)
def observe() -> CallToolResult:
    """Gripper frame and joints read together: joints, tool_xyz_m, frame_age_s, and stale (true when the
    frame is older than 0.5 s, missing, or the arm is still moving). Decide only from a non-stale frame."""
    return _skill(_skills().observe)


@server.tool(annotations=MOVE)
def goto_xyz(x_m: float, y_m: float, z_m: float, roll_deg: float | None = None, speed: float | None = None, ignore_table: bool = False) -> CallToolResult:
    """Move the fingertips to a base-frame point (metres: x forward, y left, z up), gripper pointing
    down. Refused 'unreachable' when top-down IK cannot reach it. Returns the motion result and tool_xyz_m."""
    return _skill(_skills().goto_xyz, [x_m, y_m, z_m], roll=roll_deg, speed=speed or 20.0, ignore_table=ignore_table)


@server.tool(annotations=MOVE)
def move_tool(dx_m: float = 0.0, dy_m: float = 0.0, dz_m: float = 0.0, speed: float | None = None, ignore_table: bool = False) -> CallToolResult:
    """Relative move of the fingertips in the base frame (metres: x forward/away from the base, y
    left, z up). Small steps (<= 0.03) and look() between them. In the RAW view image-left = toward
    the base (-x) and image-top = world left (+y), so: an object LEFT of the band needs dx < 0 (tool
    toward the base) to come right; an object ABOVE the jaw row needs dy > 0 (tool to the left) to
    come down (checked on the 23 Oct 06:28 grab). A refusal names the highest reachable z above the
    target (x, y): near the base keep z at or below it; 8 cm is reachable everywhere the arm works."""
    """Relative move of the fingertips in the base frame (metres). Small steps (<= 0.03) and observe()
    between them. In the RAW gripper frame image top = world left (+y) and image left = toward the base (-x): an object high in the raw frame needs dy negative to come between the jaws."""
    return _skill(_skills().move_tool, dx=dx_m, dy=dy_m, dz=dz_m, speed=speed or 20.0, ignore_table=ignore_table)


@server.tool(annotations=READ)
def project(u_px: float, v_px: float, z_m: float | None = None) -> CallToolResult:
    """Pixel (u, v) of the RAW gripper frame (640x480) -> base-frame point on the table plane (or on
    plane z_m), using the camera calibration and the current joints. Good for 'the object is at
    pixel (300, 350)': then goto_xyz above that point. Verify with one small move before trusting."""
    return _skill(_skills().project, u_px, v_px, z_m)


@server.tool(annotations=MOVE)
def descend_to_contact(max_cm: float = 8.0, step_cm: float = 0.5, back_off_cm: float = 1.0) -> CallToolResult:
    """Lower the tool straight down in steps until the load/lag guard reports contact, then back off
    back_off_cm. Returns contact (bool), z_contact_m and the new tool_xyz_m. Height comes from touch,
    not from the model (this bypasses the advisory table check; contact is the real floor)."""
    return _skill(_skills().descend_to_contact, max_cm=max_cm, step_cm=step_cm, back_off_cm=back_off_cm)


@server.tool(annotations=MOVE)
def grasp(close_pct: float = 0.0, lift_cm: float = 2.0, label: str | None = None) -> CallToolResult:
    """Close, read the jaw opening, lift a little, read again, and return a verdict with evidence:
    held | empty (jaws closed fully) | slipped (opening changed after the lift) | unknown, plus the frame
    after the lift. With a label the detector also checks that the object rides with the jaws after
    the lift (a thin object can read as fully closed); when the detector sees nothing at all the jaw
    reading decides. Never report an object as held without this."""
    return _skill(_skills().grasp, close_pct=close_pct, lift_cm=lift_cm, label=label)


@server.tool(annotations=MOVE)
def release(open_pct: float = 50.0, retreat_cm: float = 3.0) -> CallToolResult:
    """Open the gripper, retreat upward, and return the frame after the retreat so placement is
    confirmed by observation, not assumed."""
    return _skill(_skills().release, open_pct=open_pct, retreat_cm=retreat_cm)



@server.tool(annotations=MOVE)
def analyze_object(label: str, survey_z_m: float = 0.11) -> CallToolResult:
    """STUDY BEFORE GRABBING when the object is NOT in the frame or the detector sees it well.
    Rises to survey_z_m above the current spot (or as high as is reachable there: near the base
    11 cm is not), finds the object (scans +-15/30 deg around the current pan and restores it when
    nothing is found), measures it with the vision overlay and returns a card (position, size, long
    axis, estimated height) plus a grasp plan: which side the jaws close across (roll 0 = along x,
    90 = along y), jaw opening, approach and grasp heights, feasible (narrowest side <= 7.5 cm).
    Plans only from detections with conf >= 0.15; a weaker hit comes back under low_confidence with
    no plan. Read the card and the frame, then grab(plan). No card but you can see the object
    yourself -> align_to(label, u, v) instead. Moves the arm up and may pan it; never descends."""
    return _skill(_skills().analyze_object, label, survey_z=survey_z_m)


@server.tool(annotations=MOVE)
def grab(plan: dict) -> CallToolResult:
    """Execute a plan from analyze_object (or replan): approach above the target with the planned
    roll, align on the vision offsets (<= 1 cm, up to 3 corrections), open the jaws to the planned
    width, descend to the planned height, close, lift, and evaluate. Returns corrections, the grasp
    verdict with jaw readings, the evaluation (good | edge | empty | slipped) and the frame after
    the lift. On anything but 'good', call replan(plan, evaluation) before trying again."""
    return _skill(_skills().grab, plan)


@server.tool(annotations=MOVE)
def align_to(label: str | None = None, u: float | None = None, v: float | None = None, width_cm: float = 3.0,
             open_pct: float = 55.0, grasp_z_m: float = 0.02, descend: bool = True) -> CallToolResult:
    """PUT THE OBJECT BETWEEN THE FINGERS, from wherever you can see it (no survey needed). Opens
    the jaws to open_pct, slides the tool at the current height until the object crosses the jaw-tip
    row inside the green band (just right of the fixed jaw), then descends in 1.5 cm steps
    re-centering after each until grasp_z_m. Never closes: on 'aligned' call grasp(label). label:
    the detector follows that object when it sees it (conf >= 0.15). u, v: a pixel in the RAW view
    on the object's body, tracked as a blob when the detector does not see it (give both when
    unsure). width_cm: the side the jaws will close across (3 for the carton standing up).
    Outcomes: aligned | not_aligned (says where it is) | lost | touch (contact, backed up 1 cm) |
    refused. Returns the moves made, px_per_cm measured at the object's depth and the raw view."""
    return _skill(_skills().align_to, label=label, u=u, v=v, width_cm=width_cm, open_pct=open_pct, grasp_z_m=grasp_z_m, descend=descend)


@server.tool(annotations=MOVE)
def grab_at(label: str | None = None, u: float | None = None, v: float | None = None, width_cm: float = 3.0,
            open_pct: float = 55.0, grasp_z_m: float = 0.02, lift_cm: float = 3.0) -> CallToolResult:
    """THE WHOLE PICK FOR WHAT YOU CAN SEE: align_to(...) then grasp(label) then evaluate_grasp
    against width_cm. Closes the jaws only after 'aligned'. Returns align, grasp (verdict with jaw
    readings) and evaluation (good | edge | empty | slipped) plus the frame after the lift."""
    return _skill(_skills().grab_at, label=label, u=u, v=v, width_cm=width_cm, open_pct=open_pct, grasp_z_m=grasp_z_m, lift_cm=lift_cm)


@server.tool(annotations=MOVE)
def measure_height(label: str, rise_cm: float = 3.0) -> CallToolResult:
    """How high the object's top is above the table, from parallax: the detector box shrinks as the
    tool rises rise_cm, which gives the camera-to-top distance. Stores the height as the vision
    height for this label so look()/analyze offsets are measured on the object's top plane. Needs
    the detector to see the object from both heights; otherwise says so. The tool stays raised."""
    return _skill(_skills().measure_height, label, rise_cm=rise_cm)


@server.tool(annotations=READ)
def evaluate_grasp(grasp: dict, plan: dict) -> CallToolResult:
    """Judge a grasp result against the plan: the jaws should stop near the opening expected for
    the planned width. good = held at a plausible width; edge = held but far too narrow (a corner
    pinch that will drop); empty; slipped. Pure computation, no motion."""
    return _result(_skills().evaluate_grasp(grasp, plan))


@server.tool(annotations=READ)
def replan(plan: dict, evaluation: dict) -> CallToolResult:
    """Adjust a plan after a poor grasp: empty -> re-survey and 0.5 cm lower; edge -> shift along
    the closing axis toward the object's middle; slipped -> lower and a shorter lift. After 3
    attempts it returns feasible=false with a reason to give the human. Pure computation."""
    return _result(_skills().replan(plan, evaluation))


def main(argv=None):
    global BASE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=BASE)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args(argv)
    BASE = a.url
    if a.list:
        import asyncio
        for t in asyncio.run(server.list_tools()):
            print(f"{t.name:12s} {t.description.splitlines()[0]}")
        return
    server.run("stdio")


if __name__ == "__main__":
    main()
