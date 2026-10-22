# arm-daemon Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** One long-lived process that owns the SO-101 serial bus and camera feeds and exposes them on 127.0.0.1:8800 over HTTP + WebSocket, with a human-only control token and always-on motion guards.

**Architecture:** A `BusWorker` thread owns the serial connection, runs one command at a time from a queue, and publishes a state snapshot between every motion tick. A `CameraWorker` thread proxies cam-server (:8765). An aiohttp app maps HTTP/WS routes to those two workers plus a pure `ControlState` (holder/stop). Hardware is behind a small backend protocol so everything is unit-tested with a fake bus.

**Tech Stack:** Python 3.12 in `tools/arm-lerobot` (uv), `aiohttp` (new runtime dep), `pytest` + `pytest-aiohttp` (new dev deps), existing `scripts/grasp.py`, `scripts/kin.py`, `scripts/arm-go.py`, `lab/arm-vision/pose-library.json`.

**Spec:** `docs/superpowers/specs/2026-10-23-arm-daemon-design.md`

## Global Constraints

- Python `>=3.12,<3.13`, run everything as `cd tools/arm-lerobot && uv run ...`.
- Serial port `/dev/cu.usbmodem5B790812561`; opened only by `BusWorker`, only inside `daemon.py`'s process. Tests never open it.
- Bind `127.0.0.1:8800` only. cam-server is `http://127.0.0.1:8765`; all local HTTP uses a no-proxy client (shell sets `HTTP(S)_PROXY`).
- Joints in degrees, gripper in percent open `0..95`. Limits = `kin._limits(margin=0.8)`.
- Guards: clamp and load/lag always on; table check (`path_min_z < 0.015`) refusable only with `ignore_table: true`.
- `load_max` 350 / `lag_max` 6.0 for ordinary moves; 650 / 10.0 for named-pose hub legs (`ready`, `straight`, `rest`, `look-down`).
- Holder starts `human`. Human = request carries header `X-Arm-Human-Key` equal to `arm-daemon/human.key`.
- Torque is never dropped implicitly: connect/disconnect with `disable_torque=False`.
- This folder is not a git repository: there are no commit steps. Do not `git init`.

## Review Focus

1. A write arriving while a motion runs must be refused `busy`, not queued (Task 3 test `test_second_write_while_moving_is_busy`).
2. `POST /servo/gripper {"deg": 10}` (wrong unit) must be refused `value out of range`/`wrong unit`, never moved (Task 2 test `test_gripper_rejects_deg`).
3. The `arm-agent/STOP` file appearing mid-move must freeze that move and report `by: file` (Task 3 test `test_stop_file_freezes_mid_move`).
4. cam-server down must not affect reads or writes; `/state.cameras.gripper.ok` is `false` and `frame.jpg` is 503 (Task 4 test `test_camera_down_reports_not_ok`).
5. A human request with a wrong key must be treated as agent, not as human, and not leak whether the key exists (Task 5 test `test_wrong_key_is_agent`).

---

## File structure

```
arm-daemon/
  __init__.py
  control.py     ControlState: holder, stop, key check (pure)
  guards.py      servo name resolution, unit/range checks, clamp, table check, MotionResult (pure + kin)
  bus.py         Backend protocol, RealBackend (grasp.bus), BusWorker thread: queue, state, watched move, poses, torque
  cameras.py     CameraWorker thread polling cam-server; frame store
  api.py         aiohttp routes + WS broadcast; builds the app from (control, bus, cameras)
  daemon.py      entry point: key file, workers, app, signals
  API.md         client documentation
  start.command  user double-clicks in Terminal
  smoke.sh       curl checks against a running daemon
  tests/
    conftest.py          FakeBackend, fixtures
    test_control.py
    test_guards.py
    test_bus.py
    test_cameras.py
    test_api.py
```

`human.key` and `logs/` are created at runtime and ignored by tests.

---

### Task 1: Scaffold, dependencies, ControlState

**Files:**
- Modify: `tools/arm-lerobot/pyproject.toml` (add `aiohttp>=3.10` to dependencies; add `[dependency-groups] dev = ["pytest>=8", "pytest-aiohttp>=1.0"]`)
- Create: `arm-daemon/__init__.py` (empty), `arm-daemon/control.py`, `arm-daemon/tests/__init__.py`, `arm-daemon/tests/test_control.py`, `arm-daemon/pytest.ini` (`[pytest] pythonpath = . ..` so `import control` and `import scripts` style imports both resolve; `testpaths = tests`)

**Interfaces:**
- Produces:
  ```python
  class ControlState:
      def __init__(self, key: str, stop_file: Path): ...
      holder: str                  # "human" | "agent"
      since: float
      def is_human(self, header_key: str | None) -> bool   # constant-time compare, None -> False
      def set_holder(self, holder: str, human: bool) -> tuple[bool, str]  # (ok, reason); reason "human key required" | "unknown holder"
      def stop(self, reason: str, by: str) -> None          # by: "human"|"agent"
      def resume(self, human: bool) -> tuple[bool, str]     # deletes stop_file if present; reason "human key required"
      def stop_state(self) -> dict                          # {"set": bool, "reason": str, "by": "human"|"agent"|"file"|""}; checks stop_file each call
      def can_write(self, human: bool) -> tuple[bool, str]  # False with "stop is set" | "human has control"
  ```

- [ ] **Step 1: Add deps and sync**

Run: `cd tools/arm-lerobot && uv add aiohttp>=3.10 && uv add --group dev pytest pytest-aiohttp`
Expected: lock updated, `uv run python -c "import aiohttp, pytest"` prints nothing.

- [ ] **Step 2: Write failing tests** in `arm-daemon/tests/test_control.py`

```python
def test_starts_human(tmp_path):
    c = ControlState("k", tmp_path / "STOP"); assert c.holder == "human"
def test_agent_cannot_take(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.set_holder("agent", human=False) == (False, "human key required"); assert c.holder == "human"
def test_human_gives_and_takes(tmp_path):
    c = ControlState("k", tmp_path / "STOP")
    assert c.set_holder("agent", human=True) == (True, ""); assert c.can_write(human=False) == (True, "")
    assert c.set_holder("human", human=True) == (True, ""); assert c.can_write(human=False) == (False, "human has control")
def test_human_writes_while_agent_holds(tmp_path):
    c = ControlState("k", tmp_path / "STOP"); c.set_holder("agent", True); assert c.can_write(True) == (True, "")
def test_key_check(tmp_path):
    c = ControlState("secret", tmp_path / "STOP")
    assert c.is_human("secret") and not c.is_human("wrong") and not c.is_human(None)
def test_stop_blocks_everyone_until_human_resume(tmp_path):
    c = ControlState("k", tmp_path / "STOP"); c.stop("test", by="agent")
    assert c.can_write(True) == (False, "stop is set"); assert c.resume(human=False) == (False, "human key required")
    assert c.resume(human=True) == (True, ""); assert c.can_write(True) == (True, "")
def test_stop_file_is_honoured_and_cleared(tmp_path):
    f = tmp_path / "STOP"; c = ControlState("k", f); f.touch()
    assert c.stop_state()["by"] == "file"; c.resume(human=True); assert not f.exists()
```

- [ ] **Step 3: Run tests, expect ImportError**

Run: `cd arm-daemon && uv --directory ../tools/arm-lerobot run pytest tests/test_control.py -v`
(From here on, "Run pytest" means this invocation pattern with the test path.)

- [ ] **Step 4: Implement `ControlState` in `arm-daemon/control.py`** per the Interfaces block. Use `hmac.compare_digest`. `set_holder` on `"human"` when currently `"agent"` returns ok; callers (Task 3) handle the freeze.

- [ ] **Step 5: Run pytest, expect 7 passed**

---

### Task 2: Guards and MotionResult

**Files:**
- Create: `arm-daemon/guards.py`, `arm-daemon/tests/test_guards.py`

**Interfaces:**
- Consumes: `scripts/kin.py` (`CHAIN`, `_limits`, `fk`, `tool_pose`), `scripts/arm-go.py` `path_min_z(a, b)` (load via `importlib` as `control.py` does).
- Produces:
  ```python
  JOINTS = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]
  ALIASES = {"pan":"shoulder_pan","lift":"shoulder_lift","shoulder":"shoulder_lift","elbow":"elbow_flex",
             "wrist":"wrist_flex","roll":"wrist_roll","grip":"gripper", "1":"shoulder_pan", ... "6":"gripper"}
  LIMITS: dict[str, tuple[float,float]]   # kin._limits(margin=0.8) + gripper (0.0, 95.0)
  TABLE_Z = 0.015
  HUBS = {"ready","straight","rest","look-down"}
  def resolve_servo(name: str) -> str | None
  def parse_target(servo: str, body: dict) -> tuple[float | None, str]   # value or (None, reason); reason "wrong unit: gripper takes pct" | "wrong unit: <servo> takes deg" | "value out of range" (non-numeric) 
  def clamp(q: dict[str,float]) -> dict[str,float]
  def table_min_z(cur: dict, tgt: dict) -> float                          # path_min_z over kin.CHAIN joints
  def tool_info(q: dict) -> dict   # {"xyz_m":[...], "tilt_from_down_deg": float, "lowest_link_z_m": float}
  @dataclass
  class MotionResult:  # fields exactly as the spec's result object; to_dict()
      id: int; outcome: str; reason: str; requested: dict; clamped: dict; table_min_z_m: float | None
      table_ignored: bool; final: dict; peak_load: int; duration_s: float
  ```
  `tilt_from_down_deg = degrees(acos(-R[2,2]))` of `kin.tool_pose`, as `grasp.ik_or_fail` does.

- [ ] **Step 1: Write failing tests** in `test_guards.py`

```python
def test_resolve_aliases(): assert resolve_servo("pan") == "shoulder_pan"; assert resolve_servo("6") == "gripper"; assert resolve_servo("Wrist_Flex") == "wrist_flex"; assert resolve_servo("foot") is None
def test_gripper_rejects_deg(): assert parse_target("gripper", {"deg": 10}) == (None, "wrong unit: gripper takes pct")
def test_joint_rejects_pct(): assert parse_target("elbow_flex", {"pct": 10}) == (None, "wrong unit: elbow_flex takes deg")
def test_non_numeric(): assert parse_target("elbow_flex", {"deg": "ten"}) == (None, "value out of range")
def test_clamp_reports_limits():
    q = clamp({"shoulder_pan": 500, "gripper": 120}); assert q["shoulder_pan"] == LIMITS["shoulder_pan"][1]; assert q["gripper"] == 95.0
def test_limits_margin(): assert abs(LIMITS["shoulder_pan"][1] - 118.8) < 0.2   # spec value, margin 0.8
def test_table_min_z_rest_is_above_threshold():
    rest = {"shoulder_pan": -93.5, "shoulder_lift": -104.1, "elbow_flex": -95.6, "wrist_flex": 55.2, "wrist_roll": -73.8}
    ready = {j: 0.0 for j in rest}; assert table_min_z(ready, rest) > TABLE_Z
def test_tool_info_look_down():
    ld = {"shoulder_pan": 0.4, "shoulder_lift": -4.4, "elbow_flex": 7.5, "wrist_flex": 88.6, "wrist_roll": 0.0}
    t = tool_info(ld); assert t["tilt_from_down_deg"] < 12 and 0.07 < t["xyz_m"][2] < 0.09
def test_result_roundtrip():
    r = MotionResult(1, "refused", "busy", {}, {}, None, False, {}, 0, 0.0); assert r.to_dict()["outcome"] == "refused"
```

- [ ] **Step 2: Run pytest, expect failures (ImportError)**

- [ ] **Step 3: Implement `guards.py`** per Interfaces. Put the `sys.path.insert` for `scripts/` and the `importlib` load of `arm-go.py` at module top; set `kin.LIMITS = kin._limits(margin=0.8)` before any clamp so `grasp.clamp` (used later) agrees.

- [ ] **Step 4: Run pytest, expect 9 passed**

---

### Task 3: BusWorker

**Files:**
- Create: `arm-daemon/bus.py`, `arm-daemon/tests/conftest.py`, `arm-daemon/tests/test_bus.py`

**Interfaces:**
- Consumes: Task 1 `ControlState`, Task 2 guards, `scripts/grasp.py` (`bus`, `hold`, `settle`, `HZ`), `scripts/arm-go.py` (`load`, `resolve`, `route`), pose library.
- Produces:
  ```python
  class Backend(Protocol):            # everything the worker needs from hardware
      def connect(self) -> None
      def disconnect(self) -> None     # torque untouched
      def read(self) -> dict[str,float]            # deg / pct for all 6
      def read_load(self) -> dict[str,int]
      def read_temp(self) -> dict[str,int]
      def torque_on(self) -> bool
      def hold(self) -> None                       # grasp.hold
      def write_goal(self, q: dict[str,float]) -> None
      def freeze(self) -> None                     # goal := present, torque on
      def disable_torque(self) -> None
      def settle(self, goal: dict) -> dict
  class RealBackend(Backend)           # wraps grasp.bus; read_temp = sync_read("Present_Temperature")
  class BusWorker(threading.Thread):
      def __init__(self, backend: Backend, control: ControlState, poses_path: Path, hz: float = 20.0, log_dir: Path | None = None)
      def snapshot(self) -> dict        # the spec state object minus "cameras"; "t" is the time of the last servo read
      def submit(self, cmd: dict, human: bool) -> MotionResult   # blocks until result; returns refused "busy" immediately if a motion runs
      def on_result(self, cb: Callable[[MotionResult], None]) -> None
      def stop(self) -> None            # thread exit after current tick; backend.disconnect()
  ```
  `cmd` shapes: `{"op":"servo","servo":str,"value":float,"speed":float|None,"ignore_table":bool}`,
  `{"op":"nudge","servo":str,"delta":float,...}` (delta clipped to ±15), `{"op":"move","joints":{...},...}`,
  `{"op":"pose","name":str}`, `{"op":"torque","on":bool,"confirm":bool}`.
  Default speeds: 30 deg/s joints, 60 for gripper, 40 for pose legs.

- [ ] **Step 1: Write `conftest.py` `FakeBackend`**: holds `q` (starts at ready zeros, gripper 0), `write_goal` sets `q` to goal immediately unless `self.block_joint` is set, in which case that joint stays put (produces lag) ; `load` returns `self.load` dict (default 50 each); `torque` flag; records `calls` list (`"hold"`, `"freeze"`, `"disable_torque"`). Fixture `worker(tmp_path)` builds `ControlState("k", tmp_path/"STOP")`, `BusWorker(FakeBackend(), control, POSE_LIB, hz=200)`, starts it, yields, stops it.

- [ ] **Step 2: Write failing tests** in `test_bus.py`

```python
def test_snapshot_has_all_servos_and_tool(worker):
    s = worker.snapshot(); assert set(s["servos"]) == set(JOINTS); assert "lowest_link_z_m" in s["tool"]; assert s["motion"]["state"] == "idle"
def test_servo_move_ok(worker, control):
    control.set_holder("agent", True)
    r = worker.submit({"op":"servo","servo":"elbow_flex","value":10.0}, human=False)
    assert r.outcome == "ok" and abs(r.final["elbow_flex"] - 10.0) < 0.1 and r.clamped["elbow_flex"] == 10.0
def test_agent_refused_when_human_holds(worker):
    r = worker.submit({"op":"servo","servo":"elbow_flex","value":10.0}, human=False); assert (r.outcome, r.reason) == ("refused", "human has control")
def test_clamp_is_reported_not_refused(worker):
    r = worker.submit({"op":"servo","servo":"shoulder_pan","value":500}, human=True); assert r.outcome == "ok" and r.clamped["shoulder_pan"] < 500
def test_table_refused_then_ignored(worker):
    low = {"op":"move","joints":{"shoulder_lift": 60, "elbow_flex": 0, "wrist_flex": 90}}   # tips into the table from ready
    r = worker.submit(low, human=True); assert r.outcome == "refused" and r.reason.startswith("path within")
    r = worker.submit({**low, "ignore_table": True}, human=True); assert r.outcome == "ok" and r.table_ignored
def test_lag_freezes(worker, backend):
    backend.block_joint = "elbow_flex"
    r = worker.submit({"op":"servo","servo":"elbow_flex","value":30.0}, human=True)
    assert r.outcome == "frozen" and "lag" in r.reason and "freeze" in backend.calls
def test_load_freezes(worker, backend):
    backend.load["wrist_flex"] = 400
    r = worker.submit({"op":"servo","servo":"wrist_flex","value":30.0}, human=True); assert r.outcome == "frozen" and "load" in r.reason
def test_gripper_ignores_load_guard(worker, backend):
    backend.load["gripper"] = 400; r = worker.submit({"op":"servo","servo":"gripper","value":50.0}, human=True); assert r.outcome == "ok"
def test_second_write_while_moving_is_busy(worker, backend):
    backend.slow = True   # write_goal sleeps 20 ms per tick so a 30-deg move takes > 100 ms
    t = threading.Thread(target=lambda: worker.submit({"op":"servo","servo":"elbow_flex","value":30.0}, human=True)); t.start(); time.sleep(0.03)
    r = worker.submit({"op":"servo","servo":"wrist_flex","value":5.0}, human=True); assert (r.outcome, r.reason) == ("refused", "busy"); t.join()
def test_stop_file_freezes_mid_move(worker, backend, control, tmp_path):
    backend.slow = True
    t = threading.Thread(target=lambda: setattr(worker, "_last", worker.submit({"op":"servo","servo":"elbow_flex","value":30.0}, human=True))); t.start()
    time.sleep(0.03); (tmp_path / "STOP").touch(); t.join()
    assert worker._last.outcome == "frozen" and worker._last.reason == "stop is set" and control.stop_state()["by"] == "file"
def test_human_takeover_freezes_mid_move(worker, backend, control):
    control.set_holder("agent", True); backend.slow = True
    t = threading.Thread(target=lambda: setattr(worker, "_last", worker.submit({"op":"servo","servo":"elbow_flex","value":30.0}, human=False))); t.start()
    time.sleep(0.03); control.set_holder("human", True); t.join(); assert worker._last.reason == "control taken by human"
def test_frozen_does_not_latch(worker, backend):
    backend.load["wrist_flex"] = 400; worker.submit({"op":"servo","servo":"wrist_flex","value":30.0}, human=True)
    backend.load["wrist_flex"] = 50; assert worker.submit({"op":"servo","servo":"wrist_flex","value":5.0}, human=True).outcome == "ok"
def test_nudge_clips_to_15(worker):
    r = worker.submit({"op":"nudge","servo":"elbow_flex","delta":40}, human=True); assert abs(r.final["elbow_flex"] - 15) < 0.1
def test_pose_ready_then_rest_releases_torque(worker, backend):
    assert worker.submit({"op":"pose","name":"ready"}, human=True).outcome == "ok"
    r = worker.submit({"op":"pose","name":"rest"}, human=True); assert r.outcome == "ok" and "disable_torque" in backend.calls and backend.torque is False
def test_pose_hub_legs_use_hub_limits(worker, backend):
    backend.load["elbow_flex"] = 500   # > 350, < 650
    assert worker.submit({"op":"pose","name":"straight"}, human=True).outcome == "ok"
def test_torque_off_guard(worker, backend):
    r = worker.submit({"op":"torque","on":False}, human=True); assert r.outcome == "refused" and r.reason == "arm is raised; pass confirm"
    assert worker.submit({"op":"torque","on":False,"confirm":True}, human=True).outcome == "ok" and backend.torque is False
def test_unknown_pose(worker): assert worker.submit({"op":"pose","name":"banana"}, human=True).reason.startswith("unknown pose")
def test_hold_before_write_when_torque_off(worker, backend):
    backend.torque = False; worker.submit({"op":"servo","servo":"elbow_flex","value":5.0}, human=True); assert "hold" in backend.calls
```

- [ ] **Step 3: Run pytest, expect failures**

- [ ] **Step 4: Implement `bus.py`**

Thread loop: `while not self._exit: cmd = queue.get(timeout=1/hz) or None; if cmd: self._run(cmd) else: self._poll()`. `_poll` reads position, load, torque every tick and temperature every 50th tick; writes `self._state` under a lock. `submit` returns `MotionResult(refused, "busy")` without enqueuing when `self._moving` is set; otherwise enqueues `(cmd, human, Future)` and waits.

`_run` order: `control.can_write(human)` → resolve/parse (`guards`) → `backend.hold()` if `not backend.torque_on()` → build `requested`/`clamped` → table check (skip for gripper-only) → `_watched_move(goal, speed, load_max, lag_max)`.

`_watched_move` is `grasp.move_watch`'s loop with four changes: (1) after every tick, refresh the state snapshot with the latest `read()` so `/state` and the WebSocket move during motion; (2) every 4th tick also check `control.stop_state()["set"]` and, for agent moves, `control.holder == "human"`; on either call `backend.freeze()` and return `frozen` with reason `stop is set` / `control taken by human`; (3) load guard ignores `gripper`; (4) gripper-only targets use `backend.write_goal` eased over `max(dist/speed, 0.3)` s with no guards, then `settle` is skipped. On success call `backend.settle(goal)`.

`pose`: `name_ = arm_go.resolve(...)` (catch `SystemExit` → refused `unknown pose <name>`); `plan = arm_go.route(cur, name_, poses)`; refuse if any leg's `z < TABLE_Z` unless `ignore_table`; run legs with hub limits for `HUBS`; a frozen leg stops the plan with reason `stopped on the way to <leg>: <reason>`; afterwards if `not poses[name_].get("hold_after", True)` and `tool_info(final)["lowest_link_z_m"] <= 0.05` call `backend.disable_torque()`.

`torque` off: refuse `arm is raised; pass confirm` when `lowest_link_z_m > 0.05` and not `confirm`.

Logging: if `log_dir`, append one JSON line per command and per result to `log_dir/<YYYYMMDD>.jsonl`.

- [ ] **Step 5: Run pytest, expect 19 passed**

---

### Task 4: CameraWorker

**Files:**
- Create: `arm-daemon/cameras.py`, `arm-daemon/tests/test_cameras.py`

**Interfaces:**
- Produces:
  ```python
  class CameraWorker(threading.Thread):
      def __init__(self, base_url: str = "http://127.0.0.1:8765", names=("gripper","mac","phone","ipad"), hz: float = 10.0)
      def frame(self, name: str) -> tuple[bytes, float] | None      # (jpeg, age_s)
      def health(self) -> dict      # {"gripper": {"age_s": float|None, "ok": bool, "camera": str|None}, ...}
      def stop(self) -> None
  ```
  Uses `urllib.request.build_opener(urllib.request.ProxyHandler({}))`. `ok` is `True` when the last fetch succeeded within 2 s and cam-server's own `frame_age_s` for that camera is `< 5`.

- [ ] **Step 1: Write failing tests** with a fake cam-server (`http.server` in a thread on an ephemeral port serving `/health` JSON and `/<cam>/frame.jpg` bytes `b"\xff\xd8fake"`; a flag makes it return 503 or refuse connections):

```python
def test_frame_and_health(fake_cam): w = CameraWorker(fake_cam.url, names=("gripper",), hz=50); w.start(); time.sleep(0.1)
    jpg, age = w.frame("gripper"); assert jpg.startswith(b"\xff\xd8") and age < 1; assert w.health()["gripper"]["ok"]; w.stop()
def test_camera_down_reports_not_ok(fake_cam): fake_cam.down = True; w = CameraWorker(fake_cam.url, names=("gripper",), hz=50); w.start(); time.sleep(0.1)
    assert w.frame("gripper") is None and w.health()["gripper"]["ok"] is False; w.stop()
def test_stale_frame_not_ok(fake_cam): fake_cam.frame_age = 3000.0; ...; assert w.health()["gripper"]["ok"] is False
```

- [ ] **Step 2: Run pytest, expect failures**
- [ ] **Step 3: Implement `cameras.py`** per Interfaces; fetch timeout 1.0 s; never raise out of the loop.
- [ ] **Step 4: Run pytest, expect 3 passed**

---

### Task 5: HTTP + WebSocket API

**Files:**
- Create: `arm-daemon/api.py`, `arm-daemon/tests/test_api.py`

**Interfaces:**
- Consumes: Tasks 1, 3, 4.
- Produces: `def build_app(control: ControlState, bus: BusWorker, cams: CameraWorker) -> aiohttp.web.Application` with routes exactly as the spec tables (`/state`, `/servo/{name}` GET/POST, `/servo/{name}/nudge`, `/move`, `/gripper`, `/pose`, `/poses`, `/torque`, `/control` POST, `/stop`, `/resume`, `/camera/{cam}/frame.jpg`, `/camera/{cam}/stream`, `/cameras`, `/health`, `/ws`). Human flag per request: `control.is_human(request.headers.get("X-Arm-Human-Key"))`. Write handlers call `bus.submit` via `loop.run_in_executor`. Responses: `MotionResult.to_dict()` with HTTP 200 for `ok`/`frozen`, 409 for `refused` (body still the result). `/control` and `/resume` failures → 403 `{"refused": reason}`. Unknown servo → 404 `{"refused": "unknown servo <name>"}`. `/ws`: on connect send snapshot + cameras; then every `1/20` s send `{"type":"state", ...}` and on each `bus.on_result` callback send `{"type":"result", ...}`.

- [ ] **Step 1: Write failing tests** using `aiohttp` test client with `FakeBackend` + `FakeCam` fixtures from earlier tasks:

```python
async def test_state_shape(client): s = await (await client.get("/state")).json(); assert {"t","bus","control","stop","servos","tool","motion","cameras"} <= set(s)
async def test_agent_write_refused_409(client): r = await client.post("/servo/elbow", json={"deg": 5}); assert r.status == 409 and (await r.json())["reason"] == "human has control"
async def test_human_write_ok(client, key): r = await client.post("/servo/elbow", json={"deg": 5}, headers={"X-Arm-Human-Key": key}); assert r.status == 200 and (await r.json())["outcome"] == "ok"
async def test_wrong_key_is_agent(client): r = await client.post("/servo/elbow", json={"deg": 5}, headers={"X-Arm-Human-Key": "nope"}); assert r.status == 409 and (await r.json())["reason"] == "human has control"
async def test_control_switch(client, key):
    assert (await client.post("/control", json={"holder":"agent"})).status == 403
    assert (await client.post("/control", json={"holder":"agent"}, headers={"X-Arm-Human-Key": key})).status == 200
    assert (await client.post("/servo/elbow", json={"deg": 5})).status == 200
async def test_stop_and_resume(client, key):
    await client.post("/stop", json={"reason":"t"}); assert (await client.post("/servo/elbow", json={"deg":5}, headers={"X-Arm-Human-Key": key})).status == 409
    assert (await client.post("/resume")).status == 403; assert (await client.post("/resume", headers={"X-Arm-Human-Key": key})).status == 200
async def test_unknown_servo_404(client): assert (await client.get("/servo/foot")).status == 404
async def test_gripper_alias_and_unit(client, key): r = await client.post("/gripper", json={"pct": 40}, headers={"X-Arm-Human-Key": key}); assert (await r.json())["final"]["gripper"] == 40
async def test_poses_lists_library(client): assert "ready" in (await (await client.get("/poses")).json())
async def test_frame_and_503(client, fake_cam): assert (await client.get("/camera/gripper/frame.jpg")).status == 200; fake_cam.down = True; await asyncio.sleep(0.2); assert (await client.get("/camera/gripper/frame.jpg")).status == 503
async def test_ws_streams_state_and_result(client, key):
    ws = await client.ws_connect("/ws"); first = await ws.receive_json(); assert first["type"] == "state"
    await client.post("/servo/elbow", json={"deg": 3}, headers={"X-Arm-Human-Key": key})
    msgs = [await ws.receive_json() for _ in range(10)]; assert any(m["type"] == "result" for m in msgs); await ws.close()
```

- [ ] **Step 2: Run pytest, expect failures**
- [ ] **Step 3: Implement `api.py`** per Interfaces. MJPEG stream: `multipart/x-mixed-replace; boundary=f`, send a part whenever `cams.frame()` age changes, 20 ms sleep otherwise.
- [ ] **Step 4: Run pytest, expect 11 passed**

---

### Task 6: Entry point, start script, smoke test, API.md

**Files:**
- Create: `arm-daemon/daemon.py`, `arm-daemon/start.command`, `arm-daemon/smoke.sh`, `arm-daemon/API.md`

**Interfaces:**
- `daemon.py` args: `--port 8800`, `--cam-url http://127.0.0.1:8765`, `--hz 20`, `--fake` (use `FakeBackend` for a dry run with no serial port). On start: refuse to run if `lsof -t /dev/cu.usbmodem5B790812561` lists another pid (print it and exit 2); create `human.key` (32 hex chars, mode 600) if missing; start `CameraWorker`, `BusWorker(RealBackend(), ...)` with `stop_file = ROOT/"arm-agent/STOP"`, `poses_path = ROOT/"lab/arm-vision/pose-library.json"`, `log_dir = ROOT/"arm-daemon/logs"`; `web.run_app` on 127.0.0.1; SIGTERM/SIGINT → `bus.stop()` (finishes the tick, disconnects torque-on).
- `start.command`: `cd` to repo, `exec uv --directory tools/arm-lerobot run python -B ../../arm-daemon/daemon.py "$@"`.
- `smoke.sh`: reads `human.key`, runs with `--noproxy '*'`: `/health`, `/state`, agent `POST /servo/elbow {"deg": +2 relative via /nudge}` expecting 409, human nudge `+2` then `-2` expecting `ok`, `GET /camera/gripper/frame.jpg` expecting 200 or 503; prints `PASS`/`FAIL` per line, exit 1 on any FAIL.

- [ ] **Step 1: Implement `daemon.py`, `start.command` (chmod +x), `smoke.sh` (chmod +x)**

- [ ] **Step 2: Dry run with the fake backend**

Run: `cd tools/arm-lerobot && uv run python -B ../../arm-daemon/daemon.py --fake --port 8801 &` then `ARM_URL=http://127.0.0.1:8801 arm-daemon/smoke.sh`
Expected: all lines `PASS` (camera line may be 503 PASS if cam-server is down); kill the daemon afterwards.

- [ ] **Step 3: Write `API.md`** with sections: Start/stop (the `.command`, the port, the key file, "never start from a Claude shell or a watchdog"); Control model (5 lines: holder, human key header, agent refusal, takeover freezes, STOP anyone/resume human); Endpoints (one `curl --noproxy '*'` example and one example response per endpoint, copied from the dry run); Outcome and reason vocabulary (the full list from the spec plus `busy`, `wrong unit: ...`, `unknown pose ...`, `arm is raised; pass confirm`, `stopped on the way to <leg>: ...`); WebSocket message shapes; Building a client (MCP: one tool per write endpoint, no key; dashboard: read key from disk, open `/ws`, send `/control`).

- [ ] **Step 4: Real-arm smoke (user present)**

Precondition: no `run_runtime.py`, `keep-arm-up.sh`, `servo-server.py` or `cam-server`-independent bus user running (`lsof -t /dev/cu.usbmodem5B790812561` empty). User double-clicks `arm-daemon/start.command`. Run `arm-daemon/smoke.sh`.
Expected: all `PASS`; `/state` shows the live pose; the human nudge moves the elbow 2° and back.

- [ ] **Step 5: Full test run**

Run: `cd arm-daemon && uv --directory ../tools/arm-lerobot run pytest -v`
Expected: 49 passed.

---

## Self-review notes

- Spec coverage: process/threads (T3, T4, T6), state object (T3 snapshot + T5 cameras merge), control token (T1, T5), STOP incl. file (T1, T3), writes table incl. aliases, nudge clip, pose routing with `hold_after`, torque guard on lowest link (T3), guards order and non-latching freeze (T3), busy refusal (T3), reads table incl. MJPEG and WS (T5), API.md and smoke (T6), lsof refusal at start (T6), margin 0.8 (T2).
- Names used across tasks: `ControlState`, `MotionResult`, `BusWorker.submit/snapshot/on_result`, `CameraWorker.frame/health`, `build_app`; checked consistent.
- Not covered by tests, left to the real-arm smoke: serial timing at 20 Hz under motion, temperature read support on this firmware (if `Present_Temperature` is unsupported, report `temp_c: null`).
