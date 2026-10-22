"""BusWorker: the one thread that owns the servo bus. Runs one command at a time from a queue,
polls every servo between commands, and refreshes the state snapshot after every motion tick so
readers never see a stale pose during a move. Hardware sits behind the Backend protocol."""
import hashlib
import json
import queue
import re
import threading
import time
from pathlib import Path
from typing import Callable, Protocol

import guards
from guards import HUBS, JOINTS, LIMITS, TABLE_Z, MotionResult, arm_go, kin

MOTION_HZ = 50
DEFAULT_SPEED = {"joint": 30.0, "gripper": 60.0, "pose": 40.0}
LOAD_MAX, LAG_MAX = 350, 6.0
HUB_LOAD_MAX, HUB_LAG_MAX = 650, 10.0
NUDGE_MAX = 15.0
SPEED_MAX = 90.0
TORQUE_OFF_MAX_Z = 0.05
TELEOP_HZ = 30
TELEOP_STEP_MAX = 15.0      # deg per tick toward the leader (LeRobot's max_relative_target)
REPLAY_EVERY = 10           # one waypoint per N recorded rows


class Backend(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def read(self) -> dict: ...
    def read_load(self) -> dict: ...
    def read_temp(self) -> dict: ...
    def torque_on(self) -> bool: ...
    def hold(self) -> None: ...
    def write_goal(self, q: dict) -> None: ...
    def freeze(self) -> None: ...
    def disable_torque(self) -> None: ...
    def settle(self, goal: dict) -> dict: ...


class RealBackend:
    """Wraps scripts/grasp.py's FeetechMotorsBus. Torque is never dropped implicitly."""

    def __init__(self):
        import grasp
        self.g = grasp
        self.bus = grasp.bus

    def connect(self):
        self.bus.connect()
        if not self.bus.is_calibrated:
            self.bus.disconnect(disable_torque=False)
            raise RuntimeError("calibration mismatch")

    def disconnect(self):
        self.bus.disconnect(disable_torque=False)

    def read(self):
        return {k: float(v) for k, v in self.bus.sync_read("Present_Position").items()}

    def read_load(self):
        return {k: int(v) for k, v in self.bus.sync_read("Present_Load", num_retry=2).items()}

    def read_temp(self):
        try:
            return {k: int(v) for k, v in self.bus.sync_read("Present_Temperature", normalize=False, num_retry=1).items()}
        except Exception:
            return {}

    def torque_on(self):
        return self.g.torque_on()

    def hold(self):
        self.g.hold()

    def write_goal(self, q):
        self.bus.sync_write("Goal_Position", q)

    def freeze(self):
        self.bus.sync_write("Goal_Position", self.bus.sync_read("Present_Position", normalize=False), normalize=False)

    def disable_torque(self):
        self.bus.disable_torque(num_retry=3)

    def settle(self, goal):
        return {k: float(v) for k, v in self.g.settle(goal).items()}


class Leader(Protocol):
    """The leader arm: read only."""
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def read(self) -> dict: ...


class RealLeader:
    """LeRobot SO101Leader, calibration id hackaiwoo_leader."""

    def __init__(self, port: str, cal_id: str = "hackaiwoo_leader"):
        from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
        self.dev = SO101Leader(SO101LeaderConfig(port=port, id=cal_id))

    def connect(self):
        self.dev.connect()

    def disconnect(self):
        try:
            self.dev.disconnect()
        except Exception:
            pass

    def read(self):
        return {k.removesuffix(".pos"): float(v) for k, v in self.dev.get_action().items() if k.endswith(".pos")}


class _Stop(Exception):
    def __init__(self, reason):
        self.reason = reason


class BusWorker(threading.Thread):
    def __init__(self, backend: Backend, control, poses_path: Path, hz: float = 20.0, log_dir: Path | None = None,
                 leader: Leader | None = None, frame_fn: Callable | None = None, demo_dir: Path | None = None):
        super().__init__(name="bus", daemon=True)
        self.backend, self.control = backend, control
        self.leader, self.frame_fn = leader, frame_fn
        self.demo_dir = Path(demo_dir) if demo_dir else None
        self._leader_on = False
        self._demo = None
        self._demo_lock = threading.Lock()
        self.poses = json.loads(Path(poses_path).read_text())["poses"]
        self.hz = hz
        self.log_dir = Path(log_dir) if log_dir else None
        self._q: queue.Queue = queue.Queue()
        self._exit = threading.Event()
        self._lock = threading.Lock()
        self._moving = False
        self._next_id = 1
        self._callbacks: list[Callable[[MotionResult], None]] = []
        self._temps: dict = {}
        self._poll_n = 0
        self._connected = False
        self._fatal = ""            # set when the bus cannot be used (connect failed): writes refused
        self._state = {"t": 0.0, "bus": "starting", "servos": {}, "tool": {}, "motion": {"id": 0, "state": "idle", "result": None}}

    # --- public ---------------------------------------------------------------------------------
    def snapshot(self) -> dict:
        with self._lock:
            s = json.loads(json.dumps(self._state))
        s["control"] = {"holder": self.control.holder, "since": self.control.since}
        s["stop"] = self.control.stop_state()
        s["leader"] = {"configured": self.leader is not None, "active": self._leader_on}
        s["demo"] = self.demo_active()
        return s

    def submit(self, cmd: dict, human: bool) -> MotionResult:
        req = self._describe(cmd)
        with self._lock:
            mid = self._next_id
            self._next_id += 1
            if self._exit.is_set():
                why = "daemon stopping"
            elif self._fatal:
                why = f"bus error: {self._fatal}"
            elif self._moving:
                why = "busy"
            else:
                why = ""
                self._moving = True          # reserve the slot here, under the lock
        if why:
            return self._finish(MotionResult(mid, "refused", why, req, {}, None, False, self._last_pose(), 0, 0.0), cmd)
        done = threading.Event()
        box = {}
        self._q.put((mid, cmd, human, done, box))
        if not done.wait(timeout=600):
            box["r"] = MotionResult(mid, "frozen", "error: command timed out", req, {}, None, False, self._last_pose(), 0, 0.0)
        return box["r"]

    @staticmethod
    def _describe(cmd: dict) -> dict:
        """What the client asked for, in the shape later guards report: joint->value, pose or torque."""
        op = cmd.get("op")
        if op == "pose":
            return {"pose": cmd.get("name")}
        if op == "replay":
            return {"replay": cmd.get("id")}
        if op == "torque":
            return {"torque": bool(cmd.get("on", True))}
        if op == "move":
            j = cmd.get("joints")
            return {str(k): v for k, v in j.items()} if isinstance(j, dict) else {}
        s = guards.resolve_servo(cmd.get("servo", "")) or str(cmd.get("servo"))
        if op == "nudge":
            return {s: {"delta": cmd.get("delta")}}
        body = cmd.get("body")
        if isinstance(body, dict) and body:
            return {s: next(iter(body.values()))}
        return {s: cmd.get("value")}

    def _last_pose(self):
        with self._lock:
            sv = self._state.get("servos", {})
        return {j: e.get("deg", e.get("pct")) for j, e in sv.items()}

    def on_result(self, cb: Callable[[MotionResult], None]) -> None:
        self._callbacks.append(cb)

    def stop(self) -> None:
        self._exit.set()
        self.join(timeout=10)

    # --- thread ---------------------------------------------------------------------------------
    def _connect(self):
        try:
            self.backend.connect()
            self._connected = True
            self._fatal = ""
            self._set_bus("ok")
        except Exception as e:
            self._connected = False
            self._fatal = str(e)
            self._set_bus(f"error:{e}")

    def _sync_release(self):
        """holder 'released' = the bus is handed to an external process (LeRobot's own teleop or
        record): disconnect (torque untouched) and stop polling; any other holder reconnects.
        holder 'leader' = teleop through the daemon: connect the leader arm and mirror it."""
        holder = self.control.holder
        want = holder != "released"
        if want and not self._connected and not self._fatal:
            self._connect()
        elif not want and self._connected:
            try:
                self.backend.disconnect()
            except Exception:
                pass
            self._connected = False
            self._set_bus("released")
        want_leader = holder == "leader" and self._connected
        if want_leader and not self._leader_on:
            if self.leader is None:
                self._set_bus("error:no leader arm configured")
                return
            try:
                self.leader.connect()
                if not self.backend.torque_on():
                    self.backend.hold()
                self._leader_on = True
                self._set_bus("ok")
            except Exception as e:
                self._set_bus(f"error:leader {e}")
        elif not want_leader and self._leader_on:
            try:
                self.leader.disconnect()
            except Exception:
                pass
            self._leader_on = False
            if self._connected:
                self._set_bus("ok")

    def run(self):
        self._connect()
        while not self._exit.is_set():
            self._sync_release()
            if self._leader_on:
                self._teleop_tick()
                try:
                    item = self._q.get(timeout=1.0 / TELEOP_HZ)
                except queue.Empty:
                    continue
            else:
                try:
                    item = self._q.get(timeout=1.0 / self.hz)
                except queue.Empty:
                    if self._connected:
                        self._poll()
                    continue
            mid, cmd, human, done, box = item
            with self._lock:
                self._state["motion"] = {"id": mid, "state": "moving", "result": None}
            t0 = time.time()
            try:
                r = self._run(mid, cmd, human)
            except Exception as e:
                try:
                    self.backend.freeze()
                except Exception:
                    pass
                r = MotionResult(mid, "frozen", f"error: {e}", cmd, {}, None, False, self._safe_read(), 0, 0.0)
            r.duration_s = round(time.time() - t0, 2)
            r.requested = r.requested or self._describe(cmd)
            with self._lock:
                self._moving = False
                self._state["motion"] = {"id": mid, "state": "frozen" if r.outcome == "frozen" else "idle", "result": r.to_dict()}
            box["r"] = r
            done.set()
            self._finish(r, cmd)
        while True:                      # nothing queued may wait forever on a thread that is gone
            try:
                mid, cmd, human, done, box = self._q.get_nowait()
            except queue.Empty:
                break
            box["r"] = MotionResult(mid, "refused", "daemon stopping", self._describe(cmd), {}, None, False, self._last_pose(), 0, 0.0)
            done.set()
        if self._leader_on:
            try:
                self.leader.disconnect()
            except Exception:
                pass
        self.demo_stop()
        if self._connected:
            try:
                self.backend.disconnect()
            except Exception:
                pass

    # --- state ----------------------------------------------------------------------------------
    def _set_bus(self, text):
        with self._lock:
            self._state["bus"] = text

    def _safe_read(self):
        try:
            return {k: round(float(v), 1) for k, v in self.backend.read().items()}
        except Exception:
            return {}

    def _poll(self):
        try:
            q = self.backend.read()
            load = self.backend.read_load()
            torque = self.backend.torque_on()
            self._poll_n += 1
            if self._poll_n % 50 == 1:
                self._temps = self.backend.read_temp()
        except Exception as e:
            self._set_bus(f"error:{e}")
            return
        self._update(q, load, torque)

    def _update(self, q, load=None, torque=None, goal=None):
        with self._lock:
            prev = self._state["servos"]
            servos = {}
            for j in JOINTS:
                lo, hi = LIMITS[j]
                e = {"load": int(load[j]) if load and j in load else prev.get(j, {}).get("load", 0),
                     "torque": bool(torque) if torque is not None else prev.get(j, {}).get("torque", True),
                     "temp_c": self._temps.get(j)}
                g = goal[j] if goal and j in goal else prev.get(j, {}).get("goal_deg", prev.get(j, {}).get("goal_pct", q[j]))
                if j == "gripper":
                    e.update(pct=round(float(q[j]), 1), goal_pct=round(float(g), 1), min_pct=lo, max_pct=hi)
                else:
                    e.update(deg=round(float(q[j]), 1), goal_deg=round(float(g), 1), min_deg=round(lo, 1), max_deg=round(hi, 1))
                servos[j] = e
            self._state["servos"] = servos
            self._state["tool"] = guards.tool_info(q)
            self._state["t"] = time.time()
            self._state["bus"] = "ok"

    # --- teleop ---------------------------------------------------------------------------------
    def _teleop_tick(self):
        """One leader -> follower step: clamp to limits, cap the step, write, read back, record."""
        try:
            a = self.leader.read()
            cur = self.backend.read()
            if self.control.stop_state()["set"]:
                self._update(cur, None, True)
                return
            goal = {}
            for j in JOINTS:
                if j in a:
                    d = max(-TELEOP_STEP_MAX, min(TELEOP_STEP_MAX, a[j] - cur[j]))
                    goal[j] = cur[j] + d
            goal = guards.clamp(goal)
            self.backend.write_goal(goal)
            meas = self.backend.read()
            self._update(meas, None, True, goal=goal)
            self._record(meas, goal)
        except Exception as e:
            self._set_bus(f"error:leader {e}")
            try:
                self.leader.disconnect()
            except Exception:
                pass
            self._leader_on = False

    # --- demos ----------------------------------------------------------------------------------
    def demo_start(self, task: str) -> dict | None:
        if not self.demo_dir:
            return None
        with self._demo_lock:
            if self._demo:
                return None
            slug = re.sub(r"[^a-z0-9]+", "-", task.lower()).strip("-")[:40] or "demo"
            did = time.strftime("%Y%m%d-%H%M%S") + "-" + slug
            d = self.demo_dir / did
            (d / "gripper").mkdir(parents=True, exist_ok=True)
            meta = {"id": did, "task": task, "started": time.time(), "fps_teleop": TELEOP_HZ}
            (d / "meta.json").write_text(json.dumps(meta))
            self._demo = {"id": did, "dir": d, "task": task, "f": open(d / "frames.jsonl", "a"), "rows": 0, "images": 0,
                          "last_hash": None, "started": meta["started"]}
            return {"id": did, "task": task}

    def demo_stop(self, extra: dict | None = None) -> dict | None:
        """extra: any JSON-able fields the client wants in meta.json (outcome, notes, ...)."""
        with self._demo_lock:
            if not self._demo:
                return None
            dm = self._demo
            self._demo = None
        dm["f"].close()
        meta = json.loads((dm["dir"] / "meta.json").read_text())
        meta.update({k: v for k, v in (extra or {}).items() if k not in ("id", "task", "started", "rows", "images")})
        meta.update(ended=time.time(), rows=dm["rows"], images=dm["images"])
        (dm["dir"] / "meta.json").write_text(json.dumps(meta))
        out = {**meta, "dir": str(dm["dir"])}
        try:                                   # skill card: phases, key numbers, keyframes, tags
            import demolib
            summ = demolib.summarize(dm["dir"])
            out.update(outcome=summ["outcome"], phases=[p["name"] for p in summ["phases"]], key=summ["key"], tags=summ["tags"])
        except Exception as e:
            out["summary_error"] = str(e)
        return out

    def demo_active(self):
        dm = self._demo
        return {"id": dm["id"], "task": dm["task"], "rows": dm["rows"], "images": dm["images"]} if dm else None

    def _record(self, state, action):
        with self._demo_lock:
            dm = self._demo
            if not dm:
                return
            row = {"t": round(time.time(), 4), "holder": self.control.holder,
                   "state": self._round(state), "action": self._round(action),
                   "tool": guards.tool_info(state)}
            if self.frame_fn:
                try:
                    fr = self.frame_fn()
                    if fr:
                        jpg = fr[0]
                        h = hashlib.md5(jpg).digest()
                        if h != dm["last_hash"]:
                            name = f"gripper/{dm['images']:06d}.jpg"
                            (dm["dir"] / name).write_bytes(jpg)
                            row["image"] = name
                            dm["images"] += 1
                            dm["last_hash"] = h
                except Exception:
                    pass
            dm["f"].write(json.dumps(row) + "\n")
            dm["rows"] += 1
            if dm["rows"] % 30 == 0:
                dm["f"].flush()

    def list_demos(self) -> list:
        if not self.demo_dir or not self.demo_dir.exists():
            return []
        out = []
        for d in sorted(self.demo_dir.iterdir()):
            m = d / "meta.json"
            if m.exists():
                try:
                    out.append(json.loads(m.read_text()))
                except Exception:
                    pass
        return out

    def demo(self, did: str) -> dict | None:
        if not self.demo_dir or "/" in did or ".." in did:
            return None
        d = self.demo_dir / did
        if not (d / "meta.json").exists():
            return None
        rows = []
        f = d / "frames.jsonl"
        if f.exists():
            rows = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
        return {"meta": json.loads((d / "meta.json").read_text()), "rows": rows}

    def demo_file(self, did: str, name: str) -> Path | None:
        if not self.demo_dir or "/" in did or ".." in did or ".." in name:
            return None
        p = self.demo_dir / did / name
        return p if p.is_file() else None

    def _replay(self, mid, cmd, human):
        did = str(cmd.get("id", ""))
        req = {"replay": did}
        dm = self.demo(did)
        if dm is None:
            return MotionResult(mid, "refused", f"unknown demo {did}", req, {}, None, False, self._safe_read(), 0, 0.0)
        every = max(1, int(cmd.get("every") or REPLAY_EVERY))
        rows = dm["rows"]
        if not rows:
            return MotionResult(mid, "refused", "demo has no rows", req, {}, None, False, self._safe_read(), 0, 0.0)
        wps = [r["action"] for r in rows[::every]]
        if rows[-1]["action"] is not wps[-1]:
            wps.append(rows[-1]["action"])
        if not self.backend.torque_on():
            self.backend.hold()
        speed, why = self._speed(cmd, DEFAULT_SPEED["joint"])
        if why:
            return MotionResult(mid, "refused", why, req, {}, None, False, self._safe_read(), 0, 0.0)
        peak_all, final, ignored = 0, self._safe_read(), bool(cmd.get("ignore_table"))
        for k, wp in enumerate(wps):
            cur = self.backend.read()
            goal = {**cur, **guards.clamp({j: v for j, v in wp.items() if j in JOINTS})}
            zmin = guards.table_min_z(cur, goal)
            if zmin < TABLE_Z and not ignored:
                return MotionResult(mid, "refused", f"path within {zmin*100:.1f} cm of table at waypoint {k}", req, {}, round(zmin, 4), False, self._round(cur), peak_all, 0.0)
            outcome, reason, peak, final = self._watched_move(goal, speed, LOAD_MAX, LAG_MAX, human)
            peak_all = max(peak_all, peak)
            if outcome != "ok":
                return MotionResult(mid, outcome, f"stopped at waypoint {k}/{len(wps)}: {reason}", req, {}, None, ignored, final, peak_all, 0.0)
        return MotionResult(mid, "ok", f"{len(wps)} waypoints", req, {}, None, ignored, final, peak_all, 0.0)

    # --- commands -------------------------------------------------------------------------------
    def _run(self, mid, cmd, human) -> MotionResult:
        op = cmd.get("op")
        req = self._describe(cmd)
        ok, why = self.control.can_write(human)
        if not ok:
            return MotionResult(mid, "refused", why, req, {}, None, False, self._safe_read(), 0, 0.0)
        if op == "torque":
            return self._torque(mid, cmd)
        if op == "pose":
            return self._pose(mid, cmd, human)
        if op == "replay":
            return self._replay(mid, cmd, human)
        if op not in ("servo", "nudge", "move"):
            return MotionResult(mid, "refused", f"unknown op {op}", req, {}, None, False, self._safe_read(), 0, 0.0)
        cur = self.backend.read()
        requested, why = self._targets(op, cmd, cur)
        if why:
            return MotionResult(mid, "refused", why, req, {}, None, False, self._round(cur), 0, 0.0)
        gripper_only = set(requested) == {"gripper"}
        speed, why = self._speed(cmd, DEFAULT_SPEED["gripper"] if gripper_only else DEFAULT_SPEED["joint"])
        if why:
            return MotionResult(mid, "refused", why, requested, {}, None, False, self._round(cur), 0, 0.0)
        if not self.backend.torque_on():
            self.backend.hold()
        clamped = guards.clamp(requested)
        goal = {**cur, **clamped}
        zmin, ignored = None, bool(cmd.get("ignore_table"))
        if not gripper_only:
            zmin = round(guards.table_min_z(cur, goal), 4)
            if zmin < TABLE_Z and not ignored:
                return MotionResult(mid, "refused", f"path within {zmin*100:.1f} cm of table", requested, clamped, zmin, False, self._round(cur), 0, 0.0)
        outcome, reason, peak, final = self._watched_move(clamped if gripper_only else goal, speed, LOAD_MAX, LAG_MAX, human, guard=not gripper_only)
        return MotionResult(mid, outcome, reason, requested, clamped, zmin, ignored and zmin is not None, final, peak, 0.0)

    def _targets(self, op, cmd, cur):
        if op == "move":
            joints = cmd.get("joints") or {}
            if not isinstance(joints, dict) or not joints:
                return {}, "value out of range"
            out = {}
            for name, v in joints.items():
                s = guards.resolve_servo(name)
                if s is None:
                    return {}, f"unknown servo {name}"
                val, why = guards.parse_target(s, {"pct" if s == "gripper" else "deg": v})
                if why:
                    return {}, why
                out[s] = val
            return out, ""
        s = guards.resolve_servo(cmd.get("servo", ""))
        if s is None:
            return {}, f"unknown servo {cmd.get('servo')}"
        if op == "nudge":
            try:
                d = float(cmd["delta"])
            except (KeyError, TypeError, ValueError):
                return {}, "value out of range"
            d = max(-NUDGE_MAX, min(NUDGE_MAX, d))
            return {s: cur[s] + d}, ""
        body = cmd.get("body")
        if body is None:
            body = {("pct" if s == "gripper" else "deg"): cmd.get("value")}
        val, why = guards.parse_target(s, body)
        return ({s: val}, "") if not why else ({}, why)

    @staticmethod
    def _speed(cmd, default):
        v = cmd.get("speed")
        if v is None:
            return float(default), ""
        try:
            v = float(v)
        except (TypeError, ValueError):
            return 0.0, "value out of range"
        if not (v > 0):
            return 0.0, "value out of range"
        return min(v, SPEED_MAX), ""

    @staticmethod
    def _round(q):
        return {k: round(float(v), 1) for k, v in q.items()}

    def _check_interrupt(self, human):
        if self._exit.is_set():
            raise _Stop("daemon stopping")
        if self.control.stop_state()["set"]:
            raise _Stop("stop is set")
        if self.control.holder == "released":
            raise _Stop("bus released")
        if self.control.holder == "leader":
            raise _Stop("leader has control")
        if not human and self.control.holder == "human":
            raise _Stop("control taken by human")

    def _watched_move(self, goal, speed, load_max, lag_max, human, guard=True):
        """grasp.move_watch's eased move with the state refreshed every tick and stop/takeover
        checks every 4th tick. Returns (outcome, reason, peak_load, final)."""
        start = self.backend.read()
        goal = guards.clamp(dict(goal))
        joints = [j for j in JOINTS if j in goal]          # only these get new goals; the rest keep theirs
        dist = max(abs(goal[j] - start[j]) for j in joints)
        n = max(int(max(dist / speed, 0.3) * MOTION_HZ), 1)
        peak = 0
        try:
            for i in range(1, n + 1):
                s = i / n
                s = s * s * (3 - 2 * s)
                cmd = {j: start[j] + (goal[j] - start[j]) * s for j in joints}
                self.backend.write_goal(cmd)
                time.sleep(1.0 / MOTION_HZ)
                meas = self.backend.read()
                load = None
                if i % 4 == 0 or i == n:
                    self._check_interrupt(human)
                    load = self.backend.read_load()
                    hot = {j: abs(v) for j, v in load.items() if j != "gripper" and abs(v) > load_max}
                    peak = max([peak] + [abs(v) for j, v in load.items() if j != "gripper"])
                    if guard:
                        chain = [j for j in kin.CHAIN if j in cmd]
                        lagj = max(chain, key=lambda j: abs(cmd[j] - meas[j]))
                        lag = abs(cmd[lagj] - meas[lagj])
                        if hot:
                            j = max(hot, key=hot.get)
                            raise _Stop(f"load {hot[j]} on {j}")
                        if lag > lag_max:
                            raise _Stop(f"lag {lag:.1f} deg on {lagj}")
                self._update(meas, load, True, goal=cmd)
                self._record(meas, cmd)
        except _Stop as e:
            self.backend.freeze()
            final = self.backend.read()
            self._update(final, None, True, goal=final)
            return "frozen", e.reason, peak, self._round(final)
        final = self.backend.settle({**start, **goal}) if guard else self.backend.read()
        self._update(final, None, True, goal=goal)
        return "ok", "", peak, self._round(final)

    def _pose(self, mid, cmd, human):
        name = str(cmd.get("name", ""))
        try:
            name_ = arm_go.resolve(name, self.poses)
        except SystemExit as e:
            return MotionResult(mid, "refused", f"unknown pose {name}: {e}", cmd, {}, None, False, self._safe_read(), 0, 0.0)
        cur = self.backend.read()
        plan = arm_go.route(cur, name_, self.poses)
        zmin = round(min(z for _, _, z in plan), 4)
        ignored = bool(cmd.get("ignore_table"))
        if zmin < TABLE_Z and not ignored:
            leg = next(s for s, _, z in plan if z < TABLE_Z)
            return MotionResult(mid, "refused", f"path within {zmin*100:.1f} cm of table on the way to {leg}", {"pose": name_}, {}, zmin, False, self._round(cur), 0, 0.0)
        if not self.backend.torque_on():
            self.backend.hold()
        peak_all, final = 0, self._round(cur)
        for s, q, _z in plan:
            hub = s in HUBS
            outcome, reason, peak, final = self._watched_move(q, DEFAULT_SPEED["pose"], HUB_LOAD_MAX if hub else LOAD_MAX,
                                                             HUB_LAG_MAX if hub else LAG_MAX, human)
            peak_all = max(peak_all, peak)
            if outcome != "ok":
                return MotionResult(mid, outcome, f"stopped on the way to {s}: {reason}", {"pose": name_}, {}, zmin, ignored, final, peak_all, 0.0)
        if not self.poses[name_].get("hold_after", True) and guards.tool_info(final)["lowest_link_z_m"] <= TORQUE_OFF_MAX_Z:
            self.backend.disable_torque()
            self._update(self.backend.read(), None, False)
        return MotionResult(mid, "ok", "", {"pose": name_}, {}, zmin, ignored, final, peak_all, 0.0)

    def _torque(self, mid, cmd):
        on = bool(cmd.get("on", True))
        cur = self.backend.read()
        if on:
            self.backend.hold()
            self._update(cur, None, True)
            return MotionResult(mid, "ok", "", {"torque": True}, {}, None, False, self._round(cur), 0, 0.0)
        if guards.tool_info(cur)["lowest_link_z_m"] > TORQUE_OFF_MAX_Z and not cmd.get("confirm"):
            return MotionResult(mid, "refused", "arm is raised; pass confirm", {"torque": False}, {}, None, False, self._round(cur), 0, 0.0)
        self.backend.disable_torque()
        self._update(cur, None, False)
        return MotionResult(mid, "ok", "", {"torque": False}, {}, None, False, self._round(cur), 0, 0.0)

    # --- bookkeeping ----------------------------------------------------------------------------
    def _finish(self, r: MotionResult, cmd: dict) -> MotionResult:
        if self.log_dir:
            try:
                self.log_dir.mkdir(parents=True, exist_ok=True)
                with open(self.log_dir / (time.strftime("%Y%m%d") + ".jsonl"), "a") as f:
                    f.write(json.dumps({"t": time.time(), "cmd": cmd, "result": r.to_dict()}) + "\n")
            except Exception:
                pass
        for cb in self._callbacks:
            try:
                cb(r)
            except Exception:
                pass
        return r
