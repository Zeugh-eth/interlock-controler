import json
import threading
import time

from guards import JOINTS


def _wait(cond, t=2.0):
    end = time.time() + t
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_leader_mode_mirrors_leader_with_step_cap(worker_t, backend, leader, control):
    control.set_holder("leader", True)
    leader.q["elbow_flex"] = 10.0
    assert _wait(lambda: abs(backend.q["elbow_flex"] - 10.0) < 0.1)
    assert backend.connected and leader.connected
    assert worker_t.snapshot()["bus"] == "ok" and worker_t.snapshot()["control"]["holder"] == "leader"
    leader.q["elbow_flex"] = 80.0
    time.sleep(0.03)                                   # first tick(s) after the jump
    assert backend.goals["elbow_flex"] <= 10.0 + 15.0 * 3 + 0.01
    assert _wait(lambda: abs(backend.q["elbow_flex"] - 80.0) < 0.1)
    control.set_holder("human", True)
    assert _wait(lambda: not leader.connected)
    assert backend.connected


def test_leader_mode_refuses_writes_and_holds(worker_t, backend, control):
    backend.torque = False
    control.set_holder("leader", True)
    time.sleep(0.05)
    assert "hold" in backend.calls
    r = worker_t.submit({"op": "servo", "servo": "elbow_flex", "value": 5.0}, human=True)
    assert (r.outcome, r.reason) == ("refused", "leader has control")


def test_leader_mode_without_leader_arm_is_refused(worker, control):
    ok, why = control.set_holder("leader", True)
    assert ok
    time.sleep(0.05)
    assert worker.snapshot()["bus"].startswith("error:no leader arm")


def test_demo_records_teleop_and_agent_moves(worker_t, backend, leader, frames, control, tmp_path):
    d = worker_t.demo_start("pick up the carton")
    assert d["id"].endswith("pick-up-the-carton") and (tmp_path / "demos" / d["id"] / "meta.json").exists()
    control.set_holder("leader", True)
    leader.q["elbow_flex"] = 10.0
    assert _wait(lambda: abs(backend.q["elbow_flex"] - 10.0) < 0.1)
    frames.bump()
    control.set_holder("human", True)
    time.sleep(0.05)
    r = worker_t.submit({"op": "servo", "servo": "wrist_flex", "value": 5.0}, human=True)
    assert r.outcome == "ok"
    s = worker_t.demo_stop()
    assert s["id"] == d["id"] and s["rows"] > 5 and s["images"] == 2
    rows = [json.loads(l) for l in (tmp_path / "demos" / d["id"] / "frames.jsonl").read_text().splitlines()]
    assert rows[0]["image"] == "gripper/000000.jpg" and "image" not in rows[-1]
    assert {"t", "state", "action", "tool", "holder"} <= set(rows[0])
    assert any(r_["holder"] == "leader" for r_ in rows) and any(r_["holder"] == "human" for r_ in rows)
    assert rows[-1]["state"]["wrist_flex"] == 5.0
    meta = json.loads((tmp_path / "demos" / d["id"] / "meta.json").read_text())
    assert meta["task"] == "pick up the carton" and meta["rows"] == s["rows"] and "ended" in meta
    assert worker_t.demo_stop() is None


def test_demo_list_and_read(worker_t, control, leader, backend, tmp_path):
    d = worker_t.demo_start("wave")
    control.set_holder("leader", True)
    leader.q["shoulder_pan"] = 5.0
    assert _wait(lambda: abs(backend.q["shoulder_pan"] - 5.0) < 0.1)
    control.set_holder("human", True)
    worker_t.demo_stop()
    lst = worker_t.list_demos()
    assert [x["id"] for x in lst] == [d["id"]] and lst[0]["task"] == "wave" and lst[0]["rows"] > 0
    full = worker_t.demo(d["id"])
    assert full["meta"]["task"] == "wave" and len(full["rows"]) == lst[0]["rows"]
    assert worker_t.demo("nope") is None


def test_replay_follows_demo(worker_t, control, leader, backend):
    d = worker_t.demo_start("bend")
    control.set_holder("leader", True)
    for v in (5.0, 10.0, 15.0, 20.0):
        leader.q["elbow_flex"] = v
        assert _wait(lambda: abs(backend.q["elbow_flex"] - v) < 0.1)
        time.sleep(0.02)
    control.set_holder("human", True)
    worker_t.demo_stop()
    backend.q["elbow_flex"] = 0.0
    r = worker_t.submit({"op": "replay", "id": d["id"], "every": 5}, human=True)
    assert r.outcome == "ok" and abs(r.final["elbow_flex"] - 20.0) < 0.5
    assert r.requested == {"replay": d["id"]}
    assert worker_t.submit({"op": "replay", "id": "nope"}, human=True).reason == "unknown demo nope"


def test_replay_freezes_and_reports_waypoint(worker_t, control, leader, backend):
    d = worker_t.demo_start("bend")
    control.set_holder("leader", True)
    leader.q["elbow_flex"] = 10.0
    assert _wait(lambda: abs(backend.q["elbow_flex"] - 10.0) < 0.1)
    control.set_holder("human", True)
    worker_t.demo_stop()
    backend.q["elbow_flex"] = 0.0
    backend.block_joint = "elbow_flex"
    r = worker_t.submit({"op": "replay", "id": d["id"], "every": 5}, human=True)
    assert r.outcome == "frozen" and r.reason.startswith("stopped at waypoint")


def test_demo_stop_merges_extra_meta(worker_t, control, leader, backend, tmp_path):
    d = worker_t.demo_start("tag me")
    control.set_holder("leader", True)
    leader.q["elbow_flex"] = 3.0
    assert _wait(lambda: abs(backend.q["elbow_flex"] - 3.0) < 0.1)
    control.set_holder("human", True)
    s = worker_t.demo_stop({"reported_outcome": "failed", "notes": "closed on air"})
    meta = json.loads((tmp_path / "demos" / d["id"] / "meta.json").read_text())
    assert meta["reported_outcome"] == "failed" and meta["notes"] == "closed on air" and s["reported_outcome"] == "failed"
