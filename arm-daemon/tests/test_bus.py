import threading
from tests.conftest import POSE_LIB
import time

from guards import JOINTS


def _go(worker, cmd, human):
    box = {}
    t = threading.Thread(target=lambda: box.setdefault("r", worker.submit(cmd, human=human)))
    t.start()
    return t, box


def test_snapshot_has_all_servos_and_tool(worker):
    s = worker.snapshot()
    assert set(s["servos"]) == set(JOINTS)
    assert "lowest_link_z_m" in s["tool"]
    assert s["motion"]["state"] == "idle"
    assert s["servos"]["gripper"]["max_pct"] == 95.0 and "min_deg" in s["servos"]["elbow_flex"]
    assert s["bus"] == "ok" and s["control"]["holder"] == "human"


def test_servo_move_ok(worker, control):
    control.set_holder("agent", True)
    r = worker.submit({"op": "servo", "servo": "elbow_flex", "value": 10.0}, human=False)
    assert r.outcome == "ok" and abs(r.final["elbow_flex"] - 10.0) < 0.1 and r.clamped["elbow_flex"] == 10.0
    assert r.id >= 1 and worker.snapshot()["motion"]["result"]["id"] == r.id


def test_agent_refused_when_human_holds(worker):
    r = worker.submit({"op": "servo", "servo": "elbow_flex", "value": 10.0}, human=False)
    assert (r.outcome, r.reason) == ("refused", "human has control")


def test_unknown_servo_and_bad_unit(worker):
    assert worker.submit({"op": "servo", "servo": "foot", "value": 1}, human=True).reason == "unknown servo foot"
    r = worker.submit({"op": "servo", "servo": "gripper", "body": {"deg": 10}}, human=True)
    assert r.outcome == "refused" and r.reason.startswith("wrong unit")


def test_clamp_is_reported_not_refused(worker):
    r = worker.submit({"op": "servo", "servo": "shoulder_pan", "value": 500}, human=True)
    assert r.outcome == "ok" and r.clamped["shoulder_pan"] < 500 and r.requested["shoulder_pan"] == 500


def test_table_refused_then_ignored(worker):
    low = {"op": "move", "joints": {"shoulder_lift": 60, "elbow_flex": 0, "wrist_flex": 90}}
    r = worker.submit(low, human=True)
    assert r.outcome == "refused" and r.reason.startswith("path within")
    r = worker.submit({**low, "ignore_table": True}, human=True)
    assert r.outcome == "ok" and r.table_ignored


def test_lag_freezes(worker, backend):
    backend.block_joint = "elbow_flex"
    r = worker.submit({"op": "servo", "servo": "elbow_flex", "value": 30.0}, human=True)
    assert r.outcome == "frozen" and "lag" in r.reason and "freeze" in backend.calls


def test_load_freezes(worker, backend):
    backend.load["wrist_flex"] = 400
    r = worker.submit({"op": "servo", "servo": "wrist_flex", "value": 30.0}, human=True)
    assert r.outcome == "frozen" and "load" in r.reason and r.peak_load >= 400


def test_gripper_ignores_load_guard(worker, backend):
    backend.load["gripper"] = 400
    r = worker.submit({"op": "servo", "servo": "gripper", "value": 50.0}, human=True)
    assert r.outcome == "ok" and r.final["gripper"] == 50.0


def test_second_write_while_moving_is_busy(worker, backend):
    backend.slow = True
    t, box = _go(worker, {"op": "servo", "servo": "elbow_flex", "value": 30.0}, True)
    time.sleep(0.05)
    r = worker.submit({"op": "servo", "servo": "wrist_flex", "value": 5.0}, human=True)
    assert (r.outcome, r.reason) == ("refused", "busy")
    t.join()
    assert box["r"].outcome == "ok"


def test_stop_file_freezes_mid_move(worker, backend, control, tmp_path):
    backend.slow = True
    t, box = _go(worker, {"op": "servo", "servo": "elbow_flex", "value": 30.0}, True)
    time.sleep(0.05)
    (tmp_path / "STOP").touch()
    t.join()
    assert box["r"].outcome == "frozen" and box["r"].reason == "stop is set"
    assert control.stop_state()["by"] == "file" and "freeze" in backend.calls


def test_human_takeover_freezes_mid_move(worker, backend, control):
    control.set_holder("agent", True)
    backend.slow = True
    t, box = _go(worker, {"op": "servo", "servo": "elbow_flex", "value": 30.0}, False)
    time.sleep(0.05)
    control.set_holder("human", True)
    t.join()
    assert box["r"].outcome == "frozen" and box["r"].reason == "control taken by human"


def test_human_move_survives_holder_agent(worker, backend, control):
    control.set_holder("agent", True)
    r = worker.submit({"op": "servo", "servo": "elbow_flex", "value": 10.0}, human=True)
    assert r.outcome == "ok"


def test_frozen_does_not_latch(worker, backend):
    backend.load["wrist_flex"] = 400
    worker.submit({"op": "servo", "servo": "wrist_flex", "value": 30.0}, human=True)
    backend.load["wrist_flex"] = 50
    assert worker.submit({"op": "servo", "servo": "wrist_flex", "value": 5.0}, human=True).outcome == "ok"
    assert worker.snapshot()["motion"]["state"] == "idle"


def test_nudge_clips_to_15(worker):
    r = worker.submit({"op": "nudge", "servo": "elbow_flex", "delta": 40}, human=True)
    assert abs(r.final["elbow_flex"] - 15) < 0.1
    r = worker.submit({"op": "nudge", "servo": "elbow_flex", "delta": -40}, human=True)
    assert abs(r.final["elbow_flex"] - 0) < 0.1


def test_pose_ready_then_rest_releases_torque(worker, backend):
    assert worker.submit({"op": "pose", "name": "ready"}, human=True).outcome == "ok"
    r = worker.submit({"op": "pose", "name": "rest"}, human=True)
    assert r.outcome == "ok" and "disable_torque" in backend.calls and backend.torque is False


def test_pose_hub_legs_use_hub_limits(worker, backend):
    backend.load["elbow_flex"] = 500   # > 350, < 650
    assert worker.submit({"op": "pose", "name": "straight"}, human=True).outcome == "ok"


def test_pose_alias_and_unknown(worker):
    assert worker.submit({"op": "pose", "name": "standby"}, human=True).outcome == "ok"
    assert worker.submit({"op": "pose", "name": "banana"}, human=True).reason.startswith("unknown pose")


def test_torque_off_guard(worker, backend):
    r = worker.submit({"op": "torque", "on": False}, human=True)
    assert r.outcome == "refused" and r.reason == "arm is raised; pass confirm"
    assert worker.submit({"op": "torque", "on": False, "confirm": True}, human=True).outcome == "ok"
    assert backend.torque is False
    assert worker.submit({"op": "torque", "on": True}, human=True).outcome == "ok" and backend.torque is True


def test_hold_before_write_when_torque_off(worker, backend):
    backend.torque = False
    worker.submit({"op": "servo", "servo": "elbow_flex", "value": 5.0}, human=True)
    assert "hold" in backend.calls


def test_result_callback(worker):
    seen = []
    worker.on_result(seen.append)
    worker.submit({"op": "servo", "servo": "elbow_flex", "value": 5.0}, human=True)
    assert seen and seen[-1].outcome == "ok"


def test_unknown_op(worker):
    assert worker.submit({"op": "fly"}, human=True).reason == "unknown op fly"


def test_released_frees_and_reclaims_bus(worker, backend, control):
    assert backend.connected is True
    control.set_holder("released", True)
    time.sleep(0.1)
    assert backend.connected is False
    assert worker.snapshot()["bus"] == "released"
    r = worker.submit({"op": "servo", "servo": "elbow_flex", "value": 5.0}, human=True)
    assert (r.outcome, r.reason) == ("refused", "bus released")
    control.set_holder("human", True)
    time.sleep(0.1)
    assert backend.connected is True and worker.snapshot()["bus"] == "ok"
    assert worker.submit({"op": "servo", "servo": "elbow_flex", "value": 5.0}, human=True).outcome == "ok"


def test_released_freezes_mid_move(worker, backend, control):
    backend.slow = True
    t, box = _go(worker, {"op": "servo", "servo": "elbow_flex", "value": 30.0}, True)
    time.sleep(0.05)
    control.set_holder("released", True)
    t.join()
    assert box["r"].outcome == "frozen" and box["r"].reason == "bus released"
    time.sleep(0.1)
    assert backend.connected is False


def test_gripper_only_move_does_not_rewrite_arm_goals(worker, backend):
    backend.sag = 1.5
    worker.submit({"op": "servo", "servo": "shoulder_lift", "value": 20.0}, human=True)
    lift_after_move = backend.q["shoulder_lift"]
    for pct in (50, 0, 50, 0, 50, 0):
        worker.submit({"op": "servo", "servo": "gripper", "value": float(pct)}, human=True)
    assert set(backend.goals) == {"gripper"}
    assert abs(backend.q["shoulder_lift"] - lift_after_move) < 0.01


def test_concurrent_submits_only_one_runs(worker, backend):
    backend.slow = True
    boxes = []
    ts = []
    for _ in range(4):
        t, box = _go(worker, {"op": "servo", "servo": "elbow_flex", "value": 30.0}, True)
        ts.append(t); boxes.append(box)
    for t in ts:
        t.join()
    outcomes = sorted(b["r"].outcome for b in boxes)
    assert outcomes == ["ok", "refused", "refused", "refused"]
    assert all(b["r"].reason == "busy" for b in boxes if b["r"].outcome == "refused")


def test_daemon_stop_freezes_mid_move(worker, backend):
    backend.slow = True
    t, box = _go(worker, {"op": "servo", "servo": "elbow_flex", "value": 30.0}, True)
    time.sleep(0.05)
    worker.stop()
    t.join()
    assert box["r"].outcome == "frozen" and box["r"].reason == "daemon stopping"
    assert worker.submit({"op": "servo", "servo": "elbow_flex", "value": 1.0}, human=True).reason == "daemon stopping"


def test_connect_failure_latches(backend, control):
    from bus import BusWorker
    backend.fail_connect = "calibration mismatch"
    w = BusWorker(backend, control, POSE_LIB, hz=200)
    w.start(); time.sleep(0.05)
    try:
        assert w.snapshot()["bus"].startswith("error:calibration mismatch")
        r = w.submit({"op": "servo", "servo": "elbow_flex", "value": 1.0}, human=True)
        assert r.outcome == "refused" and r.reason.startswith("bus error:")
        time.sleep(0.05)
        assert w.snapshot()["bus"].startswith("error:")
    finally:
        w.stop()


def test_speed_validated(worker):
    r = worker.submit({"op": "servo", "servo": "elbow_flex", "value": 5.0, "speed": "fast"}, human=True)
    assert (r.outcome, r.reason) == ("refused", "value out of range")
    r = worker.submit({"op": "servo", "servo": "elbow_flex", "value": 5.0, "speed": -3}, human=True)
    assert (r.outcome, r.reason) == ("refused", "value out of range")
    t0 = time.time()
    r = worker.submit({"op": "servo", "servo": "shoulder_pan", "value": 90.0, "speed": 1e9}, human=True)
    assert r.outcome == "ok" and time.time() - t0 >= 90 / 90 - 0.1   # capped at 90 deg/s


def test_requested_is_normalised_on_refusal(worker):
    r = worker.submit({"op": "servo", "servo": "elbow_flex", "value": 5.0}, human=False)
    assert r.requested == {"elbow_flex": 5.0}


def test_health_log_records_temperature_in_every_mode(backend, control, tmp_path):
    from bus import BusWorker
    backend.temp["elbow_flex"] = 61
    w = BusWorker(backend, control, POSE_LIB, hz=200, log_dir=tmp_path, temp_every_s=0.01, health_every_s=0.05)
    w.start()
    time.sleep(0.4)
    w.stop()
    import json
    rows = [json.loads(l) for f in (tmp_path / "health").glob("*.jsonl") for l in f.read_text().splitlines()]
    assert len(rows) >= 2 and rows[0]["servos"]["elbow_flex"]["temp_c"] == 61
    assert set(rows[0]["servos"]) == set(JOINTS) and {"temp_c", "load", "pos", "torque"} <= set(rows[0]["servos"]["gripper"])
    assert w.snapshot()["servos"]["elbow_flex"]["temp_c"] == 61


def test_temperature_refreshes_during_leader_teleop(backend, control, leader, tmp_path):
    from bus import BusWorker
    w = BusWorker(backend, control, POSE_LIB, hz=200, leader=leader, log_dir=tmp_path, temp_every_s=0.01)
    w.start()
    control.set_holder("leader", True)
    time.sleep(0.2)
    backend.temp["elbow_flex"] = 66
    time.sleep(0.3)
    assert w.snapshot()["leader"]["active"] and w.snapshot()["servos"]["elbow_flex"]["temp_c"] == 66
    w.stop()
