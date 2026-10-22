import math

import pytest

from tests.fake_daemon import StatefulFakeDaemon


@pytest.fixture
def fd():
    d = StatefulFakeDaemon()
    yield d
    d.close()


@pytest.fixture
def sk(fd):
    import skills
    return skills.Skills(fd.url)


def test_observe_is_synchronized_and_flags_stale(sk, fd):
    o = sk.observe()
    assert o["frame"].startswith(b"\xff\xd8") and o["stale"] is False and "joints" in o and "tool_xyz_m" in o
    fd.frame_age = 1.2
    assert sk.observe()["stale"] is True
    fd.cam_ok = False
    o = sk.observe()
    assert o["frame"] is None and o["stale"] is True


def test_move_tool_changes_tool_position(sk, fd):
    z0 = sk.observe()["tool_xyz_m"][2]
    r = sk.move_tool(dz=-0.02)
    assert r["outcome"] == "ok" and abs(r["tool_xyz_m"][2] - (z0 - 0.02)) < 0.004
    r = sk.goto_xyz([0.20, 0.02, 0.06])
    assert r["outcome"] == "ok" and max(abs(a - b) for a, b in zip(r["tool_xyz_m"], [0.20, 0.02, 0.06])) < 0.004


def test_move_tool_unreachable_is_refused(sk):
    r = sk.goto_xyz([0.60, 0.0, 0.05])
    assert r["outcome"] == "refused" and r["reason"].startswith("unreachable")


def test_project_pixel_to_table(sk):
    p = sk.project(320, 240)
    assert abs(p["z_m"] - 0.003) < 1e-6 and 0.1 < p["x_m"] < 0.35 and abs(p["y_m"]) < 0.1
    far = sk.project(320, 100)      # image top = world right (-y)... just must differ along one axis
    assert abs(far["y_m"] - p["y_m"]) > 0.005 or abs(far["x_m"] - p["x_m"]) > 0.005


def test_descend_to_contact_backs_off(sk, fd):
    fd.contact_z = 0.03
    r = sk.descend_to_contact(max_cm=8, step_cm=0.5, back_off_cm=1.0)
    assert r["contact"] is True and 0.03 <= r["z_contact_m"] < 0.036 and abs(r["tool_xyz_m"][2] - (r["z_contact_m"] + 0.01)) < 0.004
    assert r["reason"].startswith("load")


def test_descend_without_contact_stops_at_floor(sk, fd):
    r = sk.descend_to_contact(max_cm=3, step_cm=1.0)
    assert r["contact"] is False and r["steps"] == 3


def test_grasp_reports_held_with_evidence(sk, fd):
    fd.object_width_pct = 8.0
    r = sk.grasp()
    assert r["verdict"] == "held" and r["jaw_pct_after_close"] == 8.0 and r["jaw_pct_after_lift"] == 8.0 and r["frame"]


def test_grasp_reports_empty(sk, fd):
    r = sk.grasp()
    assert r["verdict"] == "empty" and r["jaw_pct_after_close"] <= 3.0


def test_grasp_reports_slipped(sk, fd):
    fd.object_width_pct = 8.0
    fd.slip_after_lift = True
    r = sk.grasp()
    assert r["verdict"] == "slipped"


def test_release_opens_retreats_and_observes(sk, fd):
    z0 = sk.observe()["tool_xyz_m"][2]
    r = sk.release(retreat_cm=3)
    assert r["outcome"] == "ok" and fd.q["gripper"] >= 50 and r["tool_xyz_m"][2] > z0 + 0.02 and r["frame"]


def test_demo_stop_with_outcome(sk, fd):
    r = sk.demo_stop(reported_outcome="failed", notes="closed on air")
    assert fd.demo_stop_body == {"reported_outcome": "failed", "notes": "closed on air"} and r["reported_outcome"] == "failed"


# --- lessons from the 23 Oct runs ----------------------------------------------------------------
def _plan(sk, fd, **obj):
    fd.object = {"label": "carton", "x": 0.20, "y": 0.0, "w_cm": 4.0, "h_cm": 7.0, "conf": 0.5, **obj}
    fd.object_width_pct = 8.0
    return sk.analyze_object("carton")


def _closes(fd):
    return [c for c in fd.calls if c[0] == "POST" and c[1] == "/gripper" and float(c[2].get("pct", 99)) <= 0.0]


def test_grab_never_closes_when_not_aligned(sk, fd):
    """23 Oct 04:57 and 05:31: closed with the carton beside the jaws. A grab that cannot centre
    the object within tolerance must not close at all."""
    a = _plan(sk, fd)
    fd.object["sticky_offset_cm"] = [3.0, 0.0]          # whatever we do, it stays 3 cm forward
    r = sk.grab(a["plan"])
    assert _closes(fd) == [], "closed the jaws without the object between them"
    assert r["outcome"] == "not_aligned" and r["evaluation"]["quality"] == "not_aligned"
    assert "offset" in r["reason"]


def test_grab_realigns_at_grasp_height_before_closing(sk, fd):
    """Parallax: the carton looked centred from 6 cm and sat 2 cm off at grasp height."""
    a = _plan(sk, fd, shift_when_low_cm=3.0)
    r = sk.grab(a["plan"])
    assert r["outcome"] == "ok" and r["low_corrections"] >= 1 and r["evaluation"]["quality"] in ("good", "edge")
    # the last close happened after the last low correction
    idx_close = max(i for i, c in enumerate(fd.calls) if c[0] == "POST" and c[1] == "/gripper" and float(c[2]["pct"]) <= 0.0)
    idx_moves = [i for i, c in enumerate(fd.calls) if c[0] == "POST" and c[1] == "/move"]
    assert any(i < idx_close for i in idx_moves)
    assert r["aligned_at_grasp_height"] is True


def test_grab_measures_gain_from_first_correction(sk, fd):
    """Mismeasurement: the overlay's cm were 5 cm off tonight. The first correction move measures
    how far the object really shifted per commanded cm and scales the next one."""
    a = _plan(sk, fd, gain=2.0)                  # overlay reports twice the true offset
    fd.object["x"] = 0.22                        # 2 cm true, reported 4 cm
    r = sk.grab({**a["plan"], "target_xy_m": [0.20, 0.0]})
    assert r["outcome"] == "ok" and abs(r["measured_gain"] - 2.0) < 0.3 and r["corrections"] <= 3


def test_analyze_requires_full_view(sk, fd):
    """Half-identified object: a bbox touching the frame edge means size and centre are unknown."""
    fd.object = {"label": "carton", "x": 0.20, "y": 0.0, "w_cm": 4.0, "h_cm": 7.0, "conf": 0.5, "clipped": True}
    a = sk.analyze_object("carton")
    assert a["plan"] is None and "partly out of view" in a["reason"]
    assert a["card"]["full_view"] is False
    fd.object["clipped"] = False
    a = sk.analyze_object("carton")
    assert a["plan"] is not None and a["card"]["full_view"] is True


def test_grasp_uses_vision_when_jaws_read_closed(sk, fd):
    """23 Oct 04:17: a real hold read 1.5 %; 03:51: an empty close read 1.0 %. The jaw percent
    cannot tell them apart; whether the object rides with the jaws after the lift can."""
    fd.object = {"label": "carton", "x": 0.20, "y": 0.0, "w_cm": 4.0, "h_cm": 7.0, "held": True}
    fd.object_width_pct = 1.0
    r = sk.grasp(label="carton")
    assert r["verdict"] == "held" and "vision" in r["evidence"]
    fd.object = {"label": "carton", "x": 0.24, "y": 0.0, "w_cm": 4.0, "h_cm": 7.0}
    fd.object_width_pct = 1.0
    r = sk.grasp(label="carton")
    assert r["verdict"] == "empty" and "vision" in r["evidence"]


def test_grab_descends_in_steps_below_approach(sk, fd):
    """Paced approach: the human demos descend the last centimetres slowly with the object in view;
    the agent's misses dropped straight to the planned height."""
    a = _plan(sk, fd)
    r = sk.grab(a["plan"])
    moves = [c for c in fd.calls if c[0] == "POST" and c[1] == "/move"]
    assert r["descent_steps"] >= 2 and len(moves) >= r["descent_steps"]
