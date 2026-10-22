"""analyze -> grab -> evaluate -> replan skills over the stateful fake daemon."""
import pytest

from tests.fake_daemon import StatefulFakeDaemon


@pytest.fixture
def fd():
    d = StatefulFakeDaemon()
    d.object = {"label": "carton", "x": 0.24, "y": -0.06, "w_cm": 7.0, "h_cm": 10.5, "conf": 0.3}
    yield d
    d.close()


@pytest.fixture
def sk(fd):
    import skills
    return skills.Skills(fd.url)


def test_analyze_builds_card_and_plan(sk, fd):
    a = sk.analyze_object("carton")
    card, plan = a["card"], a["plan"]
    assert card["label"] == "carton" and card["table_xy_m"] == [0.24, -0.06]
    assert card["short_cm"] == 7.0 and card["long_cm"] == 10.5 and card["long_axis"] == "x"
    assert plan["roll_deg"] == 90 and plan["across_jaws_cm"] == 7.0      # close across the short side
    assert plan["open_pct"] == 95 and 0.005 <= plan["grasp_z_m"] <= 0.02 and plan["approach_z_m"] == 0.06
    assert a["survey_tool_z_m"] >= 0.10 and a["frame"]
    assert plan["feasible"] is True


def test_analyze_long_axis_y_uses_roll_zero(sk, fd):
    fd.object.update(w_cm=10.5, h_cm=7.0)
    plan = sk.analyze_object("carton")["plan"]
    assert plan["roll_deg"] == 0 and plan["across_jaws_cm"] == 7.0


def test_analyze_too_wide_is_infeasible(sk, fd):
    fd.object.update(w_cm=9.0, h_cm=12.0)
    plan = sk.analyze_object("carton")["plan"]
    assert plan["feasible"] is False and "too wide" in plan["reason"]


def test_analyze_pans_to_find_object(sk, fd):
    fd.object["min_pan"] = 30.0
    a = sk.analyze_object("carton")
    assert a["card"]["label"] == "carton" and a["searched_pans"]


def test_analyze_not_found(sk, fd):
    fd.object = None
    a = sk.analyze_object("cup")
    assert a["card"] is None and "not found" in a["reason"]


def test_grab_aligns_descends_and_holds(sk, fd):
    fd.object_width_pct = 45.0
    plan = sk.analyze_object("carton")["plan"]
    r = sk.grab(plan)
    assert r["evaluation"]["quality"] == "good" and r["grasp"]["verdict"] == "held"
    assert abs(fd.q["wrist_roll"] - 90) < 0.1 and r["corrections"] <= 3
    assert abs(r["aligned_xy_m"][0] - 0.24) < 0.012 and abs(r["aligned_xy_m"][1] + 0.06) < 0.012


def test_evaluate_flags_edge_pinch(sk):
    plan = {"across_jaws_cm": 7.0, "open_pct": 95}
    ev = sk.evaluate_grasp({"verdict": "held", "jaw_pct_after_close": 3.6, "jaw_pct_after_lift": 4.0}, plan)
    assert ev["quality"] == "edge" and ev["expected_jaw_pct"] > 20
    ev = sk.evaluate_grasp({"verdict": "held", "jaw_pct_after_close": 44.0, "jaw_pct_after_lift": 44.5}, plan)
    assert ev["quality"] == "good"
    assert sk.evaluate_grasp({"verdict": "empty", "jaw_pct_after_close": 1.2}, plan)["quality"] == "empty"
    assert sk.evaluate_grasp({"verdict": "slipped", "jaw_pct_after_close": 40, "jaw_pct_after_lift": 2}, plan)["quality"] == "slipped"


def test_replan_rules(sk, fd):
    plan = sk.analyze_object("carton")["plan"]
    p2 = sk.replan(plan, {"quality": "empty"})
    assert p2["grasp_z_m"] < plan["grasp_z_m"] and p2["attempt"] == 2 and p2["resurvey"] is True
    p3 = sk.replan(p2, {"quality": "edge"})
    assert p3["attempt"] == 3 and p3["shift_along_jaws_cm"] != 0
    p4 = sk.replan(p3, {"quality": "edge"})
    assert p4["feasible"] is False and "ask" in p4["reason"]
