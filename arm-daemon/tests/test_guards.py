from guards import (JOINTS, LIMITS, TABLE_Z, MotionResult, clamp, parse_target, resolve_servo,
                    table_min_z, tool_info)


def test_resolve_aliases():
    assert resolve_servo("pan") == "shoulder_pan"
    assert resolve_servo("6") == "gripper"
    assert resolve_servo("Wrist_Flex") == "wrist_flex"
    assert resolve_servo("foot") is None


def test_gripper_rejects_deg():
    assert parse_target("gripper", {"deg": 10}) == (None, "wrong unit: gripper takes pct")


def test_joint_rejects_pct():
    assert parse_target("elbow_flex", {"pct": 10}) == (None, "wrong unit: elbow_flex takes deg")


def test_non_numeric():
    assert parse_target("elbow_flex", {"deg": "ten"}) == (None, "value out of range")
    assert parse_target("elbow_flex", {}) == (None, "value out of range")


def test_parse_ok():
    assert parse_target("elbow_flex", {"deg": 12.5}) == (12.5, "")
    assert parse_target("gripper", {"pct": 40}) == (40.0, "")


def test_clamp_reports_limits():
    q = clamp({"shoulder_pan": 500, "gripper": 120})
    assert q["shoulder_pan"] == LIMITS["shoulder_pan"][1]
    assert q["gripper"] == 95.0


def test_limits_margin():
    assert abs(LIMITS["shoulder_pan"][1] - 118.8) < 0.2
    assert LIMITS["gripper"] == (0.0, 95.0)
    assert len(JOINTS) == 6


def test_table_min_z_rest_is_above_threshold():
    rest = {"shoulder_pan": -93.5, "shoulder_lift": -104.1, "elbow_flex": -95.6, "wrist_flex": 55.2, "wrist_roll": -73.8}
    ready = {j: 0.0 for j in rest}
    assert table_min_z(ready, rest) > TABLE_Z


def test_table_min_z_detects_dive():
    ready = {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 0, "wrist_flex": 0, "wrist_roll": 0}
    low = {**ready, "shoulder_lift": 60, "wrist_flex": 90}
    assert table_min_z(ready, low) < TABLE_Z


def test_tool_info_look_down():
    ld = {"shoulder_pan": 0.4, "shoulder_lift": -4.4, "elbow_flex": 7.5, "wrist_flex": 88.6, "wrist_roll": 0.0}
    t = tool_info(ld)
    assert t["tilt_from_down_deg"] < 12
    assert 0.07 < t["xyz_m"][2] < 0.09
    assert t["lowest_link_z_m"] <= t["xyz_m"][2] + 0.001


def test_result_roundtrip():
    r = MotionResult(1, "refused", "busy", {}, {}, None, False, {}, 0, 0.0)
    d = r.to_dict()
    assert d["outcome"] == "refused" and d["id"] == 1 and d["table_min_z_m"] is None
