import json
import time
from pathlib import Path

from demolib import search, summarize

J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def make_demo(root: Path, did="20261023-034820-pick-up-box", task="pick up box", holder="leader"):
    """Synthetic pick: approach high, open, descend to 1.5 cm, close, lift to 20 cm, carry +10 cm x, open, retreat."""
    d = root / did
    (d / "gripper").mkdir(parents=True)
    rows, t, img = [], 1000.0, 0
    def row(z, g, x=0.20, y=0.0, with_img=False):
        nonlocal t, img
        st = {"shoulder_pan": 0.0, "shoulder_lift": -10.0, "elbow_flex": 20.0, "wrist_flex": 80.0, "wrist_roll": 0.0, "gripper": g}
        r = {"t": round(t, 3), "holder": holder, "state": st, "action": dict(st),
             "tool": {"xyz_m": [x, y, z], "tilt_from_down_deg": 3.0, "lowest_link_z_m": z}}
        if with_img:
            name = f"gripper/{img:06d}.jpg"; (d / name).write_bytes(b"\xff\xd8x"); r["image"] = name; img += 1
        rows.append(r); t += 1 / 30
    for i in range(60): row(0.10, 1.0, with_img=(i % 10 == 0))            # approach, closed
    for i in range(60): row(0.10, 50.0, with_img=(i % 10 == 0))           # open, align
    for i in range(60): row(0.10 - 0.085 * i / 59, 50.0, with_img=(i % 10 == 0))   # descend
    for i in range(30): row(0.015, 50.0 - 45.0 * i / 29, with_img=(i % 10 == 0))   # close
    for i in range(60): row(0.015 + 0.185 * i / 59, 5.0, with_img=(i % 10 == 0))   # lift
    for i in range(60): row(0.20, 5.0, x=0.20 + 0.10 * i / 59, with_img=(i % 10 == 0))  # carry
    for i in range(30): row(0.20, 5.0 + 45.0 * i / 29, x=0.30, with_img=(i % 10 == 0))  # release
    for i in range(30): row(0.25, 50.0, x=0.30, with_img=(i % 10 == 0))   # retreat
    (d / "frames.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    (d / "meta.json").write_text(json.dumps({"id": did, "task": task, "started": 1000.0, "ended": t, "rows": len(rows), "images": img, "fps_teleop": 30}))
    return d


def test_summarize_finds_phases_and_numbers(tmp_path):
    d = make_demo(tmp_path)
    s = summarize(d)
    names = [p["name"] for p in s["phases"]]
    assert names == ["approach", "align", "descend", "grasp", "lift", "carry", "release", "retreat"]
    assert abs(s["key"]["grasp_tool_z_cm"] - 1.5) < 0.3
    assert abs(s["key"]["grasp_xy_cm"][0] - 20.0) < 0.5
    assert s["key"]["gripper_open_pct"] >= 45 and s["key"]["gripper_closed_pct"] <= 6
    assert abs(s["key"]["lift_height_cm"] - 20.0) < 0.5
    assert abs(s["key"]["release_tool_z_cm"] - 20.0) < 0.5
    assert s["key"]["carry_dx_cm"] > 9
    for p in s["phases"]:
        assert p["rows"][0] <= p["rows"][1] and "start_state" in p and "end_state" in p and p["keyframe"].startswith("gripper/")
    assert s["outcome"] == "grasped_and_released"
    assert {"pick", "box", "leader", "grasp", "lift"} <= set(s["tags"])
    assert (d / "summary.json").exists() and (d / "README.md").exists()
    md = (d / "README.md").read_text()
    assert "grasp" in md and "1.5 cm" in md


def test_summarize_failed_pick(tmp_path):
    did = "20261023-035114-pick-up-the-carton"
    d = make_demo(tmp_path, did=did, task="pick up the carton", holder="agent")
    rows = [json.loads(l) for l in (d / "frames.jsonl").read_text().splitlines()]
    for r in rows[180:]:          # after the close the gripper goes fully shut: nothing held, no release
        r["state"]["gripper"] = 1.0; r["action"]["gripper"] = 0.0
    (d / "frames.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    s = summarize(d)
    assert s["outcome"] == "closed_empty" and "failed" in s["tags"]
    assert "release" not in [p["name"] for p in s["phases"]]


def test_search_ranks_by_task_and_tags(tmp_path):
    make_demo(tmp_path); summarize(tmp_path / "20261023-034820-pick-up-box")
    d2 = make_demo(tmp_path, did="20261023-040000-wave-hello", task="wave hello"); summarize(d2)
    r = search(tmp_path, "pick up the box")
    assert r[0]["id"] == "20261023-034820-pick-up-box" and r[0]["score"] > 0
    assert [x["id"] for x in search(tmp_path, "wave")] == ["20261023-040000-wave-hello"]
    assert search(tmp_path, "banana") == []
    assert "key" in r[0] and "phases" in r[0] and "grasp_tool_z_cm" in r[0]["key"]
    assert all("rows" not in p or isinstance(p["rows"], list) for p in r[0]["phases"])


def test_card_lists_close_attempts_and_uses_lift_evidence(tmp_path):
    d = make_demo(tmp_path)
    rows = [json.loads(l) for l in (d / "frames.jsonl").read_text().splitlines()]
    for r in rows[180:]:      # a real hold that reads 1.5 % (thin carton, like 23 Oct 04:17)
        if r["state"]["gripper"] < 20:
            r["state"]["gripper"] = 1.5
    (d / "frames.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    s = summarize(d)
    assert len(s["attempts"]) == 1 and s["attempts"][0]["settled_pct"] == 1.5 and s["attempts"][0]["lifted_cm"] > 15
    assert s["outcome"] == "grasped_and_released"       # it lifted and carried: that is the evidence


def test_card_reported_outcome_wins_tags(tmp_path):
    d = make_demo(tmp_path)
    m = json.loads((d / "meta.json").read_text()); m["reported_outcome"] = "failed"; m["notes"] = "knocked it over"
    (d / "meta.json").write_text(json.dumps(m))
    s = summarize(d)
    assert "failed" in s["tags"] and "success" not in s["tags"] and s["reported_outcome"] == "failed"


def test_survey_that_shuts_the_jaws_high_is_no_close(tmp_path):
    """23 Oct 06:18/06:22: analyze_object's survey poses drive the jaws 30 -> 0 -> 30 % while the tool
    rises to 24 cm. No grasp was commanded; that must not become closed_empty with lift/carry/release."""
    d = make_demo(tmp_path, did="20261023-061851-grab-the-box", task="grab the box", holder="agent")
    rows = [json.loads(l) for l in (d / "frames.jsonl").read_text().splitlines()][:120]
    for i, r in enumerate(rows):
        k = i / 119
        g = 30.0 * abs(1 - 2 * k)                                  # 30 -> 0 -> 30
        r["state"]["gripper"] = r["action"]["gripper"] = g
        r["tool"]["xyz_m"][2] = 0.08 + 0.16 * (1 - abs(1 - 2 * k))  # 8 -> 24 -> 8 cm
    (d / "frames.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    s = summarize(d)
    assert s["outcome"] == "no_close" and "failed" in s["tags"]
    assert [p["name"] for p in s["phases"]] == ["survey"] and "grasp_tool_z_cm" not in s["key"]
    assert not any(a["held"] for a in s["attempts"])


def test_survey_then_real_grasp_labels_the_grasp(tmp_path):
    d = make_demo(tmp_path)
    rows = [json.loads(l) for l in (d / "frames.jsonl").read_text().splitlines()]
    pre = []
    for i in range(90):                                            # survey first: open, shut at 24 cm, reopen
        r = json.loads(json.dumps(rows[60]))
        k = i / 89
        g = 30.0 - 29.0 * abs(1 - 2 * k)                           # 1 -> 30 -> 1, shut again while the tool is at 24 cm
        r["state"]["gripper"] = r["action"]["gripper"] = g
        r["tool"]["xyz_m"][2] = 0.08 + 0.16 * k
        r["t"] = rows[0]["t"] - 3 + i / 30
        pre.append(r)
    (d / "frames.jsonl").write_text("\n".join(json.dumps(r) for r in pre + rows) + "\n")
    s = summarize(d)
    assert s["outcome"] == "grasped_and_released"
    assert abs(s["key"]["grasp_tool_z_cm"] - 1.5) < 0.3
    assert s["phases"][0]["name"] == "approach" and s["phases"][0]["rows"][1] >= 90


import pytest

REAL = Path(__file__).resolve().parents[1] / "demos"


@pytest.mark.parametrize("did,outcome", [("20261023-061851-grab-the-box", "no_close"), ("20261023-062200-grab-the-box", "no_close"),
                                         ("20261023-062828-grab-the-box", "grasped_and_held")])
def test_regenerated_cards_of_23_oct_runs(tmp_path, did, outcome):
    src = REAL / did
    if not (src / "frames.jsonl").exists():
        pytest.skip("recorded demo not on this machine")
    d = tmp_path / did
    d.mkdir()
    for f in ("frames.jsonl", "meta.json"):
        (d / f).write_bytes((src / f).read_bytes())
    s = summarize(d)
    assert s["outcome"] == outcome
    if outcome == "no_close":
        assert not {"grasp", "lift", "carry", "release"} & {p["name"] for p in s["phases"]}


def test_place_only_run_is_placed_not_failed(tmp_path):
    """23 Oct 06:37: began with the carton already held (jaws 4.3 %), lowered, released: a success."""
    d = make_demo(tmp_path, did="20261023-063743-put-the-carton-back", task="put the carton back", holder="agent")
    rows = [json.loads(l) for l in (d / "frames.jsonl").read_text().splitlines()]
    sel = rows[210:]                                   # lift, carry, release, retreat of the synthetic pick
    for r in sel:
        if r["state"]["gripper"] < 20:
            r["state"]["gripper"] = r["action"]["gripper"] = 4.3
    (d / "frames.jsonl").write_text("\n".join(json.dumps(r) for r in sel) + "\n")
    s = summarize(d)
    assert s["outcome"] == "placed" and "success" in s["tags"] and "failed" not in s["tags"]
    assert [p["name"] for p in s["phases"]][:2] == ["lower", "release"] and "release_tool_z_cm" in s["key"]


def test_real_place_demo_regenerates_as_placed(tmp_path):
    did = "20261023-063743-put-the-carton-back-down-where-it-was-an"
    src = REAL / did
    if not (src / "frames.jsonl").exists():
        pytest.skip("recorded demo not on this machine")
    d = tmp_path / did
    d.mkdir()
    for f in ("frames.jsonl", "meta.json"):
        (d / f).write_bytes((src / f).read_bytes())
    s = summarize(d)
    assert s["outcome"] == "placed" and "failed" not in s["tags"]
