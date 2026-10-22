"""Demo skill cards: break a recorded demonstration into phases with key numbers, keyframes and
tags (summary.json + README.md next to the demo), and a keyword search over them so an agent can
find a similar past run with one call while it works."""
import json
import math
import re
from pathlib import Path

GRASP_Z_MAX = 0.12       # m: a jaw close that settles higher is a posture/survey move, not a grasp
OPEN_PCT = 15.0         # gripper above this = open
Z_NEAR = 0.02            # m above the minimum = still "descending" / "lifting"
STOP_WORDS = {"the", "a", "an", "up", "to", "of", "on", "and", "it", "with", "in"}
SYNONYMS = {"pick": ["grab", "grasp", "lift", "take"], "box": ["carton", "package"], "put": ["place", "drop", "release"],
            "cup": ["mug", "glass"], "bottle": ["flask"]}


def _rows(d: Path):
    f = d / "frames.jsonl"
    if not f.exists():
        return []
    return [json.loads(l) for l in f.read_text().splitlines() if l.strip()]


def _z(r):
    return float(r["tool"]["xyz_m"][2])


def _g(r):
    return float(r["state"]["gripper"])


def _settle(g, i, n):
    """Row where the jaws bottom out after the close that crossed OPEN_PCT at row i (within ~2 s)."""
    return g.index(min(g[i:min(n - 1, i + 60) + 1]), i)


def _keyframe(rows, i):
    for k in range(i, -1, -1):
        if "image" in rows[k]:
            return rows[k]["image"]
    for k in range(i, len(rows)):
        if "image" in rows[k]:
            return rows[k]["image"]
    return None


def _phase(rows, name, a, b):
    a, b = max(0, a), min(len(rows) - 1, max(a, b))
    ra, rb = rows[a], rows[b]
    return {"name": name, "rows": [a, b], "t0_s": round(ra["t"] - rows[0]["t"], 2), "t1_s": round(rb["t"] - rows[0]["t"], 2),
            "start_state": ra["state"], "end_state": rb["state"],
            "start_tool_cm": [round(v * 100, 1) for v in ra["tool"]["xyz_m"]],
            "end_tool_cm": [round(v * 100, 1) for v in rb["tool"]["xyz_m"]],
            "keyframe": _keyframe(rows, (a + b) // 2)}


def segment(rows):
    """Phase boundaries from gripper and tool height. Returns (phases, outcome)."""
    n = len(rows)
    if n < 2:
        return [], "empty"
    g = [_g(r) for r in rows]
    z = [_z(r) for r in rows]
    opens = [i for i in range(n) if g[i] > OPEN_PCT]
    if not opens:
        return [_phase(rows, "motion", 0, n - 1)], "gripper_never_opened"
    e_open = opens[0]
    edges = [i for i in range(e_open + 1, n) if g[i - 1] > OPEN_PCT and g[i] <= OPEN_PCT]
    if not edges and e_open > 0 and 2.5 <= g[0] <= OPEN_PCT:
        # began with the jaws shut on something (a held object reads >= 2.5 %) and ended open: a place
        r_end = next((i for i in range(e_open, n) if i + 5 >= n or max(abs(g[i + k] - g[i]) for k in range(1, 6)) < 0.5), n - 1)
        phases = [_phase(rows, "lower", 0, e_open - 1), _phase(rows, "release", e_open, r_end)]
        if r_end < n - 1:
            phases.append(_phase(rows, "retreat", r_end + 1, n - 1))
        return phases, "placed"
    if not edges:
        phases = [_phase(rows, "approach", 0, e_open - 1)] if e_open > 0 else []
        phases.append(_phase(rows, "open_motion", e_open, n - 1))
        return phases, "never_closed"
    # a posture or survey chain also drives the jaws shut, high above the table: only a close that
    # settles low over the table is a grasp attempt
    grasp_edges = [i for i in edges if z[_settle(g, i, n)] <= GRASP_Z_MAX]
    if not grasp_edges:
        return [_phase(rows, "survey", 0, n - 1)], "no_close"
    e_close = grasp_edges[0]
    e_open = e_close - 1                       # the open run this close ends, not the first open of the run
    while e_open > 0 and g[e_open - 1] > OPEN_PCT:
        e_open -= 1
    phases = [_phase(rows, "approach", 0, e_open - 1)] if e_open > 0 else []
    # descend: from the last row (before close) still within Z_NEAR of the pre-close high point
    zmin_i = min(range(e_open, e_close + 1), key=lambda i: z[i])
    zhigh = max(z[e_open:zmin_i + 1]) if zmin_i > e_open else z[e_open]
    desc_start = next((i for i in range(e_open, zmin_i + 1) if z[i] < zhigh - 0.01), zmin_i)
    if desc_start > e_open:
        phases.append(_phase(rows, "align", e_open, desc_start - 1))
    if zmin_i > desc_start:
        phases.append(_phase(rows, "descend", desc_start, zmin_i))
    # grasp: from close start until the gripper settles (changes < 0.5 over 5 rows) or 1.5 s
    g_end = e_close
    for i in range(e_close, min(n - 1, e_close + 45)):
        if i + 5 < n and max(abs(g[i + k] - g[i]) for k in range(1, 6)) < 0.5:
            g_end = i
            break
        g_end = i
    phases.append(_phase(rows, "grasp", max(zmin_i, desc_start), g_end))
    held = g[g_end] >= 2.5          # a fully shut gripper (~1 %) closed on nothing
    reopen = next((i for i in range(g_end + 1, n) if g[i] > OPEN_PCT), None)
    end_held = reopen - 1 if reopen else n - 1
    if end_held > g_end:
        zmax_i = max(range(g_end, end_held + 1), key=lambda i: z[i])
        lift_end = next((i for i in range(g_end, zmax_i + 1) if z[i] >= z[zmax_i] - 0.003), zmax_i)
        if lift_end > g_end + 1:
            phases.append(_phase(rows, "lift", g_end + 1, lift_end))
        if end_held > lift_end + 1:
            phases.append(_phase(rows, "carry", lift_end + 1, end_held))
    if reopen is not None:
        r_end = next((i for i in range(reopen, n) if i + 5 >= n or max(abs(g[i + k] - g[i]) for k in range(1, 6)) < 0.5), n - 1)
        phases.append(_phase(rows, "release", reopen, r_end))
        if r_end < n - 1:
            phases.append(_phase(rows, "retreat", r_end + 1, n - 1))
        outcome = "grasped_and_released" if held else "closed_empty"
    else:
        outcome = "grasped_and_held" if held else "closed_empty"
    return phases, outcome


def attempts(rows):
    """Every jaw close (open -> closed), with where it happened and what followed: the settle value,
    whether the tool lifted >= 2 cm while the jaws stayed closed within 6 s (held), and the keyframe."""
    out = []
    n = len(rows)
    g = [_g(r) for r in rows]
    z = [_z(r) for r in rows]
    for i in range(1, n):
        if g[i - 1] > OPEN_PCT and g[i] <= OPEN_PCT:
            j = min(n - 1, i + 60)
            settled = min(g[i:j + 1])
            k = g.index(settled, i)
            w = min(n - 1, k + 180)                        # ~6 s at 30 Hz
            lifted, held = 0.0, False
            for q in range(k, w + 1):
                if g[q] > OPEN_PCT:
                    break
                lifted = max(lifted, (z[q] - z[k]) * 100)
            low = z[k] <= GRASP_Z_MAX                      # a close high above the table is a posture, never a hold
            held = low and lifted >= 2.0 and settled >= 1.2       # fully shut reads 0.9-1.1 on this gripper; real holds read 1.5+
            ia = max(0, i - 150)
            lat = sum(math.hypot(rows[q]["tool"]["xyz_m"][0] - rows[q - 1]["tool"]["xyz_m"][0],
                                 rows[q]["tool"]["xyz_m"][1] - rows[q - 1]["tool"]["xyz_m"][1]) for q in range(ia + 1, i + 1)) * 100
            ib = max(0, i - 90)
            dt = rows[i]["t"] - rows[ib]["t"]
            out.append({"t_s": round(rows[i]["t"] - rows[0]["t"], 1), "row": i, "z_cm": round(z[i] * 100, 1),
                        "xy_cm": [round(rows[i]["tool"]["xyz_m"][0] * 100, 1), round(rows[i]["tool"]["xyz_m"][1] * 100, 1)],
                        "open_before_pct": round(g[i - 1], 1), "settled_pct": round(settled, 1), "lifted_cm": round(lifted, 1), "low": low, "held": held,
                        "descent_cm_per_s": round((z[ib] - z[i]) * 100 / dt, 2) if dt else None, "lateral_cm_last_5s": round(lat, 1),
                        "keyframe": _keyframe(rows, k)})
    return out


def _tags(task, holders, phases, outcome, reported=None):
    words = [w for w in re.findall(r"[a-z0-9]+", task.lower()) if w not in STOP_WORDS]
    tags = set(words) | set(holders) | {p["name"] for p in phases}
    for w in words:
        tags.update(SYNONYMS.get(w, []))
    if reported:
        tags.add("success" if str(reported).lower().startswith("succ") else "failed")
    else:
        tags.add("success" if outcome.startswith(("grasped", "placed")) else "failed")
    tags.add(outcome)
    return sorted(tags)


def summarize(d: Path) -> dict:
    d = Path(d)
    meta = json.loads((d / "meta.json").read_text())
    rows = _rows(d)
    phases, outcome = segment(rows)
    atts = attempts(rows)
    if atts:                                              # lift evidence beats the jaw percent
        if any(a["held"] for a in atts):
            later_open = any(_g(r) > OPEN_PCT for r in rows[[a for a in atts if a["held"]][-1]["row"]:])
            outcome = "grasped_and_released" if later_open else "grasped_and_held"
        elif outcome.startswith("grasped"):
            outcome = "closed_empty"
    by = {p["name"]: p for p in phases}
    key = {}
    if "grasp" in by:
        g = by["grasp"]
        key["grasp_tool_z_cm"] = g["start_tool_cm"][2]
        key["grasp_xy_cm"] = g["start_tool_cm"][:2]
        key["grasp_state"] = g["start_state"]
        key["gripper_closed_pct"] = g["end_state"]["gripper"]
    if "align" in by or "descend" in by:
        p = by.get("align") or by["descend"]
        key["gripper_open_pct"] = p["start_state"]["gripper"]
        key["approach_tool_z_cm"] = p["start_tool_cm"][2]
    if "lift" in by:
        key["lift_height_cm"] = by["lift"]["end_tool_cm"][2]
    if "carry" in by:
        c = by["carry"]
        key["carry_dx_cm"] = round(c["end_tool_cm"][0] - c["start_tool_cm"][0], 1)
        key["carry_dy_cm"] = round(c["end_tool_cm"][1] - c["start_tool_cm"][1], 1)
    if "release" in by:
        key["release_tool_z_cm"] = by["release"]["start_tool_cm"][2]
        key["release_xy_cm"] = by["release"]["start_tool_cm"][:2]
    holders = sorted({r["holder"] for r in rows})
    s = {"id": meta["id"], "task": meta["task"], "outcome": outcome, "reported_outcome": meta.get("reported_outcome"),
         "notes": meta.get("notes"), "holders": holders, "attempts": atts, "closes": len(atts),
         "duration_s": round(rows[-1]["t"] - rows[0]["t"], 1) if rows else 0.0,
         "rows": len(rows), "images": sum(1 for r in rows if "image" in r),
         "tags": _tags(meta["task"], holders, phases, outcome, meta.get("reported_outcome")), "key": key, "phases": phases}
    (d / "summary.json").write_text(json.dumps(s, indent=1))
    (d / "README.md").write_text(_readme(s))
    return s


def _readme(s):
    k = s["key"]
    out = [f"# {s['task']}", "", f"demo `{s['id']}` · {s['duration_s']} s · {s['rows']} rows · {s['images']} frames · "
           f"driven by {', '.join(s['holders'])} · outcome **{s['outcome']}**", "", f"tags: {', '.join(s['tags'])}", ""]
    if k:
        out += ["## Key numbers", ""]
        if "approach_tool_z_cm" in k:
            out.append(f"- approach height: {k['approach_tool_z_cm']} cm, gripper open {k['gripper_open_pct']} %")
        if "grasp_tool_z_cm" in k:
            out.append(f"- grasp at tool z {k['grasp_tool_z_cm']} cm, xy {k['grasp_xy_cm']} cm, gripper closed to {k['gripper_closed_pct']} %")
            out.append(f"- grasp joints: {k['grasp_state']}")
        if "lift_height_cm" in k:
            out.append(f"- lifted to {k['lift_height_cm']} cm")
        if "carry_dx_cm" in k:
            out.append(f"- carried dx {k['carry_dx_cm']} cm, dy {k['carry_dy_cm']} cm")
        if "release_tool_z_cm" in k:
            out.append(f"- released at tool z {k['release_tool_z_cm']} cm, xy {k['release_xy_cm']} cm")
        out.append("")
    if s.get("attempts"):
        out += ["## Jaw closes", "", "| # | t (s) | tool z / xy (cm) | open → settled % | lifted (cm) | held | descent cm/s | lateral cm in last 5 s | keyframe |", "|---|---|---|---|---|---|---|---|---|"]
        for i, a in enumerate(s["attempts"], 1):
            out.append(f"| {i} | {a['t_s']} | {a['z_cm']} / {a['xy_cm']} | {a['open_before_pct']} → {a['settled_pct']} | {a['lifted_cm']} | {'yes' if a['held'] else 'no'} | {a['descent_cm_per_s']} | {a['lateral_cm_last_5s']} | {a['keyframe']} |")
        out.append("")
    out += ["## Phases", "", "| phase | t (s) | rows | tool start → end (cm) | gripper | keyframe |", "|---|---|---|---|---|---|"]
    for p in s["phases"]:
        out.append(f"| {p['name']} | {p['t0_s']}–{p['t1_s']} | {p['rows'][0]}–{p['rows'][1]} | {p['start_tool_cm']} → {p['end_tool_cm']} | "
                   f"{p['start_state']['gripper']} → {p['end_state']['gripper']} % | {p['keyframe']} |")
    out += [""] + [f"![{p['name']}]({p['keyframe']})" for p in s["phases"] if p.get("keyframe")] + [""]
    return "\n".join(out)


def load_summary(d: Path, refresh=False) -> dict | None:
    d = Path(d)
    f = d / "summary.json"
    if f.exists() and not refresh:
        try:
            return json.loads(f.read_text())
        except Exception:
            pass
    if (d / "meta.json").exists():
        try:
            return summarize(d)
        except Exception:
            return None
    return None


def search(root: Path, query: str, limit: int = 5) -> list:
    """Keyword match over task, tags, phase names and outcome. Each hit carries the summary minus
    per-phase states (keyframes and key numbers stay) so the agent can act on it directly."""
    root = Path(root)
    words = [w for w in re.findall(r"[a-z0-9]+", (query or "").lower()) if w not in STOP_WORDS]
    if not root.exists():
        return []
    hits = []
    for d in sorted(root.iterdir()):
        s = load_summary(d)
        if not s:
            continue
        task_words = set(re.findall(r"[a-z0-9]+", s["task"].lower()))
        tags = set(s["tags"])
        score = 0
        for w in words:
            if w in task_words:
                score += 3
            elif w in tags:
                score += 2
            elif any(w in t for t in tags) or any(w in tw for tw in task_words):
                score += 1
        if score == 0 and words:
            continue
        lite = {k: v for k, v in s.items() if k != "phases"}
        lite["score"] = score
        lite["phases"] = [{k: v for k, v in p.items() if k not in ("start_state", "end_state")} for p in s["phases"]]
        hits.append(lite)
    hits.sort(key=lambda h: (-h["score"], h["id"]))
    return hits[:limit]


if __name__ == "__main__":
    import sys
    for arg in sys.argv[1:]:
        s = summarize(Path(arg))
        print(s["id"], s["outcome"], [p["name"] for p in s["phases"]], s["key"].get("grasp_tool_z_cm"))
