import asyncio

H = "X-Arm-Human-Key"


async def test_state_shape(client):
    r = await client.get("/state")
    s = await r.json()
    assert r.status == 200
    assert {"t", "bus", "control", "stop", "servos", "tool", "motion", "cameras"} <= set(s)
    assert s["cameras"]["gripper"]["ok"] is True


async def test_health(client):
    s = await (await client.get("/health")).json()
    assert s["ok"] is True and s["bus"] == "ok" and "uptime_s" in s


async def test_agent_write_refused_409(client):
    r = await client.post("/servo/elbow", json={"deg": 5})
    assert r.status == 409 and (await r.json())["reason"] == "human has control"


async def test_human_write_ok(client, key):
    r = await client.post("/servo/elbow", json={"deg": 5}, headers={H: key})
    assert r.status == 200
    j = await r.json()
    assert j["outcome"] == "ok" and abs(j["final"]["elbow_flex"] - 5) < 0.1


async def test_wrong_key_is_agent(client):
    r = await client.post("/servo/elbow", json={"deg": 5}, headers={H: "nope"})
    assert r.status == 409 and (await r.json())["reason"] == "human has control"


async def test_control_switch(client, key):
    assert (await client.post("/control", json={"holder": "agent"})).status == 403
    r = await client.post("/control", json={"holder": "agent"}, headers={H: key})
    assert r.status == 200 and (await r.json())["holder"] == "agent"
    assert (await client.post("/servo/elbow", json={"deg": 5})).status == 200
    assert (await client.post("/control", json={"holder": "robot"}, headers={H: key})).status == 400


async def test_stop_and_resume(client, key):
    r = await client.post("/stop", json={"reason": "t"})
    assert r.status == 200 and (await r.json())["stop"]["set"] is True
    assert (await client.post("/servo/elbow", json={"deg": 5}, headers={H: key})).status == 409
    assert (await client.post("/resume")).status == 403
    assert (await client.post("/resume", headers={H: key})).status == 200
    assert (await client.post("/servo/elbow", json={"deg": 5}, headers={H: key})).status == 200


async def test_unknown_servo_404(client, key):
    assert (await client.get("/servo/foot")).status == 404
    r = await client.post("/servo/foot", json={"deg": 1}, headers={H: key})
    assert r.status == 404 and (await r.json())["refused"] == "unknown servo foot"


async def test_servo_get(client):
    j = await (await client.get("/servo/pan")).json()
    assert j["name"] == "shoulder_pan" and "limits" in j and "deg" in j


async def test_gripper_alias_and_unit(client, key):
    r = await client.post("/gripper", json={"pct": 40}, headers={H: key})
    assert (await r.json())["final"]["gripper"] == 40
    r = await client.post("/servo/gripper", json={"deg": 40}, headers={H: key})
    assert r.status == 409 and (await r.json())["reason"].startswith("wrong unit")


async def test_nudge_move_pose_torque(client, key):
    assert (await (await client.post("/servo/elbow/nudge", json={"delta": 3}, headers={H: key})).json())["outcome"] == "ok"
    assert (await (await client.post("/move", json={"joints": {"elbow": 2, "wrist": 2}}, headers={H: key})).json())["outcome"] == "ok"
    assert (await (await client.post("/pose", json={"name": "ready"}, headers={H: key})).json())["outcome"] == "ok"
    r = await client.post("/torque", json={"on": False}, headers={H: key})
    assert r.status == 409 and (await r.json())["reason"] == "arm is raised; pass confirm"


async def test_bad_json(client, key):
    r = await client.post("/servo/elbow", data=b"nope", headers={H: key, "Content-Type": "application/json"})
    assert r.status == 400
    r = await client.post("/servo/elbow", data=b"nope", headers={H: key})
    assert r.status == 415


async def test_poses_lists_library(client):
    j = await (await client.get("/poses")).json()
    assert "ready" in j and "joints" in j["ready"]


async def test_frame_and_503(client, fake_cam):
    r = await client.get("/camera/gripper/frame.jpg")
    assert r.status == 200 and r.headers["Content-Type"] == "image/jpeg" and "X-Frame-Age" in r.headers
    assert (await client.get("/camera/nope/frame.jpg")).status == 404
    fake_cam.down = True
    await asyncio.sleep(2.5)
    assert (await client.get("/camera/gripper/frame.jpg")).status == 503
    assert (await (await client.get("/cameras")).json())["gripper"]["ok"] is False


async def test_stream_sends_a_part(client):
    r = await client.get("/camera/gripper/stream")
    assert r.status == 200 and r.headers["Content-Type"].startswith("multipart/x-mixed-replace")
    chunk = await r.content.read(64)
    assert b"--f" in chunk
    r.close()


async def test_ws_streams_state_and_result(client, key):
    ws = await client.ws_connect("/ws")
    first = await ws.receive_json()
    assert first["type"] == "state" and "servos" in first and "cameras" in first
    await client.post("/servo/elbow", json={"deg": 3}, headers={H: key})
    types = []
    for _ in range(30):
        m = await asyncio.wait_for(ws.receive_json(), 2)
        types.append(m["type"])
        if "result" in types:
            break
    assert "result" in types
    await ws.close()


async def test_control_released_frees_bus(client, key):
    r = await client.post("/control", json={"holder": "released"}, headers={H: key})
    assert r.status == 200 and (await r.json())["holder"] == "released"
    await asyncio.sleep(0.1)
    assert (await (await client.get("/state")).json())["bus"] == "released"
    r = await client.post("/servo/elbow", json={"deg": 5}, headers={H: key})
    assert r.status == 409 and (await r.json())["reason"] == "bus released"
    await client.post("/control", json={"holder": "human"}, headers={H: key})
    await asyncio.sleep(0.1)
    assert (await (await client.get("/health")).json())["ok"] is True


async def test_ws_two_clients_both_get_results(client, key):
    ws1 = await client.ws_connect("/ws"); ws2 = await client.ws_connect("/ws")
    await ws1.receive_json(); await ws2.receive_json()
    await client.post("/servo/elbow", json={"deg": 3}, headers={H: key})
    async def got_result(ws):
        for _ in range(40):
            m = await asyncio.wait_for(ws.receive_json(), 2)
            if m["type"] == "result":
                return True
        return False
    assert await got_result(ws1) and await got_result(ws2)
    await ws1.close(); await ws2.close()


async def test_write_requires_json_content_type(client, key):
    r = await client.post("/servo/elbow", data=b'{"deg": 5}', headers={H: key, "Content-Type": "text/plain"})
    assert r.status == 415


async def test_foreign_origin_rejected(client, key):
    r = await client.post("/servo/elbow", json={"deg": 5}, headers={H: key, "Origin": "https://evil.example"})
    assert r.status == 403
    r = await client.post("/servo/elbow", json={"deg": 5}, headers={H: key, "Origin": "http://127.0.0.1:8800"})
    assert r.status == 200
    import aiohttp, pytest
    with pytest.raises(aiohttp.WSServerHandshakeError):
        await client.ws_connect("/ws", headers={"Origin": "https://evil.example"})


async def test_demo_endpoints(client_t, key):
    r = await client_t.post("/demo/start", json={"task": "pick up the carton"})
    assert r.status == 200
    d = await r.json()
    assert d["id"].endswith("pick-up-the-carton")
    assert (await client_t.post("/demo/start", json={"task": "x"})).status == 409
    await client_t.post("/control", json={"holder": "leader"}, headers={H: key})
    await asyncio.sleep(0.2)
    await client_t.post("/control", json={"holder": "human"}, headers={H: key})
    await asyncio.sleep(0.1)
    r = await client_t.post("/demo/stop")
    assert r.status == 200 and (await r.json())["rows"] > 0
    assert (await client_t.post("/demo/stop")).status == 409
    lst = await (await client_t.get("/demos")).json()
    assert lst[0]["id"] == d["id"]
    full = await (await client_t.get(f"/demo/{d['id']}")).json()
    assert full["meta"]["task"] == "pick up the carton" and full["rows"][0]["image"] == "gripper/000000.jpg"
    img = await client_t.get(f"/demo/{d['id']}/image/gripper/000000.jpg")
    assert img.status == 200 and img.headers["Content-Type"] == "image/jpeg"
    assert (await client_t.get("/demo/nope")).status == 404
    r = await client_t.post(f"/demo/{d['id']}/replay", json={"every": 5}, headers={H: key})
    assert r.status == 200 and (await r.json())["outcome"] == "ok"
    assert (await client_t.post(f"/demo/{d['id']}/replay", json={})).status == 409   # agent, human holds


async def test_dashboard_served_with_key(aiohttp_client, worker, control, cams):
    from api import build_app
    c = await aiohttp_client(build_app(control, worker, cams, human_key="sekret"))
    r = await c.get("/")
    assert r.status == 200 and r.headers["Content-Type"].startswith("text/html")
    html = await r.text()
    assert 'window.__ARM_KEY__ = "sekret"' in html and "<title>Arm Dashboard</title>" in html
    assert (await c.get("/dashboard")).status == 200


async def test_dashboard_without_key_is_view_only(client):
    html = await (await client.get("/")).text()
    assert 'window.__ARM_KEY__ = ""' in html


async def test_demo_stop_passes_extra(client_t, key):
    await client_t.post("/demo/start", json={"task": "t"})
    r = await client_t.post("/demo/stop", json={"reported_outcome": "success", "notes": "clean"})
    assert r.status == 200 and (await r.json())["reported_outcome"] == "success"
    full = await (await client_t.get("/demos")).json()
    assert full[-1]["reported_outcome"] == "success" and full[-1]["notes"] == "clean"


async def test_demo_search_and_summary(client_t, key, worker_t):
    from tests.test_demolib import make_demo
    make_demo(worker_t.demo_dir)
    r = await client_t.get("/demos?q=pick%20the%20box")
    hits = await r.json()
    assert hits and hits[0]["id"] == "20261023-034820-pick-up-box" and hits[0]["key"]["grasp_tool_z_cm"] > 0
    assert (await (await client_t.get("/demos?q=banana")).json()) == []
    s = await (await client_t.get("/demo/20261023-034820-pick-up-box/summary")).json()
    assert s["outcome"] == "grasped_and_released" and [p["name"] for p in s["phases"]][3] == "grasp"
    assert (await client_t.get("/demo/nope/summary")).status == 404
    md = await client_t.get("/demo/20261023-034820-pick-up-box/image/README.md")
    assert md.status == 200


async def test_demo_stop_writes_summary(client_t, key, worker_t):
    await client_t.post("/demo/start", json={"task": "wave"})
    await client_t.post("/control", json={"holder": "leader"}, headers={H: key})
    await asyncio.sleep(0.2)
    await client_t.post("/control", json={"holder": "human"}, headers={H: key})
    await asyncio.sleep(0.1)
    d = await (await client_t.post("/demo/stop")).json()
    assert "outcome" in d and (worker_t.demo_dir / d["id"] / "summary.json").exists()


async def test_vision_endpoints(client_v):
    r = await client_v.get("/vision")
    j = await r.json()
    assert r.status == 200 and j["detections"][0]["label"] == "box" and "grid" in j
    r = await client_v.get("/vision/frame.jpg")
    assert r.status == 200 and r.headers["Content-Type"] == "image/jpeg"
    r = await client_v.post("/vision/prompts", json={"prompts": ["cup"]})
    assert r.status == 200 and (await r.json())["prompts"] == ["cup"]
    r = await client_v.get("/vision/stream")
    assert r.status == 200 and r.headers["Content-Type"].startswith("multipart/x-mixed-replace")
    r.close()


async def test_vision_absent_is_503(client):
    assert (await client.get("/vision")).status == 503
    assert (await client.get("/vision/frame.jpg")).status == 503


async def test_kinematics_chain_for_dashboard(client):
    r = await client.get("/kinematics")
    assert r.status == 200
    k = await r.json()
    names = [c["joint"] for c in k["chain"]]
    assert names == ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper_frame_joint"]
    for c in k["chain"]:
        assert {"link", "parent", "joint", "type", "xyz", "rpy", "axis"} <= set(c)
        assert len(c["xyz"]) == 3 and len(c["rpy"]) == 3 and len(c["axis"]) == 3
    assert set(k["offset_deg"]) == {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"}
    assert k["jaw"]["joint"] == "gripper" and k["jaw"]["parent"] == "gripper_link"
    assert k["jaw"]["lower_rad"] < 0 < k["jaw"]["upper_rad"]
    assert k["table_z_m"] == 0.015
    assert {"base_link", "upper_arm_link", "gripper_link", "moving_jaw_so101_v1_link"} <= set(k["visuals"])
    v = k["visuals"]["upper_arm_link"]
    assert any(x["mesh"] == "upper_arm_so101_v1.stl" and len(x["xyz"]) == 3 and len(x["rpy"]) == 3 for x in v)


async def test_meshes_served(client):
    r = await client.get("/meshes/upper_arm_so101_v1.stl")
    assert r.status == 200 and len(await r.read()) > 1000


async def test_static_three_served(client):
    r = await client.get("/static/three.module.min.js")
    assert r.status == 200 and "javascript" in r.headers["Content-Type"]
    assert (await client.get("/static/OrbitControls.js")).status == 200
    assert (await client.get("/static/three.core.min.js")).status == 200
    assert (await client.get("/static/../api.py")).status in (403, 404)


async def test_vision_settings_endpoint(client_v):
    r = await client_v.post("/vision/settings", json={"grid_cm": 1, "min_conf": 0.3})
    j = await r.json()
    assert r.status == 200 and j["grid_cm"] == 1 and j["min_conf"] == 0.3 and j["heights"] == {}
    await asyncio.sleep(0.1)
    assert (await (await client_v.get("/vision")).json())["grid"]["spacing_cm"] == 1


async def test_vision_raw_view_and_heights(client_v):
    r = await client_v.get("/vision/raw.jpg")
    assert r.status == 200 and r.headers["Content-Type"] == "image/jpeg" and "X-Frame-Age" in r.headers
    j = await (await client_v.get("/vision")).json()
    assert "jaw_band" in j and "raw_axes" in j and "between_jaws" in j["detections"][0]
    r = await client_v.post("/vision/settings", json={"heights": {"box": 5}})
    assert (await r.json())["heights"] == {"box": 5.0}
    await asyncio.sleep(0.3)
    j = await (await client_v.get("/vision")).json()
    assert j["heights_cm"] == {"box": 5.0}              # (this fixture's arm is not looking down, so no plane metrics to re-measure)


async def test_remote_agent_header_and_toggle(client, key):
    R = {"X-Arm-Agent": "remote"}
    await client.post("/control", json={"holder": "agent"}, headers={H: key})
    r = await client.post("/servo/elbow", json={"deg": 5}, headers=R)
    assert r.status == 200 and (await r.json())["agent"] == "remote"
    assert (await client.post("/control", json={"remote_allowed": False})).status == 403
    r = await client.post("/control", json={"remote_allowed": False}, headers={H: key})
    assert r.status == 200 and (await r.json())["agents"]["remote_allowed"] is False
    r = await client.post("/servo/elbow", json={"deg": 6}, headers=R)
    assert r.status == 409 and (await r.json())["reason"] == "remote agent blocked on the dashboard"
    r = await client.post("/servo/elbow", json={"deg": 6})
    assert r.status == 200 and (await r.json())["agent"] == "local"
    s = await (await client.get("/state")).json()
    assert s["control"]["holder"] == "agent" and s["control"]["agents"]["local"]["active"] is True
    r = await client.post("/servo/elbow", json={"deg": 7}, headers=R)
    assert r.status == 409
