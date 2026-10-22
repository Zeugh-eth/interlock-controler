import asyncio
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer


class FakeDaemon:
    """Answers like arm-daemon for the few calls the MCP tests make; records requests."""

    def __init__(self):
        self.calls = []
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype="application/json"):
                self.send_response(code); self.send_header("Content-Type", ctype); self.end_headers()
                self.wfile.write(body if isinstance(body, bytes) else json.dumps(body).encode())

            def do_GET(self):
                outer.calls.append(("GET", self.path, None))
                if self.path == "/state":
                    self._send(200, {"bus": "ok", "control": {"holder": "agent"}, "servos": {"elbow_flex": {"deg": 1.0}},
                                     "tool": {}, "motion": {"state": "idle"}, "cameras": {"gripper": {"ok": True, "age_s": 0.02}}, "stop": {"set": False}})
                elif self.path == "/camera/gripper/frame.jpg":
                    self._send(200, b"\xff\xd8fake", "image/jpeg")
                elif self.path.startswith("/demos?q="):
                    self._send(200, [{"id": "d1", "task": "pick box", "score": 5, "key": {"grasp_tool_z_cm": 3.7}, "phases": []}])
                elif self.path == "/demos":
                    self._send(200, [{"id": "d1", "task": "wave", "rows": 3}])
                elif self.path.startswith("/demo/d1/image/"):
                    self._send(200, b"\xff\xd8demo", "image/jpeg")
                elif self.path.startswith("/demo/d1"):
                    self._send(200, {"meta": {"id": "d1", "task": "wave"}, "rows": [{"t": 1, "state": {}, "action": {}, "image": "gripper/000000.jpg"}]})
                else:
                    self._send(404, {"refused": "nope"})

            def do_POST(self):
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                outer.calls.append(("POST", self.path, body, self.headers.get("X-Arm-Human-Key")))
                if self.path == "/servo/elbow":
                    self._send(409, {"outcome": "refused", "reason": "human has control", "final": {}})
                else:
                    self._send(200, {"outcome": "ok", "reason": "", "final": {"elbow_flex": 5.0}})

        self.srv = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_port}"
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()


def _post(fd):
    return [c for c in fd.calls if c[0] == "POST"][-1]


def _mcp(url):
    import importlib, sys
    sys.argv = ["mcp_server.py"]
    m = importlib.import_module("mcp_server")
    m.BASE = url
    return m


def test_tool_list_and_calls():
    fd = FakeDaemon()
    m = _mcp(fd.url)
    names = {t.name for t in asyncio.run(m.server.list_tools())}
    assert {"state", "look", "set_servo", "nudge", "move", "gripper", "pose", "stop", "resume",
            "list_demos", "read_demo", "replay_demo", "demo_start", "demo_stop"} <= names
    r = m.set_servo("elbow", 5.0)
    assert r.is_error is False and r.structured_content["outcome"] == "refused"
    assert _post(fd)[:3] == ("POST", "/servo/elbow", {"deg": 5.0}) and _post(fd)[3] is None   # never sends a key
    r = m.gripper(40)
    assert _post(fd)[2] == {"pct": 40.0}
    r = m.look()
    assert any(c.type == "image" for c in r.content)
    r = m.state()
    assert r.structured_content["control"]["holder"] == "agent"
    r = m.read_demo("d1", with_images=True)
    assert r.structured_content["meta"]["task"] == "wave" and any(c.type == "image" for c in r.content)
    r = m.replay_demo("d1", every=5)
    assert _post(fd)[1] == "/demo/d1/replay" and _post(fd)[2]["every"] == 5
    r = m.nudge("wrist", -3)
    assert _post(fd)[1] == "/servo/wrist/nudge" and _post(fd)[2] == {"delta": -3.0}
    r = m.move({"elbow": 1, "wrist": 2}, speed=20)
    assert _post(fd)[2]["joints"] == {"elbow": 1, "wrist": 2} and _post(fd)[2]["speed"] == 20


def test_daemon_down_is_a_clear_error():
    m = _mcp("http://127.0.0.1:1")
    r = m.state()
    assert r.is_error is True and "arm-daemon" in r.content[0].text


def test_find_demos_tool():
    fd = FakeDaemon()
    m = _mcp(fd.url)
    r = m.find_demos("pick box")
    assert fd.calls[-1][1].startswith("/demos?q=pick") and r.is_error is False
    names = {t.name for t in asyncio.run(m.server.list_tools())}
    assert "find_demos" in names


def test_skill_tools_over_stateful_fake():
    from tests.fake_daemon import StatefulFakeDaemon
    fd = StatefulFakeDaemon()
    m = _mcp(fd.url)
    names = {t.name for t in asyncio.run(m.server.list_tools())}
    assert {"observe", "goto_xyz", "move_tool", "project", "descend_to_contact", "grasp", "release"} <= names
    r = m.observe()
    assert r.structured_content["stale"] is False and any(c.type == "image" for c in r.content)
    r = m.move_tool(dz_m=-0.02)
    assert r.structured_content["outcome"] == "ok"
    fd.contact_z = 0.03
    r = m.descend_to_contact(max_cm=8, step_cm=0.5)
    assert r.structured_content["contact"] is True
    fd.object_width_pct = 8.0
    r = m.grasp()
    assert r.structured_content["verdict"] == "held" and any(c.type == "image" for c in r.content)
    r = m.release()
    assert r.structured_content["outcome"] == "ok"
    r = m.project(320, 240)
    assert "x_m" in r.structured_content
    m.demo_stop(reported_outcome="success", notes="n")
    assert fd.demo_stop_body == {"reported_outcome": "success", "notes": "n"}
    fd.close()


def test_plan_tools_over_stateful_fake():
    from tests.fake_daemon import StatefulFakeDaemon
    fd = StatefulFakeDaemon()
    fd.object = {"label": "carton", "x": 0.24, "y": -0.06, "w_cm": 7.0, "h_cm": 10.5, "conf": 0.3}
    fd.object_width_pct = 45.0
    m = _mcp(fd.url)
    names = {t.name for t in asyncio.run(m.server.list_tools())}
    assert {"analyze_object", "grab", "evaluate_grasp", "replan"} <= names
    a = m.analyze_object("carton").structured_content
    assert a["plan"]["feasible"] is True and a["plan"]["roll_deg"] == 90
    g = m.grab(a["plan"]).structured_content
    assert g["evaluation"]["quality"] == "good"
    ev = m.evaluate_grasp({"verdict": "held", "jaw_pct_after_close": 3.0}, a["plan"]).structured_content
    assert ev["quality"] == "edge"
    p2 = m.replan(a["plan"], ev).structured_content
    assert p2["attempt"] == 2 and p2["shift_along_jaws_cm"] != 0
    fd.close()
