"""HTTP + WebSocket API for arm-daemon. Maps routes to ControlState, BusWorker and CameraWorker.
Human = request header X-Arm-Human-Key equal to the daemon's key. Agent = anything else."""
import asyncio
import json
import time
from pathlib import Path

from aiohttp import web

import guards

HUMAN_HEADER = "X-Arm-Human-Key"
WS_HZ = 20.0


def _refused(status, reason):
    return web.json_response({"refused": reason}, status=status)


ALLOWED_ORIGINS = {f"http://{h}:{p}" for h in ("127.0.0.1", "localhost") for p in range(8000, 9000)}


def origin_ok(request):
    o = request.headers.get("Origin")
    return o is None or o in ALLOWED_ORIGINS


def _err(exc, reason):
    return exc(text=json.dumps({"refused": reason}), content_type="application/json")


async def _body(request):
    """Parse a write's JSON body; also where cross-site requests are turned away (a browser
    page cannot send application/json to another origin without a preflight, and we never answer one)."""
    if not origin_ok(request):
        raise _err(web.HTTPForbidden, "foreign origin")
    if not request.can_read_body:
        return {}
    if not request.content_type.startswith("application/json"):
        raise _err(web.HTTPUnsupportedMediaType, "body must be application/json")
    try:
        b = await request.json()
    except Exception:
        raise _err(web.HTTPBadRequest, "body must be JSON")
    return b if isinstance(b, dict) else {}


DASHBOARD = Path(__file__).resolve().parent / "dashboard.html"
STATIC = Path(__file__).resolve().parent / "static"      # vendored three.js for the dashboard's 3D view
MESHES = Path(__file__).resolve().parents[1] / "vendor" / "so101" / "assets"   # SO-101 STL meshes (TheRobotStudio/SO-ARM100)


def build_app(control, bus, cams, human_key: str | None = None, vision=None) -> web.Application:
    app = web.Application()
    app["t0"] = time.time()
    app["ws_clients"] = {}          # ws -> per-client bounded result queue

    def human(request):
        return control.is_human(request.headers.get(HUMAN_HEADER))

    def state():
        s = bus.snapshot()
        s["cameras"] = cams.health()
        if vision is not None:
            vi = vision.info()
            s["vision"] = {"detections": vi.get("detections", []), "tool": vi.get("tool"), "prompts": vi.get("prompts"),
                           "error": vi.get("error"), "ms": vi.get("ms")}
        return s

    async def submit(request, cmd):
        loop = asyncio.get_running_loop()
        r = await loop.run_in_executor(None, bus.submit, cmd, human(request))
        return web.json_response(r.to_dict(), status=409 if r.outcome == "refused" else 200)

    # --- dashboard -----------------------------------------------------------------------------
    async def get_dashboard(request):
        """The human dashboard. The human key is injected here: a page served from this origin is
        the dashboard; cross-origin pages cannot read this response (no CORS headers are sent)."""
        html = DASHBOARD.read_text()
        inject = f'<script>window.__ARM_KEY__ = {json.dumps(human_key or "")};</script>'
        html = html.replace("<script>\nconst KEY", inject + "\n<script>\nconst KEY", 1)
        return web.Response(text=html, content_type="text/html", headers={"Cache-Control": "no-store"})

    # --- reads ---------------------------------------------------------------------------------
    async def get_state(request):
        return web.json_response(state())

    async def get_health(request):
        s = bus.snapshot()
        return web.json_response({"ok": s["bus"] == "ok", "bus": s["bus"], "uptime_s": round(time.time() - app["t0"], 1),
                                  "holder": s["control"]["holder"], "stop": s["stop"]["set"]})

    async def get_servo(request):
        name = guards.resolve_servo(request.match_info["name"])
        if name is None:
            return _refused(404, f"unknown servo {request.match_info['name']}")
        e = dict(bus.snapshot()["servos"].get(name, {}))
        e.update(name=name, limits=[round(v, 1) for v in guards.LIMITS[name]],
                 aliases=[a for a, j in guards.ALIASES.items() if j == name])
        return web.json_response(e)

    kinematics = guards.kinematics_info()

    async def get_kinematics(request):
        return web.json_response(kinematics)

    async def get_poses(request):
        return web.json_response({n: {"joints": p["joints"], "aliases": p.get("aliases", []), "via": p.get("via"),
                                      "hold_after": p.get("hold_after", True)} for n, p in bus.poses.items()})

    async def get_cameras(request):
        return web.json_response(cams.health())

    async def get_frame(request):
        cam = request.match_info["cam"]
        if cam not in cams.names:
            return _refused(404, f"unknown camera {cam}")
        f = cams.frame(cam)
        if f is None or not cams.health()[cam]["ok"]:
            return _refused(503, "no frame")
        jpg, age = f
        return web.Response(body=jpg, content_type="image/jpeg", headers={"X-Frame-Age": str(age), "Cache-Control": "no-store"})

    async def get_stream(request):
        cam = request.match_info["cam"]
        if cam not in cams.names:
            return _refused(404, f"unknown camera {cam}")
        resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=f", "Cache-Control": "no-store"})
        await resp.prepare(request)
        last = None
        try:
            while True:
                f = cams.frame(cam)
                ft = cams.frame_time(cam)
                if f and ft != last:
                    last = ft
                    await resp.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(f[0])).encode() + b"\r\n\r\n" + f[0] + b"\r\n")
                await asyncio.sleep(0.02)
        except (ConnectionResetError, asyncio.CancelledError, ConnectionError):
            pass
        return resp

    # --- writes --------------------------------------------------------------------------------
    async def post_servo(request):
        raw = request.match_info["name"]
        if guards.resolve_servo(raw) is None:
            return _refused(404, f"unknown servo {raw}")
        b = await _body(request)
        return await submit(request, {"op": "servo", "servo": raw, "body": {k: b[k] for k in ("deg", "pct") if k in b},
                                      "speed": b.get("speed"), "ignore_table": bool(b.get("ignore_table"))})

    async def post_nudge(request):
        raw = request.match_info["name"]
        if guards.resolve_servo(raw) is None:
            return _refused(404, f"unknown servo {raw}")
        b = await _body(request)
        return await submit(request, {"op": "nudge", "servo": raw, "delta": b.get("delta"), "speed": b.get("speed"),
                                      "ignore_table": bool(b.get("ignore_table"))})

    async def post_move(request):
        b = await _body(request)
        return await submit(request, {"op": "move", "joints": b.get("joints"), "speed": b.get("speed"),
                                      "ignore_table": bool(b.get("ignore_table"))})

    async def post_gripper(request):
        b = await _body(request)
        return await submit(request, {"op": "servo", "servo": "gripper", "body": {k: b[k] for k in ("deg", "pct") if k in b},
                                      "speed": b.get("speed")})

    async def post_pose(request):
        b = await _body(request)
        return await submit(request, {"op": "pose", "name": b.get("name", ""), "ignore_table": bool(b.get("ignore_table"))})

    async def post_torque(request):
        b = await _body(request)
        return await submit(request, {"op": "torque", "on": bool(b.get("on", True)), "confirm": bool(b.get("confirm"))})

    # --- vision (annotated gripper view) --------------------------------------------------------
    async def get_vision(request):
        if vision is None:
            return _refused(503, "vision not configured")
        return web.json_response(vision.info())

    async def get_vision_frame(request):
        if vision is None:
            return _refused(503, "vision not configured")
        f = vision.frame()
        if f is None:
            return _refused(503, vision.info().get("error", "no frame"))
        return web.Response(body=f[0], content_type="image/jpeg", headers={"X-Frame-Age": str(f[1]), "Cache-Control": "no-store"})

    async def get_vision_stream(request):
        if vision is None:
            return _refused(503, "vision not configured")
        resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=f", "Cache-Control": "no-store"})
        await resp.prepare(request)
        last = None
        try:
            while True:
                f = vision.frame()
                ft = vision.frame_time()
                if f and ft != last:
                    last = ft
                    await resp.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(f[0])).encode() + b"\r\n\r\n" + f[0] + b"\r\n")
                await asyncio.sleep(0.05)
        except (ConnectionResetError, asyncio.CancelledError, ConnectionError):
            pass
        return resp

    async def post_vision_prompts(request):
        if vision is None:
            return _refused(503, "vision not configured")
        b = await _body(request)
        p = b.get("prompts")
        if isinstance(p, str):
            p = [x for x in p.replace(";", ",").split(",")]
        return web.json_response({"prompts": vision.set_prompts(p if isinstance(p, list) else [])})

    async def post_vision_settings(request):
        if vision is None:
            return _refused(503, "vision not configured")
        b = await _body(request)
        return web.json_response(vision.set_settings(b))

    # --- demos ---------------------------------------------------------------------------------
    async def post_demo_start(request):
        b = await _body(request)
        d = bus.demo_start(str(b.get("task", "")).strip() or "demo")
        if d is None:
            return _refused(409, "a demo is already recording" if bus.demo_active() else "demos not configured")
        return web.json_response(d)

    async def post_demo_stop(request):
        b = await _body(request)
        d = bus.demo_stop(b if isinstance(b, dict) else None)
        if d is None:
            return _refused(409, "no demo recording")
        return web.json_response(d)

    async def get_demos(request):
        q = request.query.get("q")
        if q:
            import demolib
            if not bus.demo_dir:
                return web.json_response([])
            return web.json_response(demolib.search(bus.demo_dir, q, limit=int(request.query.get("limit", 5))))
        return web.json_response(bus.list_demos())

    async def get_demo_summary(request):
        import demolib
        did = request.match_info["id"]
        if not bus.demo_dir or "/" in did or ".." in did or not (bus.demo_dir / did / "meta.json").exists():
            return _refused(404, "unknown demo")
        s = demolib.load_summary(bus.demo_dir / did, refresh=bool(request.query.get("refresh")))
        return web.json_response(s) if s else _refused(404, "unknown demo")

    async def get_demo(request):
        d = bus.demo(request.match_info["id"])
        if d is None:
            return _refused(404, "unknown demo")
        return web.json_response(d)

    async def get_demo_file(request):
        p = bus.demo_file(request.match_info["id"], request.match_info["name"])
        if p is None:
            return _refused(404, "unknown demo file")
        return web.FileResponse(p, headers={"Cache-Control": "no-store"})

    async def post_replay(request):
        b = await _body(request)
        return await submit(request, {"op": "replay", "id": request.match_info["id"], "every": b.get("every"),
                                      "speed": b.get("speed"), "ignore_table": bool(b.get("ignore_table"))})

    # --- control -------------------------------------------------------------------------------
    async def post_control(request):
        b = await _body(request)
        holder = b.get("holder")
        ok, why = control.set_holder(holder, human(request))
        if not ok:
            return _refused(400 if why == "unknown holder" else 403, why)
        return web.json_response({"holder": control.holder, "since": control.since})

    async def post_stop(request):
        b = await _body(request)
        control.stop(str(b.get("reason", "")), by="human" if human(request) else "agent")
        return web.json_response({"stop": control.stop_state(), "motion": bus.snapshot()["motion"]})

    async def post_resume(request):
        await _body(request)
        ok, why = control.resume(human(request))
        if not ok:
            return _refused(403, why)
        return web.json_response({"stop": control.stop_state()})

    # --- websocket -----------------------------------------------------------------------------
    async def ws_handler(request):
        if not origin_ok(request):
            return _refused(403, "foreign origin")
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        q: asyncio.Queue = asyncio.Queue(maxsize=100)
        app["ws_clients"][ws] = q
        try:
            await ws.send_json({"type": "state", **state()})
            while not ws.closed:
                while not q.empty():
                    await ws.send_json({"type": "result", **q.get_nowait()})
                await ws.send_json({"type": "state", **state()})
                await asyncio.sleep(1.0 / WS_HZ)
        except (ConnectionResetError, asyncio.CancelledError, ConnectionError):
            pass
        finally:
            app["ws_clients"].pop(ws, None)
        return ws

    def fan_out(r):
        for q in list(app["ws_clients"].values()):
            if q.full():
                q.get_nowait()
            q.put_nowait(r)

    async def on_startup(app_):
        loop = asyncio.get_running_loop()
        bus.on_result(lambda r: loop.call_soon_threadsafe(fan_out, r.to_dict()))

    async def on_shutdown(app_):
        for ws in list(app["ws_clients"]):
            await ws.close()

    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.add_routes([
        web.get("/", get_dashboard), web.get("/dashboard", get_dashboard),
        web.get("/state", get_state), web.get("/health", get_health),
        web.get("/servo/{name}", get_servo), web.post("/servo/{name}", post_servo),
        web.post("/servo/{name}/nudge", post_nudge),
        web.post("/move", post_move), web.post("/gripper", post_gripper), web.post("/pose", post_pose),
        web.get("/poses", get_poses), web.post("/torque", post_torque), web.get("/kinematics", get_kinematics),
        web.post("/control", post_control), web.post("/stop", post_stop), web.post("/resume", post_resume),
        web.get("/vision", get_vision), web.get("/vision/frame.jpg", get_vision_frame),
        web.get("/vision/stream", get_vision_stream), web.post("/vision/prompts", post_vision_prompts),
        web.post("/vision/settings", post_vision_settings),
        web.post("/demo/start", post_demo_start), web.post("/demo/stop", post_demo_stop),
        web.get("/demos", get_demos), web.get("/demo/{id}", get_demo), web.get("/demo/{id}/summary", get_demo_summary),
        web.get("/demo/{id}/image/{name:.+}", get_demo_file), web.post("/demo/{id}/replay", post_replay),
        web.get("/cameras", get_cameras), web.get("/camera/{cam}/frame.jpg", get_frame),
        web.get("/camera/{cam}/stream", get_stream), web.get("/ws", ws_handler),
    ])
    app.router.add_static("/static/", STATIC, show_index=False)
    if MESHES.is_dir():
        app.router.add_static("/meshes/", MESHES, show_index=False)
    return app
