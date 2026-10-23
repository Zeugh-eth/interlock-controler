import json

import pytest

from authlock import AuthLock, load_users, watcher_code

H = "X-Arm-Human-Key"


def test_watcher_recovers_the_startup_code():
    lock = AuthLock({"zeugh": "42069"})
    assert watcher_code(lock.payload(), "zeugh", "42069") == lock._code
    assert len(lock._code) == 6 and lock._code.isdigit()


def test_wrong_key_or_unknown_user_gets_no_code():
    lock = AuthLock({"zeugh": "42069"})
    assert watcher_code(lock.payload(), "zeugh", "42070") != lock._code
    assert watcher_code(lock.payload(), "mallory", "42069") is None
    assert watcher_code("not a lock code", "zeugh", "42069") is None


def test_every_listed_user_decodes_the_same_code():
    lock = AuthLock({"zeugh": "42069", "ana": "777"})
    assert watcher_code(lock.payload(), "zeugh", "42069") == watcher_code(lock.payload(), "ana", "777") == lock._code


def test_payload_does_not_carry_the_code():
    lock = AuthLock({"zeugh": "42069"})
    assert lock._code not in lock.payload().split(":", 2)[2]


def test_unlock_once_then_stays_unlocked():
    lock = AuthLock({"zeugh": "42069"})
    assert lock.unlock("000000" if lock._code != "000000" else "111111") == (False, "wrong code")
    assert not lock.unlocked
    assert lock.unlock(watcher_code(lock.payload(), "zeugh", "42069")) == (True, "")
    assert lock.unlocked and lock.status() == {"lock": True, "unlocked": True}


def test_too_many_wrong_codes_rotate_the_code():
    lock = AuthLock({"zeugh": "42069"}, max_tries=3)
    old_payload = lock.payload()
    for _ in range(3):
        lock.unlock("x")
    assert lock.payload() != old_payload
    assert lock.unlock(watcher_code(lock.payload(), "zeugh", "42069")) == (True, "")


def test_qr_png_decodes_back_to_payload():
    import base64

    import cv2
    import numpy as np
    lock = AuthLock({"zeugh": "42069"})
    png = base64.b64decode(lock.qr_data_url().split(",", 1)[1])
    img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert cv2.QRCodeDetector().detectAndDecode(img)[0] == lock.payload()


def test_load_users(tmp_path):
    f = tmp_path / "auth-users.json"
    f.write_text(json.dumps({"zeugh": "42069"}))
    assert load_users(f) == {"zeugh": "42069"}
    f.write_text("{}")
    with pytest.raises(ValueError):
        load_users(f)


def test_control_gate(tmp_path):
    from ctrl import ControlState
    c = ControlState("k", tmp_path / "STOP", auth_lock=AuthLock({"zeugh": "42069"}))
    assert c.set_holder("agent", human=True) == (False, "device auth required")
    assert c.set_holder("leader", human=True) == (True, "")
    c.auth_lock.unlock(watcher_code(c.auth_lock.payload(), "zeugh", "42069"))
    assert c.set_holder("agent", human=True) == (True, "")


@pytest.fixture
async def locked_client(aiohttp_client, worker, control, cams):
    from api import build_app
    control.auth_lock = AuthLock({"zeugh": "42069"})
    return await aiohttp_client(build_app(control, worker, cams))


async def test_api_unlock_flow(locked_client, key):
    c = locked_client
    r = await c.post("/control", json={"holder": "agent"}, headers={H: key})
    assert r.status == 403 and (await r.json())["refused"] == "device auth required"
    assert (await c.get("/auth/challenge")).status == 403
    ch = await (await c.get("/auth/challenge", headers={H: key})).json()
    assert ch["lock"] and not ch["unlocked"] and ch["qr"].startswith("data:image/png;base64,")
    assert (await c.post("/auth/unlock", json={"code": "abc"}, headers={H: key})).status == 403
    code = watcher_code(ch["payload"], "zeugh", "42069")
    assert (await c.post("/auth/unlock", json={"code": code})).status == 403
    assert (await c.post("/auth/unlock", json={"code": code}, headers={H: key})).status == 200
    assert (await c.post("/control", json={"holder": "agent"}, headers={H: key})).status == 200
    assert (await (await c.get("/state")).json())["auth"] == {"lock": True, "unlocked": True}


async def test_api_without_lock(client, key):
    assert (await (await client.get("/state")).json())["auth"] == {"lock": False, "unlocked": True}
    assert (await client.get("/auth/challenge", headers={H: key})).status == 404
