"""Optional device lock on agent mode (daemon flag --auth-lock).

At startup the daemon draws a random 6-digit code and never shows it. The dashboard shows a QR
payload instead: for every listed user, the code masked with a pad only that user's key can
rebuild. The user's device (the SenseCAP Watcher) scans the QR, removes its pad and shows the
code; the human types it into the dashboard. One correct code unlocks agent mode until the daemon
stops. Too many wrong codes draw a new code and a new QR.

    payload = ARMLOCK1:<salt>:<uid>=<masked>;<uid>=<masked>...
    pad     = HMAC-SHA256(key, "ARMLOCK1:<salt>:<uid>") first 8 bytes, big-endian, mod 10^6
    masked  = (code + pad) mod 10^6        code = (masked - pad) mod 10^6   (6 digits, zero-padded)

watcher_code() is the device side; the Watcher firmware ports those few lines. Run this file to
play the Watcher from a terminal:  python authlock.py --uid zeugh --key 42069 <payload | qr.png>
"""
import base64
import hashlib
import hmac
import json
import secrets
from pathlib import Path

TAG = "ARMLOCK1"
DIGITS = 6
MOD = 10 ** DIGITS


def _pad(key: str, salt: str, uid: str) -> int:
    mac = hmac.new(key.encode("utf-8"), f"{TAG}:{salt}:{uid}".encode("utf-8"), hashlib.sha256).digest()
    return int.from_bytes(mac[:8], "big") % MOD


def watcher_code(payload: str, uid: str, key: str):
    """The code the device shows after scanning payload, or None if uid is not listed in it."""
    parts = str(payload).split(":", 2)
    if len(parts) != 3 or parts[0] != TAG:
        return None
    salt, entries = parts[1], parts[2]
    for entry in entries.split(";"):
        name, _, masked = entry.partition("=")
        if name == uid and masked.isdigit():
            return f"{(int(masked) - _pad(key, salt, uid)) % MOD:0{DIGITS}d}"
    return None


def load_users(path: Path) -> dict:
    """{uid: key} from a JSON file. Refuses an empty or malformed list: a lock nobody can open."""
    users = json.loads(Path(path).read_text())
    if not isinstance(users, dict) or not users or not all(
            isinstance(u, str) and isinstance(k, str) and u and k and ";" not in u and "=" not in u
            for u, k in users.items()):
        raise ValueError(f"{path}: expected a non-empty JSON object of uid -> key strings")
    return users


class AuthLock:
    def __init__(self, users: dict, max_tries: int = 5):
        self._users = dict(users)
        self._max_tries = max_tries
        self.unlocked = False
        self._new_code()

    def _new_code(self):
        self._code = f"{secrets.randbelow(MOD):0{DIGITS}d}"
        self._salt = secrets.token_hex(4)
        self._tries = 0

    def payload(self) -> str:
        masked = ";".join(f"{uid}={(int(self._code) + _pad(key, self._salt, uid)) % MOD:0{DIGITS}d}"
                          for uid, key in self._users.items())
        return f"{TAG}:{self._salt}:{masked}"

    def qr_data_url(self, scale: int = 10) -> str:
        import cv2
        p = cv2.QRCodeEncoder.Params()
        p.correction_level = cv2.QRCodeEncoder_CORRECT_LEVEL_Q          # the Watcher reads a 240 px frame: favour recovery
        m = cv2.QRCodeEncoder.create(p).encode(self.payload())          # includes the quiet zone
        m = cv2.resize(m, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        ok, png = cv2.imencode(".png", m)
        return "data:image/png;base64," + base64.b64encode(png.tobytes()).decode("ascii")

    def unlock(self, code):
        if self.unlocked:
            return True, ""
        if hmac.compare_digest(str(code).strip().encode("utf-8"), self._code.encode("utf-8")):
            self.unlocked = True
            return True, ""
        self._tries += 1
        if self._tries >= self._max_tries:
            self._new_code()
            return False, "too many wrong codes: new QR"
        return False, "wrong code"

    def status(self) -> dict:
        return {"lock": True, "unlocked": self.unlocked}


# Injected into the dashboards when the lock is on: a click on "agent" while locked opens the QR
# and a code box instead of switching. Capture phase, so the page's own handler never sees it.
DASHBOARD_JS = r"""<script>
(() => {
  const H = {"Content-Type": "application/json", "X-Arm-Human-Key": window.__ARM_KEY__ || ""};
  let unlocked = __UNLOCKED__, box = null;
  async function challenge() {
    const r = await fetch("/auth/challenge", {headers: H}); return r.json();
  }
  async function toAgent() {
    await fetch("/control", {method: "POST", headers: H, body: JSON.stringify({holder: "agent"})});
  }
  function close() { if (box) { box.remove(); box = null; } }
  async function open() {
    const ch = await challenge();
    if (ch.unlocked) { unlocked = true; return toAgent(); }
    if (!ch.qr) { alert(ch.refused || "auth lock unavailable"); return; }
    close();
    box = document.createElement("div");
    box.style.cssText = "position:fixed;inset:0;background:rgba(0,0,0,.45);display:flex;align-items:center;justify-content:center;z-index:9999";
    box.innerHTML = `<div style="background:#fff;color:#111;border-radius:12px;padding:20px;width:min(420px,92vw);font:14px system-ui,sans-serif;text-align:center">
      <div style="font-weight:600;font-size:16px;margin-bottom:4px">Unlock agent mode</div>
      <div style="color:#555;margin-bottom:12px">Scan with your Watcher, then type the 6 digits it shows.</div>
      <img alt="unlock QR" title="click: full screen" style="width:min(360px,100%,calc(100vh - 190px));image-rendering:pixelated;display:block;margin:0 auto 12px;cursor:zoom-in">
      <input inputmode="numeric" maxlength="6" autocomplete="off" placeholder="000000"
        style="font:600 22px ui-monospace,monospace;letter-spacing:6px;text-align:center;width:180px;padding:6px;border:1px solid #bbb;border-radius:8px">
      <div class="m" style="color:#b00020;min-height:18px;margin:8px 0"></div>
      <div style="display:flex;gap:8px;justify-content:center">
        <button class="c" style="padding:6px 14px">Cancel</button><button class="u" style="padding:6px 14px;font-weight:600">Unlock</button></div></div>`;
    document.body.appendChild(box);
    const img = box.querySelector("img"), inp = box.querySelector("input"), msg = box.querySelector(".m");
    img.src = ch.qr; inp.focus();
    img.onclick = () => {                     // full-window QR for the Watcher's camera; click again to return
      const full = document.createElement("div");
      full.style.cssText = "position:fixed;inset:0;background:#fff;display:flex;align-items:center;justify-content:center;z-index:10000;cursor:zoom-out";
      full.innerHTML = `<img src="${img.src}" alt="unlock QR" style="width:min(96vw,96vh);image-rendering:pixelated">`;
      full.onclick = () => { full.remove(); inp.focus(); };
      document.body.appendChild(full);
    };
    box.querySelector(".c").onclick = close;
    box.onclick = e => { if (e.target === box) close(); };
    const submit = async () => {
      const r = await fetch("/auth/unlock", {method: "POST", headers: H, body: JSON.stringify({code: inp.value})});
      const j = await r.json().catch(() => ({}));
      if (r.ok) { unlocked = true; close(); return toAgent(); }
      msg.textContent = j.refused || "refused"; inp.value = ""; inp.focus();
      const again = await challenge(); if (again.qr) img.src = again.qr;
    };
    box.querySelector(".u").onclick = submit;
    inp.onkeydown = e => { if (e.key === "Enter") submit(); if (e.key === "Escape") close(); };
  }
  document.addEventListener("click", e => {
    if (unlocked || !e.target.closest('[data-holder="agent"]')) return;
    e.preventDefault(); e.stopImmediatePropagation(); open();
  }, true);
})();
</script>"""


def dashboard_js(lock) -> str:
    return DASHBOARD_JS.replace("__UNLOCKED__", "true" if lock.unlocked else "false")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="Play the Watcher: turn a QR payload (or a screenshot of it) into the code.")
    ap.add_argument("source", help="the ARMLOCK1:... payload, or an image file with the QR in it")
    ap.add_argument("--uid", required=True)
    ap.add_argument("--key", required=True)
    a = ap.parse_args()
    payload = a.source
    if Path(a.source).is_file():
        import cv2
        payload = cv2.QRCodeDetector().detectAndDecode(cv2.imread(a.source))[0]
    code = watcher_code(payload, a.uid, a.key)
    print(code if code else f"not listed: {a.uid} is not in this QR")
