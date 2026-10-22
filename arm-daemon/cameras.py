"""CameraWorker: proxies the separate cam-server (:8765). Keeps the newest JPEG per camera and
that camera's health. Never raises out of its loop; a dead cam-server only makes ok=false."""
import json
import threading
import time
import urllib.request

FETCH_TIMEOUT = 1.0
OK_WITHIN_S = 2.0
STALE_S = 5.0


class CameraWorker(threading.Thread):
    def __init__(self, base_url: str = "http://127.0.0.1:8765", names=("gripper", "mac", "phone", "ipad"), hz: float = 10.0):
        super().__init__(name="cameras", daemon=True)
        self.base = base_url.rstrip("/")
        self.names = tuple(names)
        self.hz = hz
        self._exit = threading.Event()
        self._lock = threading.Lock()
        self._open = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._frames = {n: None for n in self.names}      # (jpeg, fetched_at)
        self._health = {n: {"age_s": None, "ok": False, "camera": None} for n in self.names}
        self._server_age = {}

    def frame(self, name: str):
        with self._lock:
            f = self._frames.get(name)
        if not f:
            return None
        return f[0], round(time.time() - f[1], 3)

    def frame_time(self, name: str):
        with self._lock:
            f = self._frames.get(name)
        return f[1] if f else None

    def health(self) -> dict:
        now = time.time()
        out = {}
        with self._lock:
            for n in self.names:
                f = self._frames.get(n)
                h = dict(self._health[n])
                age = round(now - f[1], 3) if f else None
                srv_age = self._server_age.get(n)
                h["age_s"] = age
                h["ok"] = bool(f and age is not None and age < OK_WITHIN_S and srv_age is not None and srv_age < STALE_S)
                out[n] = h
        return out

    def stop(self) -> None:
        self._exit.set()
        self.join(timeout=3)

    def _get(self, path):
        return self._open.open(self.base + path, timeout=FETCH_TIMEOUT).read()

    def run(self):
        n = 0
        while not self._exit.is_set():
            t0 = time.time()
            if n % 10 == 0:
                try:
                    h = json.loads(self._get("/health"))
                    with self._lock:
                        for name in self.names:
                            e = h.get(name) or {}
                            self._health[name]["camera"] = e.get("camera")
                            self._server_age[name] = e.get("frame_age_s")
                except Exception:
                    with self._lock:
                        self._server_age = {}
            n += 1
            for name in self.names:
                try:
                    jpg = self._get(f"/{name}/frame.jpg")
                    with self._lock:
                        self._frames[name] = (jpg, time.time())
                except Exception:
                    with self._lock:
                        f = self._frames.get(name)
                        if f and time.time() - f[1] > OK_WITHIN_S:
                            self._frames[name] = None
            self._exit.wait(max(0.0, 1.0 / self.hz - (time.time() - t0)))
