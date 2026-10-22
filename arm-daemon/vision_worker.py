"""VisionWorker: annotates the newest gripper frame with the current joints at a fixed rate and keeps
the latest annotated JPEG + info for the API, the dashboard and the MCP look() tool."""
import threading
import time

import cv2
import numpy as np

import camview


class VisionWorker(threading.Thread):
    def __init__(self, frame_fn, joints_fn, holder_fn=None, detector=None, hz: float = 5.0, prompts=None):
        super().__init__(name="vision", daemon=True)
        self.frame_fn, self.joints_fn, self.holder_fn = frame_fn, joints_fn, holder_fn or (lambda: None)
        self.detector = detector
        self.hz = hz
        self._prompts = list(prompts or [])
        self._settings = {"grid_cm": camview.GRID_CM, "min_conf": camview.MIN_CONF, "heights": {}}
        self._exit = threading.Event()
        self._lock = threading.Lock()
        self._jpg = None
        self._raw_jpg = None
        self._t = None
        self._info = {"detections": [], "prompts": self._prompts, "error": "starting"}

    def frame(self):
        with self._lock:
            return (self._jpg, round(time.time() - self._t, 3)) if self._jpg else None

    def raw_frame(self):
        """The raw (unrotated) frame with the jaw band drawn on it, same age as frame()."""
        with self._lock:
            return (self._raw_jpg, round(time.time() - self._t, 3)) if self._raw_jpg else None

    def frame_time(self):
        with self._lock:
            return self._t

    def info(self) -> dict:
        with self._lock:
            return dict(self._info)

    def set_prompts(self, prompts) -> list:
        with self._lock:
            self._prompts = [str(p).strip() for p in (prompts or []) if str(p).strip()]
            return list(self._prompts)

    def set_settings(self, d: dict) -> dict:
        with self._lock:
            g = d.get("grid_cm")
            if g in (1, 5):
                self._settings["grid_cm"] = int(g)
            try:
                m = float(d["min_conf"])
                if 0 <= m <= 1:
                    self._settings["min_conf"] = m
            except (KeyError, TypeError, ValueError):
                pass
            h = d.get("heights")
            if isinstance(h, dict):                 # {label: cm above the table}; {} clears, a 0/None value removes one label
                cur = dict(self._settings["heights"])
                if not h:
                    cur = {}
                for k, v in h.items():
                    try:
                        fv = float(v) if v is not None else 0.0
                    except (TypeError, ValueError):
                        continue
                    if fv > 0:
                        cur[str(k).strip().lower()] = fv
                    else:
                        cur.pop(str(k).strip().lower(), None)
                self._settings["heights"] = cur
            return dict(self._settings)

    def settings(self) -> dict:
        with self._lock:
            return dict(self._settings)

    def prompts(self) -> list:
        with self._lock:
            return list(self._prompts)

    def stop(self):
        self._exit.set()
        self.join(timeout=3)

    def run(self):
        while not self._exit.is_set():
            t0 = time.time()
            try:
                f = self.frame_fn()
                if not f:
                    with self._lock:
                        self._info = {"t": time.time(), "detections": [], "prompts": self._prompts, "error": "no camera frame"}
                        self._jpg = None
                        self._raw_jpg = None
                else:
                    raw = cv2.imdecode(np.frombuffer(f[0], np.uint8), cv2.IMREAD_COLOR)
                    q = self.joints_fn()
                    st = self.settings()
                    holder = self.holder_fn()
                    jpg, info = camview.annotate(raw, q, self.detector, prompts=self.prompts() or None, holder=holder,
                                                 grid_cm=st["grid_cm"], min_conf=st["min_conf"], heights=st.get("heights") or None)
                    raw_jpg, info = camview.annotate_raw(raw, q, info, q.get("gripper") if isinstance(q, dict) else None, holder=holder)
                    info["frame_age_s"] = f[1]
                    with self._lock:
                        self._jpg, self._raw_jpg, self._t, self._info = jpg, raw_jpg, time.time(), info
            except Exception as e:
                with self._lock:
                    self._info = {"t": time.time(), "detections": [], "prompts": self._prompts, "error": f"{type(e).__name__}: {e}"}
            self._exit.wait(max(0.0, 1.0 / self.hz - (time.time() - t0)))
