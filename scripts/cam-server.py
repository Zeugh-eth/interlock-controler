"""Camera frame server for the arm cameras (localhost only).

Runs where macOS camera permission works (Terminal.app), so other processes can read
frames without their own camera permission. Cameras are selected by exact AVFoundation
NAME through ffmpeg: OpenCV's numeric indexes do not match ffmpeg's device order on macOS
(index 1 once opened the MacBook camera instead of the gripper camera).

  /frame.jpg  /stream            gripper camera ("USB HD Camera")
  /mac/frame.jpg  /mac/stream    MacBook camera ("FaceTime HD Camera"), external view of the arm
  /phone/frame.jpg /phone/stream iPhone Continuity Camera ("Zeugher Camera"), external view
  /ipad/frame.jpg  /ipad/stream  iPad screen over USB (open its Camera app full screen)
  /health                        per-camera frame age

Usage (from tools/arm-lerobot): uv run python ../../scripts/cam-server.py
"""
import json, subprocess, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CAMERAS = {"gripper": "USB HD Camera", "mac": "FaceTime HD Camera", "phone": "Zeugher Camera", "ipad": "iPad"}
# The iPad (USB cable) is not a camera device: its SCREEN is captured by ipadcap (Camera app open
# full screen = live camera view). ffmpeg cannot open iOS screen-capture devices.
IPADCAP = str(__import__("pathlib").Path(__file__).resolve().parent / "ipadcap" / "ipadcap")
HOST, PORT = "127.0.0.1", 8765
# Tried in order until frames arrive; the FaceTime camera rejects some size/rate pairs.
MODES = [["-framerate", "30", "-video_size", "640x480"], ["-framerate", "30", "-video_size", "1280x720"],
         ["-framerate", "30"], []]
latest = {k: {"jpg": None, "t": 0.0, "mode": None} for k in CAMERAS}
lock = threading.Lock()


def capture_loop(key):
    name, mode_i = CAMERAS[key], 0
    while True:
        cmd = [IPADCAP, name] if key == "ipad" else \
              ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "avfoundation", *MODES[mode_i], "-i", name,
               # (the -fflags nobuffer / low_delay / probesize 0 variant froze this camera's frames - reverted)
               "-vf", "scale=640:-2", "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "4", "-flush_packets", "1", "-"]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
        buf, got = b"", False
        while True:
            chunk = proc.stdout.read(65536)   # unbuffered: returns as soon as bytes are available
            if not chunk:
                break
            buf += chunk
            while True:  # split the MJPEG byte stream on JPEG start/end markers
                a = buf.find(b"\xff\xd8")
                b = buf.find(b"\xff\xd9", a + 2)
                if a < 0 or b < 0:
                    break
                with lock:
                    latest[key].update(jpg=buf[a:b + 2], t=time.time(), mode=" ".join(MODES[mode_i]) or "default")
                got, buf = True, buf[b + 2:]
        proc.wait()
        if not got and key != "ipad" and mode_i < len(MODES) - 1:
            mode_i += 1
            print(f"[{key}] mode not accepted; trying {MODES[mode_i] or 'defaults'}")
        else:
            print(f"[{key}] ffmpeg exited; restarting in 1 s")
            time.sleep(1)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _cam(self):
        for k in CAMERAS:
            if self.path.startswith(f"/{k}/"):
                return k
        return "gripper"

    def do_GET(self):
        path = self.path.split("?")[0]
        if path.endswith("/frame.jpg"):
            with lock:
                jpg = latest[self._cam()]["jpg"]
            if jpg is None:
                self.send_error(503, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
        elif path.endswith("/stream"):
            key = self._cam()
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                last_t = 0.0
                while True:
                    with lock:
                        jpg, t = latest[key]["jpg"], latest[key]["t"]
                    if jpg and t != last_t:
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                        self.wfile.flush()
                        last_t = t
                    else:
                        time.sleep(0.005)
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif path == "/health":
            now = time.time()
            body = json.dumps({k: {"camera": CAMERAS[k], "mode": v["mode"],
                                   "frame_age_s": round(now - v["t"], 3) if v["t"] else None}
                               for k, v in latest.items()}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)


if __name__ == "__main__":
    for key, name in CAMERAS.items():
        print(f"capturing {key}: {name!r}")
        threading.Thread(target=capture_loop, args=(key,), daemon=True).start()
    print(f"serving http://{HOST}:{PORT}/stream and /mac/stream  (Ctrl-C to stop)")
    ThreadingHTTPServer((HOST, PORT), Handler).serve_forever()
