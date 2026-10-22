"""Fake cam-server for tests: /health JSON and /<cam>/frame.jpg. down -> 503; frame_age settable."""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer


class FakeCam:
    def __init__(self):
        self.down = False
        self.frame_age = 0.03
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if outer.down:
                    self.send_error(503, "no frame yet")
                    return
                if self.path == "/health":
                    body = json.dumps({"gripper": {"camera": "USB HD Camera", "mode": "x", "frame_age_s": outer.frame_age},
                                       "mac": {"camera": "FaceTime HD Camera", "mode": "x", "frame_age_s": 0.02}}).encode()
                    self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(body)
                elif self.path.endswith("/frame.jpg"):
                    self.send_response(200); self.send_header("Content-Type", "image/jpeg"); self.end_headers(); self.wfile.write(b"\xff\xd8fake")
                else:
                    self.send_error(404)

        self.srv = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.srv.server_port}"
        self.t = threading.Thread(target=self.srv.serve_forever, daemon=True)
        self.t.start()

    def close(self):
        self.srv.shutdown()
