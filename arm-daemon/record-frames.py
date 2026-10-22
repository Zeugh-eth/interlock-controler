"""Record cam-server frames with timestamps while teleop runs (joints are not readable while
LeRobot teleop owns the bus; for joints+frames in one dataset use record.command / lerobot-record).
Writes recordings/<stamp>/<cam>/<n>.jpg and frames.jsonl ({t, cam, file, age_s}). Ctrl-C/SIGTERM stops."""
import json, signal, sys, time, urllib.request
from pathlib import Path

CAMS = sys.argv[1].split(",") if len(sys.argv) > 1 else ["gripper", "mac"]
HZ = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
BASE = "http://127.0.0.1:8765"
out = Path(__file__).resolve().parent / "recordings" / time.strftime("%Y%m%d-%H%M%S")
for c in CAMS:
    (out / c).mkdir(parents=True, exist_ok=True)
op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
run = True
signal.signal(signal.SIGTERM, lambda *_: globals().__setitem__("run", False))
print("recording to", out, "cams", CAMS, "hz", HZ, flush=True)
n = {c: 0 for c in CAMS}
with open(out / "frames.jsonl", "a") as idx:
    try:
        while run:
            t0 = time.time()
            for c in CAMS:
                try:
                    r = op.open(f"{BASE}/{c}/frame.jpg", timeout=1)
                    jpg = r.read()
                    f = out / c / f"{n[c]:06d}.jpg"
                    f.write_bytes(jpg)
                    idx.write(json.dumps({"t": round(t0, 3), "cam": c, "file": str(f.relative_to(out))}) + "\n")
                    n[c] += 1
                except Exception:
                    pass
            idx.flush()
            time.sleep(max(0, 1 / HZ - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
print("stopped", dict(n), flush=True)
