"""Leader-arm teleop that records state, action and the gripper camera together.

Same loop as lerobot-teleoperate (leader -> follower at FPS with max_relative_target), plus one
row per tick in recordings/<stamp>-teleop/frames.jsonl:
  {"t": unix, "i": tick, "state": {joint: deg|pct}, "action": {joint: deg|pct}, "image": "gripper/000123.jpg"}
"image" is written only when the gripper frame changed (cam-server :8765 serves ~30 fps); rows
without it reuse the previous image, as tools/smolvla/convert_runs.py already expects.
Usage: uv run python teleop-record.py "<task text>" [--fps 30] [--max-step 15] [--episode-s 0]
Ctrl-C stops (follower torque is released like plain teleop: rest the arm first)."""
import argparse, json, os, signal, sys, time, urllib.request
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

FOLLOWER = os.environ.get("ARM_FOLLOWER_PORT", "/dev/cu.usbmodem5B790812561")
LEADER = os.environ.get("ARM_LEADER_PORT", "/dev/cu.usbmodem5B901010331")
CAM = "http://127.0.0.1:8765/gripper/frame.jpg"

ap = argparse.ArgumentParser()
ap.add_argument("task")
ap.add_argument("--fps", type=int, default=30)
ap.add_argument("--max-step", type=float, default=15.0)
ap.add_argument("--episode-s", type=float, default=0, help="0 = one episode until Ctrl-C")
a = ap.parse_args()

out = Path(__file__).resolve().parent / "recordings" / (time.strftime("%Y%m%d-%H%M%S") + "-teleop")
(out / "gripper").mkdir(parents=True)
(out / "meta.json").write_text(json.dumps({"task": a.task, "fps": a.fps, "max_step": a.max_step, "t0": time.time()}))
op = urllib.request.build_opener(urllib.request.ProxyHandler({}))

robot = SO101Follower(SO101FollowerConfig(port=FOLLOWER, id="hackaiwoo_follower", max_relative_target=a.max_step))
leader = SO101Leader(SO101LeaderConfig(port=LEADER, id="hackaiwoo_leader"))
robot.connect(); leader.connect()
print("recording to", out, "| task:", a.task, "| Ctrl-C to stop", flush=True)
run = True
signal.signal(signal.SIGTERM, lambda *_: globals().__setitem__("run", False))
last_jpg, n_img, i, t_ep = None, 0, 0, time.time()
try:
    with open(out / "frames.jsonl", "a") as f:
        while run:
            t0 = time.perf_counter()
            obs = robot.get_observation()
            action = leader.get_action()
            robot.send_action(action)
            row = {"t": round(time.time(), 4), "i": i,
                   "state": {k.removesuffix(".pos"): round(float(v), 2) for k, v in obs.items() if k.endswith(".pos")},
                   "action": {k.removesuffix(".pos"): round(float(v), 2) for k, v in action.items() if k.endswith(".pos")}}
            try:
                jpg = op.open(CAM, timeout=0.2).read()
                if jpg != last_jpg:
                    p = out / "gripper" / f"{n_img:06d}.jpg"; p.write_bytes(jpg)
                    row["image"] = f"gripper/{p.name}"; last_jpg = jpg; n_img += 1
            except Exception:
                pass
            f.write(json.dumps(row) + "\n")
            i += 1
            if i % (a.fps * 5) == 0:
                f.flush(); print(f"tick {i}  images {n_img}  elapsed {time.time()-t_ep:.0f}s", flush=True)
            if a.episode_s and time.time() - t_ep > a.episode_s:
                break
            time.sleep(max(0.0, 1.0 / a.fps - (time.perf_counter() - t0)))
except KeyboardInterrupt:
    pass
finally:
    print(f"stopped: {i} ticks, {n_img} images ->", out, flush=True)
    leader.disconnect(); robot.disconnect()
