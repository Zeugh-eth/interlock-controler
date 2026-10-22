# SO-101 leader → follower teleop (working 23 Oct 2026)

- Leader port `/dev/cu.usbmodem5B901010331` (5 V adapter, 4.9 V readout is normal). Follower `/dev/cu.usbmodem5B790812561`.
- Leader calibration: `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/hackaiwoo_leader.json` (id `hackaiwoo_leader`).

## Run
Nothing else may hold the follower port (arm-daemon :8800, arm-agent runtime :8780, dashboard :8770). Then:

    yes '' | ./scripts/teleop.command

Ctrl-C stops it and releases follower torque — rest the follower first.

## If a joint is offset
Stop teleop, put BOTH arms in the same pose (folded rest, pan forward, grippers closed), then from `tools/arm-lerobot`:

    uv run python ../../scripts/leader-align.py --apply

Restart teleop. Full recalibration (`scripts/leader-calibrate.command`) only if a joint's travel is wrong, then check with `scripts/leader-check-calib.py`.
