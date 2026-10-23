#!/bin/zsh
# Double-click (or `open`) to start the gripper-camera server in Terminal.app.
cd "$(dirname "$0")/../tools/arm-lerobot" && uv run python ../../scripts/cam-server.py
