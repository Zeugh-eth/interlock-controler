#!/bin/zsh
# arm-daemon: the one process that owns the arm's serial bus. Double-click (Terminal) to start.
# Stop with Ctrl-C (torque stays on). Pass --fake for a dry run without the arm.
cd "$(dirname "$0")/.."
exec uv --directory tools/arm-lerobot run python -B ../../arm-daemon/daemon.py "$@"
