#!/bin/zsh
# Agent MCP over HTTP for agents on other machines (Hermes on the Pi). Double-click (Terminal) to start,
# Ctrl-C to stop. Needs arm-daemon running; does not touch the local stdio MCP or the bus.
cd "$(dirname "$0")/.."
exec uv --directory tools/arm-lerobot run python -B ../../arm-daemon/mcp_server.py --http "$@"
