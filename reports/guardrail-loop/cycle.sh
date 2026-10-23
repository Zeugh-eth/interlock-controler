#!/bin/sh
# usage: cycle.sh <n> <outcome> <closes> <pick-json> <blocked-json> [notes]
printf '{"t":"%s","cycle":%s,"step":"cycle","outcome":"%s","jaw_closes":%s,"pick":%s,"blocked":%s,"notes":"%s"}\n' "$(date -u +%FT%TZ)" "$1" "$2" "$3" "$4" "$5" "${6:-}" >> "$(dirname "$0")/session-2026-10-23.jsonl"
