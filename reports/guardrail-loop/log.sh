#!/bin/sh
# usage: log.sh <cycle> <step> <outcome> <json-detail>
printf '{"t":"%s","cycle":%s,"step":"%s","outcome":"%s","detail":%s}\n' "$(date -u +%FT%TZ)" "$1" "$2" "$3" "${4:-null}" >> "$(dirname "$0")/session-2026-10-23.jsonl"
