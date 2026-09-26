#!/bin/bash
# Fails if the built app image uses more than 92% of the smallest app
# partition (ota_0 / ota_1, both 0x110000 bytes per partitions.csv).
#
# Usage: check_app_size.sh <path-to-app-bin>
set -e

BIN_PATH="$1"
if [ -z "$BIN_PATH" ] || [ ! -f "$BIN_PATH" ]; then
    echo "ERROR: app binary not found at '${BIN_PATH}'" >&2
    exit 1
fi

PARTITION_SIZE=$((0x110000))
MAX_PERCENT=92

APP_SIZE=$(stat -c%s "$BIN_PATH")
PERCENT=$(( APP_SIZE * 100 / PARTITION_SIZE ))
FREE=$(( PARTITION_SIZE - APP_SIZE ))

echo "app size:       ${APP_SIZE} bytes"
echo "partition size: ${PARTITION_SIZE} bytes"
echo "usage:          ${PERCENT}% (free: ${FREE} bytes)"

if [ "$PERCENT" -gt "$MAX_PERCENT" ]; then
    echo "ERROR: app uses ${PERCENT}% of the app partition, exceeding the ${MAX_PERCENT}% gate." >&2
    exit 1
fi
