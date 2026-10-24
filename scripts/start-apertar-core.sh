#!/bin/bash
set -euo pipefail

CORE_BIN="${APERTAR_CORE_BIN:-/home/pi/ApertarCore-build/apertar-core}"
MEDIA_ROOT="${APERTAR_MEDIA_ROOT:-/media/RAW}"
SOCKET_PATH="${APERTAR_SOCKET_PATH:-/tmp/apertar-core.sock}"

if [[ ! -x "$CORE_BIN" ]]; then
    echo "ApertarCore binary not found or not executable: $CORE_BIN" >&2
    exit 1
fi

# Saved custom selections take precedence. Otherwise Core selects the sensor's
# JSON in defaults-pisp (Pi 5) or defaults-vc4 (Pi 3/4) under
# /home/pi/Tuning-Files before libcamera starts. Core validates custom targets too.
if [[ -n "${APERTAR_TUNING_FILE:-}" ]]; then
    export LIBCAMERA_RPI_TUNING_FILE="$APERTAR_TUNING_FILE"
fi

mkdir -p "$(dirname "$SOCKET_PATH")"
exec "$CORE_BIN" --socket "$SOCKET_PATH" --media "$MEDIA_ROOT"
