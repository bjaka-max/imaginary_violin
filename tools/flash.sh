#!/bin/bash
# Прошивка и монитор: tools/flash.sh <neck|bow> [порт]
# Без порта PlatformIO ищет плату сам.
set -euo pipefail

PROJ="${1:?укажи neck или bow}"
PORT="${2:-}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# PlatformIO ставится в свой venv и не всегда попадает в PATH.
if ! command -v pio >/dev/null 2>&1; then
    if [ -x "$HOME/.platformio/penv/bin/pio" ]; then
        PATH="$HOME/.platformio/penv/bin:$PATH"; export PATH
    else
        echo "Не найден pio. Установка: https://docs.platformio.org/en/latest/core/installation/" >&2
        exit 1
    fi
fi

if [ -n "$PORT" ]; then
    pio run -d "$PROJ" -t upload --upload-port "$PORT"
    pio device monitor -p "$PORT" -b 115200
else
    pio run -d "$PROJ" -t upload
    pio device monitor -b 115200
fi
