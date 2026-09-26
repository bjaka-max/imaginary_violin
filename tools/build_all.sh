#!/bin/bash
# Собирает всё: хостовые тесты и обе прошивки.
#
# Тесты идут первыми намеренно: они проверяют общий код за секунды, а сборка
# прошивок занимает минуты. Ломать цикл обратной связи не стоит.
set -euo pipefail

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

echo "=== хостовые тесты ==="
cmake -S host -B build/host -G Ninja >/dev/null
cmake --build build/host
ctest --test-dir build/host --output-on-failure

echo
echo "=== прошивки ==="
for proj in neck bow; do
    echo "--- $proj ---"
    pio run -d "$proj"
done

echo
echo "Готово. Прошить: tools/flash.sh neck /dev/ttyACM0"
