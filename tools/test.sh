#!/bin/bash
# Только хостовые тесты — быстрый цикл при работе над звуком.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Вывод сборки прячется, пока она проходит, и показывается целиком, если нет.
# Просто перенаправить в /dev/null нельзя: ninja пишет ошибки компиляции в
# stdout, и при set -e скрипт молча падал бы с пустым выводом.
quietly() {
    local out
    if ! out=$("$@" 2>&1); then
        echo "$out" >&2
        return 1
    fi
}

quietly cmake -S host -B build/host -G Ninja
quietly cmake --build build/host
ctest --test-dir build/host --output-on-failure
