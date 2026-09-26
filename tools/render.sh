#!/bin/bash
# Рендер жеста в WAV тем же синтезатором, что стоит в прошивке.
#
# Смысл — слушать правку через секунду, а не через перепрошивку. Сборка идёт
# туда же, куда и тесты, поэтому второй запуск почти мгновенный.
#
#   tools/render.sh --help
#   tools/render.sh --gesture SCALE --out /tmp/scale.wav --report
#   tools/render.sh --file мой_жест.txt --out /tmp/жест.wav
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

cmake -S host -B build/host -G Ninja >/dev/null
cmake --build build/host --target iv_render >/dev/null

exec ./build/host/iv_render "$@"
