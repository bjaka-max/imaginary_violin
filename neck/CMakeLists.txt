cmake_minimum_required(VERSION 3.16)

# Общие компоненты берутся из common/ — один источник правды для протокола.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../common")

# Версия задаётся явно, до project(). Иначе ESP-IDF полезет за ней в git через
# git_describe, и сборка станет зависеть от состояния репозитория — свежий
# `git init` без единого коммита её ломает.
set(PROJECT_VER "0.1.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(iv_neck)
