# Хостовая сборка чистого C-кода: iv_dsp, iv_protocol, iv_synth.
#
# Смысл: настройка звучания — самая длинная часть проекта, и цикл
# «поправил -> перепрошил -> послушал» её убивает. Весь звуковой код
# намеренно не зависит от ESP-IDF, поэтому собирается и проверяется здесь,
# на компьютере, за секунды.
cmake_minimum_required(VERSION 3.16)
project(iv_host C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Werror)

set(IV_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

add_library(iv_core STATIC
    ${IV_ROOT}/common/iv_dsp/iv_dsp.c
    ${IV_ROOT}/common/iv_protocol/iv_protocol.c
    ${IV_ROOT}/common/iv_link/iv_link_stats.c
    ${IV_ROOT}/common/iv_params/iv_params.c
    ${IV_ROOT}/common/iv_motion/iv_motion.c
    ${IV_ROOT}/common/iv_power/iv_power.c
    ${IV_ROOT}/common/iv_settings/iv_settings.c
    ${IV_ROOT}/neck/components/iv_synth/iv_synth.c
    ${IV_ROOT}/neck/components/iv_synth/iv_string.c
    ${IV_ROOT}/neck/components/iv_synth/iv_body.c
    ${IV_ROOT}/neck/components/iv_control/iv_control.c
    ${IV_ROOT}/neck/components/iv_menu/iv_menu.c
    ${IV_ROOT}/neck/components/iv_touch/iv_touch_track.c
)
target_include_directories(iv_core PUBLIC
    ${IV_ROOT}/common/iv_dsp/include
    ${IV_ROOT}/common/iv_protocol/include
    ${IV_ROOT}/common/iv_link/include
    ${IV_ROOT}/common/iv_params/include
    ${IV_ROOT}/common/iv_motion/include
    ${IV_ROOT}/common/iv_power/include
    ${IV_ROOT}/common/iv_settings/include
    ${IV_ROOT}/neck/components/iv_synth/include
    ${IV_ROOT}/neck/components/iv_control/include
    ${IV_ROOT}/neck/components/iv_menu/include
    ${IV_ROOT}/neck/components/iv_touch/include
)
target_link_libraries(iv_core PUBLIC m)

# Анализ сигнала — только для компьютера: им проверяют звук в тестах и в
# tools/render. В прошивку он не идёт.
add_library(iv_analysis STATIC analysis/iv_analysis.c)
target_include_directories(iv_analysis PUBLIC ${CMAKE_CURRENT_LIST_DIR}/analysis)
target_link_libraries(iv_analysis PUBLIC m)

# Оффлайн-рендер жеста в WAV: тот же синтезатор, что в прошивке, но слушать
# результат можно через секунду после правки, а не через перепрошивку.
add_executable(iv_render ${IV_ROOT}/tools/render/render.c)
target_link_libraries(iv_render PRIVATE iv_core iv_analysis)

enable_testing()

foreach(t test_dsp test_protocol test_link test_synth test_string test_motion test_control
           test_settings test_menu test_power test_touch_track)
    add_executable(${t} tests/${t}.c)
    target_link_libraries(${t} PRIVATE iv_core iv_analysis)
    add_test(NAME ${t} COMMAND ${t})
endforeach()

# Снимок параметров проверяется на двух потоках: рвущийся снимок на одном
# потоке не воспроизводится в принципе, а именно он тут и опасен.
find_package(Threads REQUIRED)
add_executable(test_params tests/test_params.c)
target_link_libraries(test_params PRIVATE iv_core Threads::Threads)
add_test(NAME test_params COMMAND test_params)
