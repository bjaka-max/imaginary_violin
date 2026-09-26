#include "bsp_neck.h"
#include "bsp_internal.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "bsp.button";

#define DEBOUNCE_SAMPLES (2)                                       /* 40 мс */
#define LONG_SAMPLES     (BSP_BUTTON_LONG_MS / BSP_BUTTON_POLL_MS) /* 75 отсчётов */

typedef struct {
    gpio_num_t pin;
    uint8_t    raw_prev;     /* предыдущий сырой отсчёт, для устранения дребезга */
    uint8_t    same_count;   /* сколько отсчётов подряд уровень не менялся */
    uint8_t    stable;       /* устоявшееся состояние: 1 — нажата */
    uint16_t   held;         /* сколько отсчётов удерживается */
    bool       long_fired;   /* долгое нажатие уже отдано, не повторять */
    bool       startup_hold; /* нажата уже на первом опросе */
} button_t;

static button_t s_buttons[BSP_BTN_COUNT];

esp_err_t bsp_button_init(void)
{
    s_buttons[BSP_BTN_PWR].pin  = BSP_BTN_PWR_GPIO;
    s_buttons[BSP_BTN_BOOT].pin = BSP_BTN_BOOT_GPIO;

    const gpio_config_t cfg = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BSP_BTN_PWR_GPIO) | (1ULL << BSP_BTN_BOOT_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "настройка выводов кнопок");

    /* Начальное состояние читаем сразу: если кнопка уже нажата, это то самое
     * удержание, которым включили питание, и события по ней надо подавить
     * до первого отпускания. */
    for (int i = 0; i < BSP_BTN_COUNT; ++i) {
        button_t *b = &s_buttons[i];
        b->stable = b->raw_prev = (gpio_get_level(b->pin) == 0) ? 1 : 0;
        b->startup_hold = (b->stable == 1);
        b->same_count = DEBOUNCE_SAMPLES;
        b->held = 0;
        b->long_fired = false;
    }

    if (s_buttons[BSP_BTN_PWR].startup_hold) {
        ESP_LOGI(TAG, "кнопка питания удерживается со старта, ждём отпускания");
    }
    return ESP_OK;
}

bsp_button_event_t bsp_button_poll(bsp_button_id_t id)
{
    if (id >= BSP_BTN_COUNT) {
        return BSP_BTN_EVENT_NONE;
    }
    button_t *b = &s_buttons[id];

    const uint8_t raw = (gpio_get_level(b->pin) == 0) ? 1 : 0;

    if (raw == b->raw_prev) {
        if (b->same_count < DEBOUNCE_SAMPLES) {
            b->same_count++;
        }
    } else {
        b->same_count = 0;
        b->raw_prev = raw;
    }

    /* Пока уровень не устоялся, состояние не меняем. */
    if (b->same_count < DEBOUNCE_SAMPLES) {
        return BSP_BTN_EVENT_NONE;
    }

    const uint8_t was = b->stable;
    b->stable = raw;

    /* Стартовое удержание: никаких событий, пока не отпустят. */
    if (b->startup_hold) {
        if (b->stable == 0) {
            b->startup_hold = false;
            b->held = 0;
            b->long_fired = false;
            ESP_LOGI(TAG, "кнопка отпущена, обработка нажатий включена");
        }
        return BSP_BTN_EVENT_NONE;
    }

    if (b->stable) {
        b->held++;
        /* Долгое нажатие отдаём по достижении порога, не по отпусканию:
         * так пользователь получает реакцию, пока держит кнопку. */
        if (!b->long_fired && b->held >= LONG_SAMPLES) {
            b->long_fired = true;
            return BSP_BTN_EVENT_LONG;
        }
        return BSP_BTN_EVENT_NONE;
    }

    /* Отпускание. */
    const bool was_short = (was == 1) && !b->long_fired && b->held > 0;
    b->held = 0;
    b->long_fired = false;
    return was_short ? BSP_BTN_EVENT_SHORT : BSP_BTN_EVENT_NONE;
}
