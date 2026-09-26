#include "bsp_neck.h"
#include "bsp_internal.h"
#include "iv_touch_track.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "bsp.touch";

/* Смещение первой точки в ответе и шаг между точками.
 * Первая точка взята из демо производителя; шаг 6 байт в демо не используется —
 * оно читает только одну точку. Проверено на живой плате: два пальца дают две
 * различающиеся точки с правдоподобными координатами. */
#define TOUCH_FIRST_OFFSET 2
#define TOUCH_POINT_STRIDE 6

/* Длина ответа: заголовок и BSP_TOUCH_MAX_POINTS точек подряд. */
#define TOUCH_FRAME_LEN (TOUCH_FIRST_OFFSET + TOUCH_POINT_STRIDE * BSP_TOUCH_MAX_POINTS)

_Static_assert(TOUCH_FRAME_LEN <= 255, "длина ответа не влезает в байт запроса");

/* Тач живёт в том же контроллере, что и дисплей (AXS15231B), но читается
 * напрямую по I2C своей командой: пишем 11-байтовый запрос, читаем ответ.
 * Последовательность взята из демо производителя.
 *
 * Байт 7 — сколько байт контроллер отдаст в ответ; в демо там 0x08, то есть
 * заголовок и ровно одна точка. Нам нужны обе, поэтому длина считается из
 * BSP_TOUCH_MAX_POINTS. Читать больше, чем просишь, бессмысленно: за концом
 * ответа идут не координаты, а то, что шина доклокала. */
static const uint8_t READ_CMD[11] = {
    0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, TOUCH_FRAME_LEN, 0x00, 0x00, 0x00
};

_Static_assert(BSP_TOUCH_MAX_POINTS == IV_TRACK_MAX,
               "трекер и драйвер считают точки по-разному");

static i2c_master_dev_handle_t s_dev = NULL;

/* Разведение призраков — это состояние, и оно одно на драйвер. Читают же тач
 * две задачи: управление на 200 Гц всегда и рисование меню на своей частоте,
 * пока открыто меню. Без замка они рвали бы и это состояние, и ленивую
 * инициализацию устройства. Сама шина замок имеет, а эти двое — нет. */
static iv_track_t        s_track;
static SemaphoreHandle_t s_lock = NULL;

static esp_err_t touch_lazy_init(void)
{
    if (s_dev) {
        return ESP_OK;
    }
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BSP_TOUCH_ADDR,
        .scl_speed_hz    = 300000,
    };
    iv_track_init(&s_track);
    return i2c_master_bus_add_device(bsp_i2c_touch, &cfg, &s_dev);
}

/* Замок создаётся при первом обращении из задачи, которая успела раньше;
 * до старта задач тач никто не читает, так что гонки на самом создании нет —
 * её закрывает bsp_touch_init(), вызываемый с одной задачи при подъёме. */
esp_err_t bsp_touch_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "замок тача");
    }
    iv_track_init(&s_track);
    return ESP_OK;
}

static esp_err_t touch_read_locked(bsp_touch_state_t *out)
{
    ESP_RETURN_ON_ERROR(touch_lazy_init(), TAG, "устройство тача");

    uint8_t buf[TOUCH_FRAME_LEN] = {0};
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(s_dev, READ_CMD, sizeof(READ_CMD),
                                                    buf, sizeof(buf), pdMS_TO_TICKS(50)),
                        TAG, "чтение тача");

    memset(out, 0, sizeof(*out));

    uint8_t count = buf[1];
    if (count == 0 || count > BSP_TOUCH_MAX_POINTS) {
        /* Ноль — касаний нет. Значение вне диапазона означает мусор в ответе:
         * молча считаем, что касаний нет, чтобы не сорвать ноту случайным сбоем. */
        return ESP_OK;
    }

    for (uint8_t i = 0; i < count; ++i) {
        /* Выход за буфер невозможен: count уже прижат к BSP_TOUCH_MAX_POINTS,
         * а буфер ровно на столько точек и посчитан. */
        const uint8_t *p = &buf[TOUCH_FIRST_OFFSET + i * TOUCH_POINT_STRIDE];
        uint16_t x = (uint16_t)((p[0] & 0x0f) << 8) | p[1];
        uint16_t y = (uint16_t)((p[2] & 0x0f) << 8) | p[3];

        /* Контроллер отдаёт координаты в своей ориентации; приводим к экрану.
         * Проверено на плате: получается портретная система с началом в левом
         * верхнем углу — x поперёк грифа (0..171), y вдоль него (0..639). */
        if (x > BSP_LCD_V_RES) x = BSP_LCD_V_RES;
        if (y > BSP_LCD_H_RES) y = BSP_LCD_H_RES;

        out->points[out->count].x = y;
        out->points[out->count].y = (uint16_t)(BSP_LCD_V_RES - x);
        out->count++;
    }

    /* Развести призрачные пары. Делается здесь, а не в раскладке: призраки —
     * это свойство сенсора, и выше драйвера про них знать не должны.
     *
     * Работает по экранным координатам, уже развёрнутым. Разворот переставляет
     * оси, но пары от этого не путаются: множество x и множество y просто
     * меняются местами целиком, а сцепление между ними то же. */
    iv_track_frame_t f = { .count = out->count };
    for (uint8_t i = 0; i < out->count; ++i) {
        f.p[i].x = (int16_t)out->points[i].x;
        f.p[i].y = (int16_t)out->points[i].y;
    }
    iv_track_resolve(&s_track, &f);
    for (uint8_t i = 0; i < out->count; ++i) {
        out->points[i].x = (uint16_t)f.p[i].x;
        out->points[i].y = (uint16_t)f.p[i].y;
    }
    return ESP_OK;
}

esp_err_t bsp_touch_read(bsp_touch_state_t *out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out");
    ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_INVALID_STATE, TAG, "тач не поднят");

    xSemaphoreTake(s_lock, portMAX_DELAY);
    const esp_err_t err = touch_read_locked(out);
    xSemaphoreGive(s_lock);
    if (err != ESP_OK) {
        memset(out, 0, sizeof(*out)); /* ошибка — это «касаний нет», не мусор */
    }
    return err;
}
