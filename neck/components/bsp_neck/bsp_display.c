#include "bsp_neck.h"
#include "bsp_internal.h"

#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_axs15231b.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "bsp.lcd";

#define LCD_HOST        SPI3_HOST
#define LCD_PCLK_HZ     (40 * 1000 * 1000)

/* Полоса на одну передачу. Полный кадр 172x640x2 = 220 КБ разом по QSPI
 * не гоняем: и буфер великоват, и очередь SPI держать проще полосами. */
#define BAND_ROWS       64
#define BAND_PIXELS     (BSP_LCD_H_RES * BAND_ROWS)
#define BAND_BYTES      (BAND_PIXELS * 2)

static esp_lcd_panel_handle_t s_panel  = NULL;
static SemaphoreHandle_t      s_flush_done = NULL;
static uint16_t              *s_band   = NULL;
static uint32_t               s_frame_us = 0;

/* Панель зовёт это из прерывания, когда передача цвета закончена. */
static bool on_trans_done(esp_lcd_panel_io_handle_t io,
                          esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_flush_done, &woken);
    return woken == pdTRUE;
}

/* Единственная команда инициализации у этой панели: выход из сна и включение
 * вывода. Всё остальное контроллер берёт из своей прошивки. */
static const axs15231b_lcd_init_cmd_t s_init_cmds[] = {
    {0x11, (uint8_t []){0x00}, 0, 100}, /* SLPOUT */
    {0x29, (uint8_t []){0x00}, 0, 100}, /* DISPON */
};

esp_err_t bsp_display_init(void)
{
    s_flush_done = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_flush_done, ESP_ERR_NO_MEM, TAG, "семафор");

    s_band = heap_caps_malloc(BAND_BYTES, MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(s_band, ESP_ERR_NO_MEM, TAG, "буфер полосы");

    const spi_bus_config_t bus = {
        .sclk_io_num    = BSP_LCD_PCLK,
        .data0_io_num   = BSP_LCD_D0,
        .data1_io_num   = BSP_LCD_D1,
        .data2_io_num   = BSP_LCD_D2,
        .data3_io_num   = BSP_LCD_D3,
        .max_transfer_sz = BAND_BYTES,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO),
                        TAG, "шина QSPI");

    esp_lcd_panel_io_handle_t io = NULL;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num       = BSP_LCD_CS,
        .dc_gpio_num       = -1,          /* в QSPI команда идёт в тех же линиях */
        .spi_mode          = 3,
        .pclk_hz           = LCD_PCLK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = on_trans_done,
        .lcd_cmd_bits      = 32,
        .lcd_param_bits    = 8,
        .flags.quad_mode   = true,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(LCD_HOST, &io_cfg, &io),
                        TAG, "panel io");

    axs15231b_vendor_config_t vendor = {
        .init_cmds      = s_init_cmds,
        .init_cmds_size = sizeof(s_init_cmds) / sizeof(s_init_cmds[0]),
        .flags.use_qspi_interface = 1,
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,             /* сброс идёт через расширитель */
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config  = &vendor,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_axs15231b(io, &panel_cfg, &s_panel),
                        TAG, "панель AXS15231B");

    bsp_lcd_reset();
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "инициализация панели");

    ESP_LOGI(TAG, "дисплей %dx%d поднят, QSPI %d МГц",
             BSP_LCD_H_RES, BSP_LCD_V_RES, LCD_PCLK_HZ / 1000000);
    return ESP_OK;
}

/* Одна передача полосы с ожиданием завершения: без ожидания нельзя
 * переиспользовать буфер, а таймер кадра показывал бы время постановки в
 * очередь, а не отрисовки.
 *
 * ГЛАВНОЕ ПРО ЭТУ ПАНЕЛЬ, и это её свойство, а не наше решение.
 *
 * Драйвер axs15231b в QSPI-режиме не задаёт строку вывода: шлёт только CASET
 * (колонки), а RASET не шлёт вовсе, и пишет либо RAMWR, если y_start ноль,
 * либо RAMWRC, «продолжить», если не ноль. То есть вертикальный адрес есть
 * ровно один — «сначала», — а всё остальное идёт подряд за предыдущей
 * передачей.
 *
 * Задать окно самим пробовалось и не работает: панель RASET игнорирует. С
 * RAMWR после него каждая полоса ложилась в начало экрана — при чёрном фоне
 * это выглядело как «рисуется только верхние восемьдесят точек». С RAMWRC без
 * него полосы шли подряд, и картинка уезжала вниз с каждой перерисовкой.
 *
 * Отсюда единственный работающий порядок: **кадр идёт с нулевой строки и
 * подряд**. Пропустить полосу нельзя. Зато закончить можно раньше — строки
 * ниже последней записанной сохраняют то, что на них было, и на этом держится
 * вся экономия: перерисовать до середины экрана вдвое дешевле, чем весь. */
static esp_err_t push_band(int y1, int y2)
{
    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(s_panel, 0, y1, BSP_LCD_H_RES, y2,
                                                  s_band),
                        TAG, "draw_bitmap");
    xSemaphoreTake(s_flush_done, pdMS_TO_TICKS(1000));
    return ESP_OK;
}

esp_err_t bsp_display_fill(uint16_t color)
{
    ESP_RETURN_ON_FALSE(s_panel, ESP_ERR_INVALID_STATE, TAG, "дисплей не поднят");

    for (int i = 0; i < BAND_PIXELS; ++i) {
        s_band[i] = color;
    }

    /* Оборвавшаяся заливка оставляет полэкрана мусором, и по такому экрану
     * уже не понять, что случилось. Поэтому неудачная полоса не прекращает
     * заливку: доводим до низа и жалуемся один раз. */
    esp_err_t first_err = ESP_OK;
    const int64_t t0 = esp_timer_get_time();
    for (int y = 0; y < BSP_LCD_V_RES; y += BAND_ROWS) {
        int y2 = y + BAND_ROWS;
        if (y2 > BSP_LCD_V_RES) {
            y2 = BSP_LCD_V_RES;
        }
        const esp_err_t err = push_band(y, y2);
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
            ESP_LOGE(TAG, "заливка: полоса с %d не прошла (%s), продолжаю",
                     y, esp_err_to_name(err));
        }
    }
    s_frame_us = (uint32_t)(esp_timer_get_time() - t0);
    return first_err;
}

esp_err_t bsp_display_blit(int x1, int y1, int x2, int y2, const uint16_t *pixels)
{
    ESP_RETURN_ON_FALSE(s_panel, ESP_ERR_INVALID_STATE, TAG, "дисплей не поднят");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(s_panel, x1, y1, x2, y2, pixels),
                        TAG, "draw_bitmap");
    xSemaphoreTake(s_flush_done, pdMS_TO_TICKS(1000));
    return ESP_OK;
}

uint32_t bsp_display_last_frame_us(void)
{
    return s_frame_us;
}
