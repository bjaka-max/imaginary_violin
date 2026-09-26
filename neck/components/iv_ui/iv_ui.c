#include "iv_ui.h"
#include "iv_ui_band.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_check.h"

static const char *TAG = "iv.ui";

/* Модель меню задаёт свою ширину сама — она нужна ей, чтобы делить строку на
 * трети, — и разъехаться с шириной экрана не имеет права: тогда нажималось бы
 * не то, что подписано. */
_Static_assert(IV_MENU_WIDTH == BSP_LCD_H_RES,
               "ширина меню разошлась с шириной экрана");

extern const uint8_t iv_ui_font5x7[96 - 32][5];

/* Буфер полосы: в него рисуют, из него копируют в теневой кадр. Постоянный и
 * один: выделять его на каждую полосу значит дёргать кучу в такт обновлению
 * экрана, а класть на стек — держать там шестнадцать килобайт. */
static uint16_t *s_band;
static int       s_band_h;

/* Теневой кадр в PSRAM и границы того, что в нём изменилось.
 *
 * Он здесь не ради скорости, а ради правильности. Панель вертикально не
 * адресуется: у неё есть «пиши сначала» и «пиши дальше за предыдущей
 * передачей», и всё (подробности в bsp_display.c). Значит любой экран обязан
 * уходить на панель одним проходом сверху вниз — а рисовать при этом хочется
 * кусками и в любом порядке. Тень эти два требования и разводит: рисуем куда
 * угодно, а на панель отдаём подряд.
 *
 * Двести двадцать килобайт в PSRAM — цена, которую раньше платить не хотелось,
 * пока казалось, что частичные обновления возможны. Они невозможны. */
static uint16_t *s_fb;
static int       s_dirty_hi; /* ниже этой строки в тени ничего не менялось */

/* Во что обходится экран. Считается здесь, а показывается в логе: когда звук
 * начинает булькать, первый вопрос — не отъедает ли шину картинка, и отвечать
 * на него догадками дорого. */
static uint32_t s_present_us;   /* последняя отдача панели */
static uint32_t s_present_rows; /* сколько строк ушло */
static uint32_t s_present_n;    /* сколько раз отдавали */

/* Панель принимает старший байт пикселя первым, а в памяти ESP32 первым лежит
 * младший. Если на плате цвета окажутся перепутаны (красный выйдет синим),
 * правится ровно здесь. */
static inline uint16_t to_panel(uint16_t rgb565)
{
    return (uint16_t)((rgb565 >> 8) | (rgb565 << 8));
}

esp_err_t iv_ui_init(void)
{
    if (s_band == NULL) {
        s_band = heap_caps_malloc(BSP_LCD_H_RES * IV_UI_BAND_ROWS * sizeof(uint16_t),
                                  MALLOC_CAP_DMA);
        ESP_RETURN_ON_FALSE(s_band, ESP_ERR_NO_MEM, TAG, "буфер полосы");
    }
    if (s_fb == NULL) {
        s_fb = heap_caps_malloc((size_t)BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t),
                                MALLOC_CAP_SPIRAM);
        ESP_RETURN_ON_FALSE(s_fb, ESP_ERR_NO_MEM, TAG, "теневой кадр");
        memset(s_fb, 0, (size_t)BSP_LCD_H_RES * BSP_LCD_V_RES * sizeof(uint16_t));
    }
    s_dirty_hi = BSP_LCD_V_RES; /* что на панели сейчас — неизвестно */
    return ESP_OK;
}

void iv_ui_clear(uint16_t color)
{
    if (s_fb == NULL) {
        return;
    }
    const uint16_t c = to_panel(color);
    const size_t   n = (size_t)BSP_LCD_H_RES * BSP_LCD_V_RES;
    for (size_t i = 0; i < n; ++i) {
        s_fb[i] = c;
    }
    s_dirty_hi = BSP_LCD_V_RES;
}

void iv_ui_present(void)
{
    if (s_fb == NULL || s_band == NULL || s_dirty_hi <= 0) {
        return;
    }
    const int64_t t0 = esp_timer_get_time();
    /* Сверху вниз и подряд — иначе панель положит полосу не туда. Зато
     * закончить можно на последней изменившейся строке: ниже неё панель
     * хранит то, что было. */
    for (int y = 0; y < s_dirty_hi; y += IV_UI_BAND_ROWS) {
        int h = IV_UI_BAND_ROWS;
        if (y + h > s_dirty_hi) {
            h = s_dirty_hi - y;
        }
        memcpy(s_band, &s_fb[(size_t)y * BSP_LCD_H_RES],
               (size_t)h * BSP_LCD_H_RES * sizeof(uint16_t));
        bsp_display_blit(0, y, BSP_LCD_H_RES, y + h, s_band);
    }
    s_present_us   = (uint32_t)(esp_timer_get_time() - t0);
    s_present_rows = (uint32_t)s_dirty_hi;
    s_present_n++;
    s_dirty_hi = 0;
}

void iv_ui_present_stats(uint32_t *us, uint32_t *rows, uint32_t *count)
{
    if (us)    { *us = s_present_us; }
    if (rows)  { *rows = s_present_rows; }
    if (count) { *count = s_present_n; }
    s_present_n = 0;
}

/* ---- полоса --------------------------------------------------------------- */

void iv_band_begin(int h, uint16_t bg)
{
    if (s_band == NULL) {
        s_band_h = 0;
        return;
    }
    if (h < 0)                { h = 0; }
    if (h > IV_UI_BAND_ROWS)  { h = IV_UI_BAND_ROWS; }
    s_band_h = h;

    const uint16_t c = to_panel(bg);
    const size_t   n = (size_t)BSP_LCD_H_RES * (size_t)h;
    for (size_t i = 0; i < n; ++i) {
        s_band[i] = c;
    }
}

void iv_band_rect(int x, int y, int w, int h, uint16_t color)
{
    if (s_band == NULL) {
        return;
    }
    /* Отсечение по всем четырём сторонам сразу: рисующий код тогда может
     * считать в своих координатах и не проверять каждый раз, влезает ли. */
    if (x < 0)             { w += x; x = 0; }
    if (y < 0)             { h += y; y = 0; }
    if (x + w > BSP_LCD_H_RES) { w = BSP_LCD_H_RES - x; }
    if (y + h > s_band_h)      { h = s_band_h - y; }
    if (w <= 0 || h <= 0) {
        return;
    }

    const uint16_t c = to_panel(color);
    for (int row = y; row < y + h; ++row) {
        uint16_t *p = s_band + (size_t)row * BSP_LCD_H_RES + x;
        for (int i = 0; i < w; ++i) {
            p[i] = c;
        }
    }
}

static void draw_glyph(char ch, int x0, int y0, int scale, uint16_t panel_color)
{
    int idx = toupper((unsigned char)ch) - 0x20;
    if (idx < 0 || idx >= (int)(sizeof(iv_ui_font5x7) / sizeof(iv_ui_font5x7[0]))) {
        idx = '?' - 0x20;
    }

    for (int col = 0; col < 5; ++col) {
        const uint8_t bits = iv_ui_font5x7[idx][col];
        for (int bit = 0; bit < 7; ++bit) {
            if ((bits & (1u << bit)) == 0) {
                continue;
            }
            /* Одна точка шрифта — квадрат scale на scale. */
            for (int dy = 0; dy < scale; ++dy) {
                const int y = y0 + bit * scale + dy;
                if (y < 0 || y >= s_band_h) {
                    continue;
                }
                uint16_t *row = s_band + (size_t)y * BSP_LCD_H_RES;
                for (int dx = 0; dx < scale; ++dx) {
                    const int x = x0 + col * scale + dx;
                    if (x >= 0 && x < BSP_LCD_H_RES) {
                        row[x] = panel_color;
                    }
                }
            }
        }
    }
}

void iv_band_text(int x, int y, int scale, uint16_t color, const char *s)
{
    if (s_band == NULL || s == NULL || scale < 1) {
        return;
    }
    const uint16_t c = to_panel(color);
    for (int i = 0; s[i] != '\0'; ++i) {
        draw_glyph(s[i], x + i * 6 * scale, y, scale, c);
    }
}

int iv_band_text_w(int scale, const char *s)
{
    return s ? (int)strlen(s) * 6 * scale : 0;
}

/* Полоса уходит не на панель, а в теневой кадр: на панель всё отдаётся разом
 * в iv_ui_present(). */
void iv_band_flush(int screen_y)
{
    if (s_band == NULL || s_fb == NULL || s_band_h <= 0) {
        return;
    }
    int h = s_band_h;
    if (screen_y < 0 || screen_y >= BSP_LCD_V_RES) {
        return;
    }
    if (screen_y + h > BSP_LCD_V_RES) {
        h = BSP_LCD_V_RES - screen_y;
    }
    memcpy(&s_fb[(size_t)screen_y * BSP_LCD_H_RES], s_band,
           (size_t)h * BSP_LCD_H_RES * sizeof(uint16_t));
    if (screen_y + h > s_dirty_hi) {
        s_dirty_hi = screen_y + h;
    }
}

/* ---- текстовая строка ------------------------------------------------------ */

void iv_ui_row(int row, uint16_t fg, uint16_t bg, const char *fmt, ...)
{
    if (row < 0 || row >= IV_UI_ROWS) {
        return;
    }

    char text[IV_UI_COLS + 1];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);

    iv_band_begin(IV_UI_ROW_H, bg);
    iv_band_text(0, 0, IV_UI_SCALE, fg, text);
    iv_band_flush(row * IV_UI_ROW_H);
}

/* ---- выключение ------------------------------------------------------------ */

void iv_ui_shutdown_screen(void)
{
    iv_ui_clear(IV_UI_BLACK);

    /* Ровно посередине экрана: выключение — единственный момент, когда
     * пользователь смотрит на экран и ждёт от него ответа. */
    static const char *TEXT = "OFF";
    const int w = iv_band_text_w(4, TEXT);

    iv_band_begin(IV_UI_BAND_ROWS, IV_UI_BLACK);
    iv_band_text((BSP_LCD_H_RES - w) / 2, (IV_UI_BAND_ROWS - 8 * 4) / 2,
                 4, IV_UI_ORANGE, TEXT);
    iv_band_flush(BSP_LCD_V_RES / 2 - IV_UI_BAND_ROWS / 2);
    iv_ui_present();
}
