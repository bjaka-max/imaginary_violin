/* Экран меню.
 *
 * Полноэкранная перерисовка тут разрешена — в меню не играют, — но всё равно
 * не делается каждый кадр: экран рисуется из задачи, которая крутится 25 раз
 * в секунду, и восемь строк по 16 КБ на каждый проход забили бы шину целиком.
 * Поэтому строка перерисовывается, только когда изменилось её значение или
 * подсветка.
 *
 * Что где нажимается, решает не этот файл, а модель в iv_menu: здесь только
 * рисуется то, что она обещает. Стрелки «<» и «>» рисуются ровно у тех
 * пунктов, у которых левая и правая трети строки действительно что-то
 * меняют, — иначе экран врал бы про управление. */
#include "iv_ui.h"
#include "iv_ui_band.h"

#include <stdio.h>
#include <string.h>

#define MENU_TITLE   "SETTINGS"
#define MENU_HINT    "HOLD BOOT=PLAY"

/* Подсветка выбранной строки. Тёмно-серая, а не яркая: это указатель на
 * последний тронутый пункт, а не кнопка, которую надо искать. */
#define SEL_BG IV_UI_DARK

static struct {
    bool valid;
    int  sel;
    char value[IV_MENU_COUNT][12];
} s_prev;

static void draw_row(int item, const iv_settings_t *s, bool selected)
{
    char value[12];
    iv_menu_value(item, s, value, sizeof(value));

    const int ty = (IV_MENU_ROW_H - IV_UI_ROW_H) / 2;
    iv_band_begin(IV_MENU_ROW_H, selected ? SEL_BG : IV_UI_BLACK);

    if (iv_menu_has_arrows(item)) {
        iv_band_text(2, ty, IV_UI_SCALE, IV_UI_GRAY, "<");
        iv_band_text(BSP_LCD_H_RES - IV_UI_CHAR_W - 2, ty, IV_UI_SCALE, IV_UI_GRAY, ">");
    }

    iv_band_text(IV_UI_CHAR_W + 2, ty, IV_UI_SCALE, IV_UI_WHITE, iv_menu_title(item));

    const int vw = iv_band_text_w(IV_UI_SCALE, value);
    iv_band_text(BSP_LCD_H_RES - IV_UI_CHAR_W - 4 - vw, ty, IV_UI_SCALE,
                 IV_UI_CYAN, value);

    iv_band_flush(IV_MENU_TOP + item * IV_MENU_ROW_H);
}

void iv_ui_menu_reset(void)
{
    memset(&s_prev, 0, sizeof(s_prev));
    s_prev.valid = false;
}

void iv_ui_menu_draw(const iv_menu_t *m, const iv_settings_t *s)
{
    const bool first = !s_prev.valid;

    if (first) {
        iv_ui_clear(IV_UI_BLACK);

        iv_band_begin(IV_MENU_TOP, IV_UI_BLACK);
        iv_band_text((BSP_LCD_H_RES - iv_band_text_w(IV_UI_SCALE, MENU_TITLE)) / 2,
                     (IV_MENU_TOP - IV_UI_ROW_H) / 2 - 2, IV_UI_SCALE,
                     IV_UI_YELLOW, MENU_TITLE);
        iv_band_rect(0, IV_MENU_TOP - 2, BSP_LCD_H_RES, 2, IV_UI_DARK);
        iv_band_flush(0);

        /* Подсказка про выход. Она не меняется, поэтому рисуется один раз:
         * без неё из меню можно выйти, только зная про пункт BACK. */
        const int hint_y = IV_MENU_TOP + IV_MENU_COUNT * IV_MENU_ROW_H + 8;
        iv_band_begin(IV_UI_ROW_H, IV_UI_BLACK);
        iv_band_text((BSP_LCD_H_RES - iv_band_text_w(IV_UI_SCALE, MENU_HINT)) / 2,
                     0, IV_UI_SCALE, IV_UI_GRAY, MENU_HINT);
        iv_band_flush(hint_y);

        s_prev.sel = -1;
    }

    for (int i = 0; i < IV_MENU_COUNT; ++i) {
        char value[12];
        iv_menu_value(i, s, value, sizeof(value));

        const bool sel_now  = (i == m->sel);
        const bool sel_prev = (i == s_prev.sel);
        if (first || sel_now != sel_prev || strcmp(value, s_prev.value[i]) != 0) {
            draw_row(i, s, sel_now);
            snprintf(s_prev.value[i], sizeof(s_prev.value[i]), "%s", value);
        }
    }

    s_prev.sel   = m->sel;
    s_prev.valid = true;
    iv_ui_present();
}
