#include "iv_menu.h"

#include <math.h>
#include <stdio.h>

void iv_menu_init(iv_menu_t *m)
{
    m->sel = 0;
}

int iv_menu_item_at(int y)
{
    if (y < IV_MENU_TOP) {
        return -1;
    }
    const int item = (y - IV_MENU_TOP) / IV_MENU_ROW_H;
    return (item >= 0 && item < IV_MENU_COUNT) ? item : -1;
}

/* -1 — уменьшить, +1 — увеличить, 0 — середина. */
static int zone(int x)
{
    if (x < IV_MENU_WIDTH / 3) {
        return -1;
    }
    if (x >= 2 * IV_MENU_WIDTH / 3) {
        return +1;
    }
    return 0;
}

static bool step_float(float *v, float dir, float step, float lo, float hi)
{
    const float before = *v;
    float next = *v + dir * step;
    if (next < lo) { next = lo; }
    if (next > hi) { next = hi; }
    /* Округление к сетке шага: без него дробный остаток от прошлых правок
     * копился бы и строй уехал бы на доли герца от подписанного. */
    next = roundf(next / step) * step;
    *v = next;
    return next != before;
}

static bool step_u8(uint8_t *v, int dir, int step, int lo, int hi)
{
    int next = (int)*v + dir * step;
    if (next < lo) { next = lo; }
    if (next > hi) { next = hi; }
    const bool changed = (next != (int)*v);
    *v = (uint8_t)next;
    return changed;
}

iv_menu_result_t iv_menu_touch(iv_menu_t *m, int x, int y, iv_settings_t *s)
{
    const int item = iv_menu_item_at(y);
    if (item < 0 || x < 0 || x >= IV_MENU_WIDTH) {
        return IV_MENU_NONE;
    }
    m->sel = item;

    const int dir = zone(x);

    switch (item) {
    case IV_MENU_TUNING:
        return step_float(&s->a4_hz, (float)dir, IV_A4_STEP, IV_A4_MIN, IV_A4_MAX)
                   ? IV_MENU_CHANGED : IV_MENU_NONE;

    case IV_MENU_VOLUME:
        return step_u8(&s->volume, dir, IV_VOLUME_STEP, IV_VOLUME_MIN, IV_VOLUME_MAX)
                   ? IV_MENU_CHANGED : IV_MENU_NONE;

    case IV_MENU_BRIGHT:
        return step_u8(&s->brightness, dir, IV_BRIGHT_STEP, IV_BRIGHT_MIN, IV_BRIGHT_MAX)
                   ? IV_MENU_CHANGED : IV_MENU_NONE;

    case IV_MENU_SNAP:
        s->snap = !s->snap;
        return IV_MENU_CHANGED;

    case IV_MENU_CENTER: return IV_MENU_DO_CENTER;
    case IV_MENU_PAIR:   return IV_MENU_DO_FORGET;
    case IV_MENU_DIAG:   return IV_MENU_DO_DIAG;
    case IV_MENU_BACK:   return IV_MENU_DO_BACK;
    default:             return IV_MENU_NONE;
    }
}

const char *iv_menu_title(int item)
{
    switch (item) {
    case IV_MENU_TUNING: return "TUNING";
    case IV_MENU_SNAP:   return "SNAP";
    case IV_MENU_VOLUME: return "VOLUME";
    case IV_MENU_BRIGHT: return "LIGHT";
    case IV_MENU_CENTER: return "CENTER";
    case IV_MENU_PAIR:   return "PAIRING";
    case IV_MENU_DIAG:   return "DIAG";
    case IV_MENU_BACK:   return "BACK";
    default:             return "?";
    }
}

void iv_menu_value(int item, const iv_settings_t *s, char *out, size_t n)
{
    if (n == 0) {
        return;
    }
    switch (item) {
    case IV_MENU_TUNING: snprintf(out, n, "%.0f", (double)s->a4_hz); break;
    case IV_MENU_SNAP:   snprintf(out, n, "%s", s->snap ? "ON" : "OFF"); break;
    case IV_MENU_VOLUME: snprintf(out, n, "%u", s->volume); break;
    case IV_MENU_BRIGHT: snprintf(out, n, "%u", s->brightness); break;
    /* Показывается сам центр, а не слово «SET»: он живёт в NVS, переживает
     * перепрошивку и в сбитом виде выглядит как неисправность смычка. Числом
     * его видно до нажатия — и видно, изменилось ли оно после. */
    case IV_MENU_CENTER: snprintf(out, n, "%+.0f", (double)s->angle_center); break;
    case IV_MENU_PAIR:   snprintf(out, n, "RESET"); break;
    case IV_MENU_DIAG:   snprintf(out, n, "SHOW"); break;
    case IV_MENU_BACK:   snprintf(out, n, "SAVE"); break;
    default:             out[0] = '\0'; break;
    }
}

bool iv_menu_has_arrows(int item)
{
    return item == IV_MENU_TUNING || item == IV_MENU_VOLUME || item == IV_MENU_BRIGHT;
}
