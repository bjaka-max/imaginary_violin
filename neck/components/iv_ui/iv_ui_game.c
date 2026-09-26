/* Игровой экран.
 *
 * Кадр рисуется с нулевой строки и подряд — это не выбор, а единственный
 * порядок, который понимает панель: вертикально она не адресуется, у неё есть
 * только «сначала» и «дальше за предыдущей передачей» (подробности и история
 * попыток — в bsp_display.c). Нарисовать полосу посреди экрана, не трогая
 * того, что выше, нельзя.
 *
 * Зато **закончить можно раньше**: строки ниже последней записанной сохраняют
 * то, что на них было. На этом и держится вся экономия. Кадр стоит не «сколько
 * изменилось», а «до какой строки пришлось дойти»:
 *
 *   палец едет у порожка  ~70 строк    24 КБ    ~7 мс
 *   палец едет посередине ~350 строк  120 КБ   ~11 мс
 *   сменился заряд         ~46 строк   16 КБ    ~1.5 мс
 *   палец лёг или снялся   640 строк  220 КБ   ~21 мс
 *   сменилась струна       640 строк  220 КБ   ~21 мс
 *
 * Время — по замеру на живой панели: 46 строк уходят за 1474 мкс, то есть
 * шина отдаёт 10.7 МБ/с.
 *
 * Полного кадра стоит всё, что меняет картину до самого низа: жёлтая зажатая
 * часть тянется от пальца к подставке, а голубая струна светится во всю длину.
 * Зато сдвиг пальца по той же струне обходится только до нижнего из двух его
 * положений — ниже них ничего не изменилось.
 *
 * Отсюда же и порог сдвига: пока палец лежит, передач нет совсем.
 *
 * И отсюда же короткий экран. На нём ровно то, что нужно знать глазами во
 * время игры: жив ли смычок, есть ли заряд и где палец. Имя ноты, полоса
 * ведения и подписи струн убраны — каждая деталь здесь стоит не своей высоты,
 * а всего, что над ней. */
#include "iv_ui.h"
#include "iv_ui_band.h"
#include "iv_power.h"

#include <stdio.h>
#include <stdlib.h> /* abs: порог сдвига пальца */
#include <string.h>

/* Метки полутонов поперёк грифа: короткая с краю на каждый полутон, а на
 * опорных — открытая струна, кварта, квинта, октава — линия во всю ширину,
 * как наклейки, которые ставят начинающим. По ним ставят руку, без них
 * безладовый гриф выглядит пустым. */
#define TICK_SHORT 12
#define TICK_H     2

/* Насколько должен сдвинуться палец, чтобы перерисовать кадр.
 *
 * Оценка касания дрожит на точку-другую, и без порога экран перерисовывался бы
 * всё время, пока палец просто лежит. Восемь точек, а не две, потому что кадр
 * стоит не своей высоты, а всего, что выше пальца, — до двухсот двадцати
 * килобайт по QSPI, и это соперничает со звуком. На шестистах сорока точках
 * грифа восемь — это меньше полутона, на глаз граница жёлтого едет плавно. */
#define FINGER_STEP 8

/* Заряд: число и полоска рядом. Число нужно, чтобы решить «хватит ли
 * доиграть», полоска — чтобы не читать число вовсе. */
#define BATT_BAR_W 32
#define BATT_BAR_H 10

_Static_assert(BSP_LCD_H_RES / 2 + 2 + 4 * IV_UI_CHAR_W + 2 + BATT_BAR_W
                   <= BSP_LCD_H_RES,
               "заряд смычка не влезает в строку");

/* Что уже нарисовано. Сравнивается целиком, поэтому поля здесь ровно те, от
 * которых зависит картинка, — и ни одного лишнего. */
static struct {
    bool    valid;
    iv_bow_status_t bow;
    bool    snap;
    uint8_t batt_neck;
    uint8_t batt_bow;
    bool    bow_charging;
    uint8_t strings;
    int     finger_y[IV_VOICES]; /* строка пальца на струне; <0 — открыта */
} s_prev;

/* ---- элементы --------------------------------------------------------------
 *
 * Каждый рисует себя в текущую полосу; y0 — экранная строка её верха. Всё,
 * что не попало в полосу, отсекают сами примитивы. */

static int semitone_y(int i)
{
    return IV_UI_FB_TOP + i * (IV_UI_FB_H - 1) / 12;
}

/* Опорная метка — та, по которой ставят руку. */
static bool is_landmark(int i)
{
    return i == 0 || i == 5 || i == 7 || i == 12;
}

/* Гриф: метки полутонов поперёк и четыре струны вдоль.
 *
 * Задетые смычком светятся голубым и толще — их видно боковым зрением, а
 * именно так на них и смотрят во время игры. На границе секторов задеты две
 * сразу: это двойная нота, и показать надо обе. */
static void band_fingerboard(int y0, uint8_t active, const int fy[IV_VOICES])
{
    for (int i = 0; i <= 12; ++i) {
        const int ty = semitone_y(i);
        iv_band_rect(0, ty - y0, is_landmark(i) ? BSP_LCD_H_RES : TICK_SHORT,
                     TICK_H, IV_UI_DARK);
    }
    for (int i = 0; i < 4; ++i) {
        const bool on = (active & (1u << i)) != 0;
        const int  w  = on ? IV_UI_STRING_W_ON : IV_UI_STRING_W_OFF;
        iv_band_rect(IV_UI_STRING_X(i) - w / 2, IV_UI_FB_TOP - y0, w,
                     IV_UI_FB_H, on ? IV_UI_CYAN : IV_UI_GRAY);
    }

    /* Зажатая часть — поверх, жёлтым и во всю толщину: от пальца вниз, до
     * подставки. Это и есть та часть струны, что звучит. Пальцев бывает два,
     * на разных струнах, и тогда жёлтых отрезков тоже два. */
    for (int i = 0; i < IV_VOICES; ++i) {
        if (fy[i] >= 0) {
            iv_band_rect(IV_UI_STRING_X(i) - IV_UI_STRING_W_ON / 2, fy[i] - y0,
                         IV_UI_STRING_W_ON, BSP_LCD_V_RES - fy[i], IV_UI_YELLOW);
        }
    }
}

/* Строка состояния смычка. Текст короткий не от бедности шрифта, а потому что
 * читают его боковым зрением во время игры: цвет говорит раньше букв. */
static void band_status(const iv_ui_game_t *g, int y0)
{
    const char *text;
    uint16_t    color;

    switch (g->bow) {
    case IV_BOW_READY:       text = "READY";  color = IV_UI_GREEN;  break;
    case IV_BOW_IDLE:        text = "IDLE";   color = IV_UI_GRAY;   break;
    case IV_BOW_CALIBRATING: text = "CAL";    color = IV_UI_YELLOW; break;
    default:                 text = "NO BOW"; color = IV_UI_RED;    break;
    }

    const int ty = IV_UI_STATUS_Y + (IV_UI_STATUS_H - IV_UI_ROW_H) / 2 - y0;
    iv_band_text(2, ty, IV_UI_SCALE, color, text);

    /* Ведомые струны буквами: на грифе они не выделены, и это единственное
     * место, где видно, какая звучит. Их бывает две — двойная нота. */
    if (g->bow == IV_BOW_READY) {
        char names[5];
        int  n = 0;
        for (int i = 0; i < 4; ++i) {
            if (g->strings & (1u << i)) {
                names[n++] = "GDAE"[i];
            }
        }
        names[n] = '\0';
        if (n > 0) {
            iv_band_text(BSP_LCD_H_RES - (n + 2) * IV_UI_CHAR_W - 2, ty,
                         IV_UI_SCALE, IV_UI_CYAN, names);
        }
    }

    /* Привязка к полутонам переключается кнопкой во время игры, и знать её
     * состояние надо, не отрываясь. */
    if (g->snap) {
        iv_band_text(BSP_LCD_H_RES - IV_UI_CHAR_W - 2, ty, IV_UI_SCALE,
                     IV_UI_GREEN, "#");
    }
}

static uint16_t batt_color(uint8_t pct)
{
    if (pct == IV_BATT_UNKNOWN)   { return IV_UI_DARK; }
    if (iv_batt_is_critical(pct)) { return IV_UI_RED; }
    if (iv_batt_is_low(pct))      { return IV_UI_YELLOW; }
    return IV_UI_GRAY;
}

static void batt_one(int x, int y0, char label, uint8_t pct, bool charging)
{
    char t[8];
    if (charging) {
        snprintf(t, sizeof(t), "%c++", label);
    } else if (pct == IV_BATT_UNKNOWN) {
        snprintf(t, sizeof(t), "%c--", label);
    } else {
        snprintf(t, sizeof(t), "%c%u", label, pct);
    }

    const uint16_t color = charging ? IV_UI_GREEN : batt_color(pct);
    iv_band_text(x, IV_UI_BATT_Y + (IV_UI_BATT_H - IV_UI_ROW_H) / 2 - y0,
                 IV_UI_SCALE, color, t);

    /* Четыре знака, а не три: полный заряд пишется как «N100». */
    const int bx = x + 4 * IV_UI_CHAR_W + 2;
    const int by = IV_UI_BATT_Y + (IV_UI_BATT_H - BATT_BAR_H) / 2 - y0;
    iv_band_rect(bx, by, BATT_BAR_W, 1, IV_UI_DARK);
    iv_band_rect(bx, by + BATT_BAR_H - 1, BATT_BAR_W, 1, IV_UI_DARK);
    iv_band_rect(bx, by, 1, BATT_BAR_H, IV_UI_DARK);
    iv_band_rect(bx + BATT_BAR_W - 1, by, 1, BATT_BAR_H, IV_UI_DARK);

    /* Полоска заполняется только у известного заряда: пустая рамка вместо
     * неизвестности честнее полной или нулевой. */
    if (pct != IV_BATT_UNKNOWN) {
        iv_band_rect(bx + 1, by + 1, (BATT_BAR_W - 2) * pct / 100,
                     BATT_BAR_H - 2, color);
    }
}

static void band_batt(const iv_ui_game_t *g, int y0)
{
    batt_one(2, y0, 'N', g->batt_neck, false);
    batt_one(BSP_LCD_H_RES / 2 + 2, y0, 'B', g->batt_bow, g->bow_charging);
}

/* ---- положение пальца -------------------------------------------------------- */

/* Экранная строка пальца. Отрицательное — касания нет. */
static int finger_y(float pos)
{
    if (pos < 0.0f) {
        return -1;
    }
    if (pos > 1.0f) {
        pos = 1.0f;
    }
    int y = IV_UI_FB_TOP + (int)(pos * (float)(IV_UI_FB_H - 1));
    if (y < IV_UI_FB_TOP) {
        y = IV_UI_FB_TOP;
    }
    if (y > BSP_LCD_V_RES - 1) {
        y = BSP_LCD_V_RES - 1;
    }
    return y;
}

/* ---- сборка ------------------------------------------------------------------ */

void iv_ui_game_reset(void)
{
    memset(&s_prev, 0, sizeof(s_prev));
    s_prev.valid = false;
}

/* Кадр: полосы с нулевой строки подряд до bottom. Ниже bottom панель хранит
 * то, что было, — этим и экономим. */
static void render(const iv_ui_game_t *g, const int fy[IV_VOICES], int bottom)
{
    if (bottom > BSP_LCD_V_RES) {
        bottom = BSP_LCD_V_RES;
    }
    for (int y = 0; y < bottom; y += IV_UI_BAND_ROWS) {
        int h = IV_UI_BAND_ROWS;
        if (y + h > bottom) {
            h = bottom - y;
        }
        iv_band_begin(h, IV_UI_BLACK);

        band_status(g, y);
        band_batt(g, y);
        band_fingerboard(y, (uint8_t)(g->strings & 0x0f), fy);

        iv_band_flush(y);
    }
}

void iv_ui_game_draw(const iv_ui_game_t *g)
{
    const bool first = !s_prev.valid;
    int fy[IV_VOICES];
    for (int i = 0; i < IV_VOICES; ++i) {
#if IV_UI_SHOW_TOUCH
        fy[i] = finger_y(g->finger_pos[i]);
#else
        fy[i] = -1; /* касание не показываем — и перерисовывать нечего */
        (void)g->finger_pos[i];
#endif
    }

    /* Дрожание в пару точек кадра не стоит: оценка касания шумит, а палец
     * лежит подолгу, и кадр здесь стоит всего, что выше пальца.
     *
     * Отдельной проверки на смену дорожки больше нет: палец, ушедший на
     * соседнюю струну, — это пропажа на одной и появление на другой, и обе
     * ловятся здесь же. */
    bool appeared = false;
    bool moved    = false;
    for (int i = 0; i < IV_VOICES; ++i) {
        const int was = s_prev.finger_y[i];
        if ((fy[i] < 0) != (was < 0)) {
            appeared = true;
        } else if (fy[i] >= 0 && abs(fy[i] - was) >= FINGER_STEP) {
            moved = true;
        }
    }
    const bool head     = g->bow != s_prev.bow
                       || g->snap != s_prev.snap
                       || (g->strings & 0x0f) != s_prev.strings
                       || g->batt_neck != s_prev.batt_neck
                       || g->batt_bow != s_prev.batt_bow
                       || g->bow_charging != s_prev.bow_charging;

    if (!first && !appeared && !moved && !head) {
        return; /* на экране всё то же самое — шину не трогаем */
    }

    /* Докуда доводить кадр. Верх перерисовывается всегда — он всё равно на
     * пути, — а низ ровно настолько, чтобы накрыть всё изменившееся.
     *
     * Жёлтая часть тянется от пальца до самого низа, поэтому её появление,
     * пропажа и переход на другую струну стоят полного кадра. А вот сдвиг
     * пальца по той же струне — только до нижнего из двух его положений:
     * ниже них жёлтое как было, так и осталось. */
    const bool strings_changed = (g->strings & 0x0f) != s_prev.strings;
    int bottom = (first || appeared || strings_changed)
                     ? BSP_LCD_V_RES : IV_UI_FB_TOP;
    for (int i = 0; i < IV_VOICES; ++i) {
        if (fy[i] > bottom) {
            bottom = fy[i];
        }
        if (s_prev.finger_y[i] > bottom) {
            bottom = s_prev.finger_y[i];
        }
    }

    render(g, fy, bottom);
    iv_ui_present();

    s_prev.valid        = true;
    s_prev.bow          = g->bow;
    s_prev.snap         = g->snap;
    s_prev.strings      = (uint8_t)(g->strings & 0x0f);
    s_prev.batt_neck    = g->batt_neck;
    s_prev.batt_bow     = g->batt_bow;
    s_prev.bow_charging = g->bow_charging;
    for (int i = 0; i < IV_VOICES; ++i) {
        s_prev.finger_y[i] = fy[i];
    }
}
