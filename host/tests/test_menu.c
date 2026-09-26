/* Меню: попадание пальца в пункт, шаги значений, границы.
 *
 * На живом инструменте проверяется только удобство; всё остальное —
 * промахнулся ли палец мимо строки, не уехало ли значение за границу, не
 * заворачивается ли громкость с нуля на сотню — проверяется здесь. */
#include "iv_menu.h"
#include "iv_test.h"

#include <string.h>

/* Координата середины строки пункта: тестам нужно попадать в пункт, а не
 * пересчитывать раскладку вручную. */
static int row_center(int item)
{
    return IV_MENU_TOP + item * IV_MENU_ROW_H + IV_MENU_ROW_H / 2;
}

#define X_LEFT   (IV_MENU_WIDTH / 6)
#define X_MID    (IV_MENU_WIDTH / 2)
#define X_RIGHT  (IV_MENU_WIDTH - IV_MENU_WIDTH / 6)

static void test_item_at(void)
{
    IV_CHECK(iv_menu_item_at(0) == -1);
    IV_CHECK(iv_menu_item_at(IV_MENU_TOP - 1) == -1);
    IV_CHECK(iv_menu_item_at(IV_MENU_TOP) == 0);
    IV_CHECK(iv_menu_item_at(IV_MENU_TOP + IV_MENU_ROW_H - 1) == 0);
    IV_CHECK(iv_menu_item_at(IV_MENU_TOP + IV_MENU_ROW_H) == 1);
    IV_CHECK(iv_menu_item_at(row_center(IV_MENU_COUNT - 1)) == IV_MENU_COUNT - 1);
    /* Ниже последнего пункта пунктов нет, и касание там ничего не значит. */
    IV_CHECK(iv_menu_item_at(IV_MENU_TOP + IV_MENU_COUNT * IV_MENU_ROW_H) == -1);
}

static void test_tuning_steps(void)
{
    iv_menu_t m;
    iv_settings_t s;
    iv_menu_init(&m);
    iv_settings_defaults(&s);

    IV_CHECK(iv_menu_touch(&m, X_RIGHT, row_center(IV_MENU_TUNING), &s) == IV_MENU_CHANGED);
    IV_CHECK_NEAR(s.a4_hz, IV_A4_DEFAULT + IV_A4_STEP, 0.001);

    IV_CHECK(iv_menu_touch(&m, X_LEFT, row_center(IV_MENU_TUNING), &s) == IV_MENU_CHANGED);
    IV_CHECK_NEAR(s.a4_hz, IV_A4_DEFAULT, 0.001);

    /* Середина строки у пункта со значением не делает ничего: там нет
     * стрелки, и обещать нажатие нечем. */
    IV_CHECK(iv_menu_touch(&m, X_MID, row_center(IV_MENU_TUNING), &s) == IV_MENU_NONE);
    IV_CHECK_NEAR(s.a4_hz, IV_A4_DEFAULT, 0.001);
}

/* Границы не заворачиваются по кругу: упереться и увидеть, что дальше
 * некуда, понятнее, чем внезапно оказаться на другом конце шкалы. */
static void test_limits_do_not_wrap(void)
{
    iv_menu_t m;
    iv_settings_t s;
    iv_menu_init(&m);
    iv_settings_defaults(&s);

    for (int i = 0; i < 500; ++i) {
        iv_menu_touch(&m, X_RIGHT, row_center(IV_MENU_VOLUME), &s);
        iv_menu_touch(&m, X_RIGHT, row_center(IV_MENU_TUNING), &s);
        iv_menu_touch(&m, X_RIGHT, row_center(IV_MENU_BRIGHT), &s);
    }
    IV_CHECK(s.volume == IV_VOLUME_MAX);
    IV_CHECK(s.brightness == IV_BRIGHT_MAX);
    IV_CHECK_NEAR(s.a4_hz, IV_A4_MAX, 0.001);
    /* Упёршись, пункт перестаёт сообщать об изменении — иначе настройки
     * сохранялись бы во flash на каждом лишнем касании. */
    IV_CHECK(iv_menu_touch(&m, X_RIGHT, row_center(IV_MENU_VOLUME), &s) == IV_MENU_NONE);

    for (int i = 0; i < 500; ++i) {
        iv_menu_touch(&m, X_LEFT, row_center(IV_MENU_VOLUME), &s);
        iv_menu_touch(&m, X_LEFT, row_center(IV_MENU_TUNING), &s);
        iv_menu_touch(&m, X_LEFT, row_center(IV_MENU_BRIGHT), &s);
    }
    IV_CHECK(s.volume == IV_VOLUME_MIN);
    IV_CHECK(s.brightness == IV_BRIGHT_MIN);
    IV_CHECK_NEAR(s.a4_hz, IV_A4_MIN, 0.001);

    /* Что бы ни творил палец, из границ выйти нельзя. */
    IV_CHECK(iv_settings_sanitize(&s) == false);
}

/* Шаг остаётся шагом после сотни правок: без округления к сетке дробный
 * остаток копился бы и строй уехал бы от подписанного. */
static void test_tuning_stays_on_grid(void)
{
    iv_menu_t m;
    iv_settings_t s;
    iv_menu_init(&m);
    iv_settings_defaults(&s);

    for (int i = 0; i < 7; ++i) {
        iv_menu_touch(&m, X_RIGHT, row_center(IV_MENU_TUNING), &s);
    }
    for (int i = 0; i < 3; ++i) {
        iv_menu_touch(&m, X_LEFT, row_center(IV_MENU_TUNING), &s);
    }
    IV_CHECK_NEAR(s.a4_hz, IV_A4_DEFAULT + 4.0f * IV_A4_STEP, 0.0005);
}

static void test_toggle_and_actions(void)
{
    iv_menu_t m;
    iv_settings_t s;
    iv_menu_init(&m);
    iv_settings_defaults(&s);

    /* Переключатель срабатывает от касания в любом месте строки. */
    const bool was = s.snap;
    IV_CHECK(iv_menu_touch(&m, X_LEFT, row_center(IV_MENU_SNAP), &s) == IV_MENU_CHANGED);
    IV_CHECK(s.snap != was);
    IV_CHECK(iv_menu_touch(&m, X_RIGHT, row_center(IV_MENU_SNAP), &s) == IV_MENU_CHANGED);
    IV_CHECK(s.snap == was);

    IV_CHECK(iv_menu_touch(&m, X_MID, row_center(IV_MENU_CENTER), &s) == IV_MENU_DO_CENTER);
    IV_CHECK(iv_menu_touch(&m, X_MID, row_center(IV_MENU_PAIR), &s) == IV_MENU_DO_FORGET);
    IV_CHECK(iv_menu_touch(&m, X_MID, row_center(IV_MENU_DIAG), &s) == IV_MENU_DO_DIAG);
    IV_CHECK(iv_menu_touch(&m, X_MID, row_center(IV_MENU_BACK), &s) == IV_MENU_DO_BACK);

    /* Действия не имеют права трогать настройки. */
    iv_settings_t fresh;
    iv_settings_defaults(&fresh);
    IV_CHECK(iv_settings_equal(&s, &fresh));
}

static void test_touch_outside(void)
{
    iv_menu_t m;
    iv_settings_t s;
    iv_menu_init(&m);
    iv_settings_defaults(&s);

    IV_CHECK(iv_menu_touch(&m, X_MID, 0, &s) == IV_MENU_NONE);
    IV_CHECK(iv_menu_touch(&m, -1, row_center(IV_MENU_VOLUME), &s) == IV_MENU_NONE);
    IV_CHECK(iv_menu_touch(&m, IV_MENU_WIDTH, row_center(IV_MENU_VOLUME), &s) == IV_MENU_NONE);
    IV_CHECK(s.volume == IV_VOLUME_DEFAULT);
}

/* Стрелки рисуются ровно у тех пунктов, у которых левая и правая трети
 * действительно что-то меняют. Экран не имеет права обещать управление,
 * которого нет. */
static void test_arrows_match_behaviour(void)
{
    for (int item = 0; item < IV_MENU_COUNT; ++item) {
        iv_menu_t m;
        iv_settings_t s;
        iv_menu_init(&m);
        iv_settings_defaults(&s);

        const iv_menu_result_t left = iv_menu_touch(&m, X_LEFT, row_center(item), &s);
        const bool changes_value = (left == IV_MENU_CHANGED);
        const bool is_toggle     = (item == IV_MENU_SNAP);

        IV_CHECK(iv_menu_has_arrows(item) == (changes_value && !is_toggle));

        /* И подпись, и значение обязаны быть непустыми: пустая строка в меню
         * это пункт, о котором нельзя догадаться. */
        char v[12];
        iv_menu_value(item, &s, v, sizeof(v));
        IV_CHECK(strlen(iv_menu_title(item)) > 0);
        IV_CHECK(strlen(v) > 0);
    }
}

int main(void)
{
    test_item_at();
    test_tuning_steps();
    test_limits_do_not_wrap();
    test_tuning_stays_on_grid();
    test_toggle_and_actions();
    test_touch_outside();
    test_arrows_match_behaviour();
    IV_TEST_END();
}
