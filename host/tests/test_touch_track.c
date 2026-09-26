/* Разведение призрачных касаний.
 *
 * Самоёмкостный сенсор не знает точек — он знает пики по осям, и из двух
 * пиков по каждой оси собирает пару вслепую. Половину времени он собирает её
 * неверно, и на грифе это слышно как две ноты, поменявшиеся струнами.
 * Проверять это на плате нечем: какая пара настоящая, знают только пальцы.
 * Поэтому — тестом.
 *
 * Договорённость по координатам здешняя, экранная: x поперёк грифа (струна),
 * y вдоль него (нота). */
#include "iv_test.h"
#include "iv_touch_track.h"
#include <string.h>

/* Кадр из одной точки. */
static iv_track_frame_t one(int x, int y)
{
    iv_track_frame_t f;
    memset(&f, 0, sizeof(f));
    f.count = 1;
    f.p[0].x = (int16_t)x;
    f.p[0].y = (int16_t)y;
    return f;
}

/* Кадр из двух точек. */
static iv_track_frame_t two(int x0, int y0, int x1, int y1)
{
    iv_track_frame_t f;
    memset(&f, 0, sizeof(f));
    f.count = 2;
    f.p[0].x = (int16_t)x0;
    f.p[0].y = (int16_t)y0;
    f.p[1].x = (int16_t)x1;
    f.p[1].y = (int16_t)y1;
    return f;
}

/* Есть ли в кадре такая точка. Порядок точек сенсор не обещает, и требовать
 * его от разведения тоже нельзя — важен набор. */
static int has(const iv_track_frame_t *f, int x, int y)
{
    for (uint8_t i = 0; i < f->count; ++i) {
        if (f->p[i].x == x && f->p[i].y == y) {
            return 1;
        }
    }
    return 0;
}

/* Одна точка призраков не порождает: пик по каждой оси один, пересечение
 * единственное. Такой кадр обязан проходить нетронутым. */
static void test_single_point_passes_through(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = one(40, 300);
    iv_track_resolve(&t, &f);

    IV_CHECK(f.count == 1);
    IV_CHECK(has(&f, 40, 300));
}

/* Палец стоял на струне G у подставки. Ложится второй — на E у порожка.
 * Контроллер отдаёт призрачную пару: те же x и те же y, сцепленные наоборот.
 * Опора есть — прошлый кадр, — и пара обязана развернуться обратно. */
static void test_ghost_pair_is_corrected(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = one(20, 500);
    iv_track_resolve(&t, &f);

    /* Настоящее: (20,500) и (150,100). Призрачное — то же наизнанку. */
    f = two(20, 100, 150, 500);
    iv_track_resolve(&t, &f);

    IV_CHECK(f.count == 2);
    IV_CHECK(has(&f, 20, 500));
    IV_CHECK(has(&f, 150, 100));
}

/* Та же пара, но пришедшая правильно, портиться не должна: разведение обязано
 * молчать, когда его не звали. */
static void test_true_pair_is_left_alone(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = one(20, 500);
    iv_track_resolve(&t, &f);

    f = two(20, 500, 150, 100);
    iv_track_resolve(&t, &f);

    IV_CHECK(has(&f, 20, 500));
    IV_CHECK(has(&f, 150, 100));
}

/* Опознали одну точку — вторая определилась сама. Ради этого всё и затеяно:
 * опора нужна ровно одна, потому что третьей пары не существует. */
static void test_one_anchor_resolves_both(void)
{
    iv_track_t t;
    iv_track_init(&t);

    /* Достоверна только точка на D. */
    iv_track_frame_t f = one(60, 200);
    iv_track_resolve(&t, &f);

    /* Пришло наизнанку: (60,450) и (130,200). */
    f = two(60, 450, 130, 200);
    iv_track_resolve(&t, &f);

    IV_CHECK(has(&f, 60, 200));  /* опора вернулась на место */
    IV_CHECK(has(&f, 130, 450)); /* и вторая пришла за ней */
}

/* Оба пальца ведут по грифу: каждый кадр опирается на предыдущий, и пара
 * обязана держаться все кадры, а не только первый. */
static void test_tracking_holds_while_both_slide(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = one(20, 500);
    iv_track_resolve(&t, &f);
    f = two(20, 500, 150, 100);
    iv_track_resolve(&t, &f);

    /* Ведём: точка на G едет к порожку, точка на E — к подставке. Пути
     * пересекаются по y, и на каждом шаге контроллер отдаёт призрачную пару. */
    for (int step = 1; step <= 8; ++step) {
        const int yg = 500 - step * 40;
        const int ye = 100 + step * 40;
        f = two(20, ye, 150, yg); /* наизнанку */
        iv_track_resolve(&t, &f);
        IV_CHECK(has(&f, 20, yg));
        IV_CHECK(has(&f, 150, ye));
    }
}

/* Контроллер переставил точки местами, но пара настоящая. Перестановка слотов
 * и призрак — разные вещи, и путать их нельзя: в первом случае трогать нечего.
 * Набор точек обязан остаться тем же. */
static void test_slot_reorder_is_not_a_ghost(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = one(20, 500);
    iv_track_resolve(&t, &f);

    f = two(150, 100, 20, 500); /* те же точки, другой порядок */
    iv_track_resolve(&t, &f);

    IV_CHECK(has(&f, 20, 500));
    IV_CHECK(has(&f, 150, 100));
}

/* Пальцы лежат неподвижно много кадров подряд. Предсказание по скорости не
 * должно уводить пару никуда: скорость нулевая, и ожидание совпадает с тем,
 * где точки и стоят. */
static void test_still_fingers_do_not_drift(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = one(60, 200);
    iv_track_resolve(&t, &f);

    for (int i = 0; i < 30; ++i) {
        f = two(60, 200, 130, 450);
        iv_track_resolve(&t, &f);
        IV_CHECK(has(&f, 60, 200));
        IV_CHECK(has(&f, 130, 450));
    }
}

/* Пальцы на одной струне: пик по x один, и призраку взяться неоткуда —
 * обе сцепки дают один и тот же набор точек. Кадр должен пройти как есть. */
static void test_same_string_has_no_ghost(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = two(60, 200, 60, 500);
    iv_track_resolve(&t, &f);

    IV_CHECK(has(&f, 60, 200));
    IV_CHECK(has(&f, 60, 500));
}

/* Пальцы на одной высоте грифа: один пик по y, и снова однозначно. */
static void test_same_position_has_no_ghost(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = two(20, 300, 150, 300);
    iv_track_resolve(&t, &f);

    IV_CHECK(has(&f, 20, 300));
    IV_CHECK(has(&f, 150, 300));
}

/* Пальцы сняли — память обязана очиститься. Иначе следующая пара опиралась бы
 * на то, где пальцы лежали в прошлый раз, а это чужие координаты. */
static void test_lifting_clears_the_anchor(void)
{
    iv_track_t t;
    iv_track_init(&t);

    iv_track_frame_t f = two(20, 500, 150, 100);
    iv_track_resolve(&t, &f);

    iv_track_frame_t empty;
    memset(&empty, 0, sizeof(empty));
    iv_track_resolve(&t, &empty);

    /* Новая пара легла разом и в других местах: опоры нет, берётся как пришла. */
    f = two(40, 120, 130, 480);
    iv_track_resolve(&t, &f);
    IV_CHECK(has(&f, 40, 120));
    IV_CHECK(has(&f, 130, 480));
}

int main(void)
{
    test_single_point_passes_through();
    test_ghost_pair_is_corrected();
    test_true_pair_is_left_alone();
    test_one_anchor_resolves_both();
    test_tracking_holds_while_both_slide();
    test_slot_reorder_is_not_a_ghost();
    test_still_fingers_do_not_drift();
    test_same_string_has_no_ghost();
    test_same_position_has_no_ghost();
    test_lifting_clears_the_anchor();
    IV_TEST_END();
}
