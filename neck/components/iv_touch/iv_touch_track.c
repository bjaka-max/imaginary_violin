#include "iv_touch_track.h"

#include <string.h>

void iv_track_init(iv_track_t *t)
{
    memset(t, 0, sizeof(*t));
}

/* Квадрат расстояния. Корень не нужен: расстояния сравниваются только между
 * собой, а он монотонен. В int32 влезает с запасом — экран 172×640. */
static int32_t dist2(iv_track_point_t a, iv_track_point_t b)
{
    const int32_t dx = (int32_t)a.x - b.x;
    const int32_t dy = (int32_t)a.y - b.y;
    return dx * dx + dy * dy;
}

/* Где точки окажутся в этом кадре, если пальцы продолжат двигаться как шли.
 *
 * Без предсказания разведение слепнет ровно там, где важнее всего: когда
 * пальцы идут встречно и проходят одну высоту грифа, опорные точки в этот миг
 * различаются только поперечной координатой, а она у обеих пар одинакова.
 * Расстояния до настоящей и до призрачной пары выходят равны, и следующий же
 * кадр уезжает на призрака — две ноты меняются струнами прямо посреди хода.
 * Скорость эту симметрию и снимает: она помнит, куда палец шёл.
 *
 * Скорость берётся сопоставлением с позапрошлым кадром по ближайшей точке.
 * Пальцы стоят на разных струнах, то есть далеко друг от друга поперёк грифа,
 * и перепутать их сопоставление не может: поперёк они не пересекаются, иначе
 * столкнулись бы. */
static iv_track_frame_t predict(const iv_track_t *t)
{
    iv_track_frame_t out = t->trusted;
    if (t->prev.count != t->trusted.count) {
        return out; /* сравнивать не с чем — предсказание вырождается в позицию */
    }
    for (uint8_t i = 0; i < t->trusted.count; ++i) {
        uint8_t best = 0;
        for (uint8_t j = 1; j < t->prev.count; ++j) {
            if (dist2(t->trusted.p[i], t->prev.p[j])
                < dist2(t->trusted.p[i], t->prev.p[best])) {
                best = j;
            }
        }
        out.p[i].x = (int16_t)(2 * t->trusted.p[i].x - t->prev.p[best].x);
        out.p[i].y = (int16_t)(2 * t->trusted.p[i].y - t->prev.p[best].y);
    }
    return out;
}

/* Насколько пара похожа на то, где точки ожидались: сумма квадратов расстояний
 * при лучшем сопоставлении. Меньше — похожее.
 *
 * Сопоставление перебором: вариантов всего два, и перебор здесь короче и
 * честнее любого «умного» правила. */
static int32_t pair_cost(const iv_track_frame_t *want,
                         iv_track_point_t a, iv_track_point_t b)
{
    if (want->count == 1) {
        /* Ожидаем одну точку — второй палец только что лёг. Смотрим, к какой
         * из пары она ближе: этого хватает, чтобы опознать пару целиком. */
        const int32_t ca = dist2(want->p[0], a);
        const int32_t cb = dist2(want->p[0], b);
        return (ca < cb) ? ca : cb;
    }
    const int32_t direct  = dist2(want->p[0], a) + dist2(want->p[1], b);
    const int32_t swapped = dist2(want->p[0], b) + dist2(want->p[1], a);
    return (direct < swapped) ? direct : swapped;
}

void iv_track_resolve(iv_track_t *t, iv_track_frame_t *f)
{
    if (f->count < IV_TRACK_MAX || t->trusted.count == 0) {
        /* Одна точка или ни одной — призраку взяться неоткуда: он родится
         * только там, где пиков по обеим осям по два. Либо опереться не на
         * что, потому что оба пальца легли в один кадр — тогда пара берётся
         * как пришла, см. шапку заголовка. */
        t->prev    = t->trusted;
        t->trusted = *f;
        return;
    }

    /* Вторая возможная пара — те же два x и те же два y, сцепленные наоборот.
     * Третьей не существует, и на этом всё держится: опознали одну точку —
     * вторая определилась сама. */
    const iv_track_point_t a0 = f->p[0], a1 = f->p[1];
    const iv_track_point_t b0 = { a0.x, a1.y };
    const iv_track_point_t b1 = { a1.x, a0.y };

    const iv_track_frame_t want = predict(t);
    if (pair_cost(&want, b0, b1) < pair_cost(&want, a0, a1)) {
        f->p[0] = b0;
        f->p[1] = b1;
    }
    t->prev    = t->trusted;
    t->trusted = *f;
}
