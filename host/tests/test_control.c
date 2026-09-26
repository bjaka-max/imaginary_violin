/* Раскладка жестов: палец, линия движения смычка, скорость и наклон грифа →
 * параметры синтеза.
 *
 * Всё это на инструменте проверяется только пальцами и ушами, а ошибки здесь
 * не звучат как ошибки — они звучат как «неудобно играть». Поэтому проверяется
 * тестом: доли струн, мёртвая зона, привязка к полутонам и то, что молчание
 * действительно молчит. */
#include "iv_test.h"
#include "iv_control.h"
#include <string.h>

/* Наклон линии движения, попадающий в середину струны k.
 * Струны идут от G к E по 0-20-40-60°, середина раскладки — граница D/A. */
static float mid_of(int k)
{
    return IV_ANGLE_CENTER_DEF + ((float)k - 1.5f) * IV_ANGLE_PER_STRING;
}

/* Середина дорожки пальца для струны k: экран поделён поперёк на четыре. */
static float lane_of(int k)
{
    return ((float)k + 0.5f) / (float)IV_VOICES;
}

/* Положить палец: слот i, дорожка струны k, позиция pos вдоль грифа. */
static void press(iv_control_in_t *in, int i, int k, float pos)
{
    in->touch[i].pos    = pos;
    in->touch[i].across = lane_of(k);
}

/* Снять палец из слота i. */
static void lift(iv_control_in_t *in, int i)
{
    in->touch[i].pos    = -1.0f;
    in->touch[i].across = -1.0f;
}

/* Смычок ведут со средней скоростью, линия движения — на середине струны A,
 * пальцы сняты: все струны открыты. */
static iv_control_in_t bowing(void)
{
    iv_control_in_t in;
    memset(&in, 0, sizeof(in));
    for (int i = 0; i < IV_TOUCH_MAX; ++i) {
        lift(&in, i);
    }
    in.have_bow = true;
    in.bow_velocity = 0.6f;
    in.bow_angle = mid_of(2);
    in.from_bow = true;
    return in;
}

/* Маска зажатых струн, бит 0 — G. Зажатых бывает две. */
static unsigned fingered_mask(const iv_control_params_t *out)
{
    unsigned m = 0;
    for (int k = 0; k < IV_VOICES; ++k) {
        if (out->finger_pos[k] >= 0.0f) {
            m |= 1u << k;
        }
    }
    return m;
}

/* Нота главной струны — та, что показывает экран и слышно громче всего. */
static float lead_hz(const iv_control_params_t *out)
{
    return out->freq_hz[out->string];
}

/* Сколько струн смычок задевает и на месте ли сумма долей. Сумма проверяется
 * везде: на ней держится громкость — уплыви она, и переход между струнами
 * слышался бы провалом или наплывом. */
static int engaged(const iv_control_params_t *out)
{
    float sum = 0.0f;
    int   n = 0;
    for (int k = 0; k < IV_VOICES; ++k) {
        IV_CHECK(out->bow_share[k] >= 0.0f && out->bow_share[k] <= 1.0f);
        sum += out->bow_share[k];
        if (out->bow_share[k] > 0.0f) {
            n++;
        }
    }
    IV_CHECK_NEAR(sum, 1.0, 1e-5);
    return n;
}

/* Нет касания — открытая струна, а не тишина: на скрипке это полноценная нота. */
static void test_open_string(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();
    iv_control_update(&c, &in, &out);

    IV_CHECK(out.string == 2);
    IV_CHECK(engaged(&out) == 1);
    IV_CHECK_NEAR(out.bow_share[2], 1.0, 1e-6);
    IV_CHECK_NEAR(lead_hz(&out), iv_open_hz[2], 0.01);
}

/* Палец укорачивает струну: середина экрана — половина диапазона грифа. */
static void test_touch_raises_pitch(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();
    press(&in, 0, 2, 0.5f);
    iv_control_update(&c, &in, &out);

    const double want = iv_open_hz[2] * pow(2.0, IV_NECK_SEMITONES / 2.0 / 12.0);
    IV_CHECK_NEAR(lead_hz(&out), want, 0.5);

    /* Зажата ровно одна струна — та, на чьей дорожке лежит палец. Остальные
     * звучат открытыми: палец на скрипке прижимает одну струну, а не все. */
    for (int k = 0; k < IV_VOICES; ++k) {
        if (k == 2) {
            IV_CHECK_NEAR(out.freq_hz[k],
                          iv_open_hz[k] * pow(2.0, IV_NECK_SEMITONES / 2.0 / 12.0),
                          0.5);
        } else {
            IV_CHECK_NEAR(out.freq_hz[k], iv_open_hz[k], 0.01);
        }
    }

    /* И у самой подставки — верх диапазона струны. */
    press(&in, 0, 2, 1.0f);
    iv_control_update(&c, &in, &out);
    IV_CHECK_NEAR(lead_hz(&out), iv_open_hz[2] * pow(2.0, IV_NECK_SEMITONES / 12.0), 0.5);
}

/* Безладово играть в ноту трудно: полутон это семь миллиметров экрана.
 * Привязка округляет до ближайшего полутона — и обязана этим отличаться. */
static void test_snap_quantizes_pitch(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    /* Позиция между третьим и четвёртым полутоном. */
    iv_control_in_t in = bowing();
    press(&in, 0, 2, 3.4f / IV_NECK_SEMITONES);

    iv_control_update(&c, &in, &out);
    const double free_hz = lead_hz(&out);

    iv_control_set_snap(&c, true);
    iv_control_update(&c, &in, &out);
    const double snapped = lead_hz(&out);

    IV_CHECK_NEAR(snapped, iv_open_hz[2] * pow(2.0, 3.0 / 12.0), 0.5);
    IV_CHECK(fabs(free_hz - snapped) > 1.0); /* безладово это другая высота */
}

/* Угол смычка к горизонту выбирает струну. Четыре струны по пятнадцать
 * градусов, середина каждой — чистая струна без соседки. */
static void test_angle_selects_string(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    for (int k = 0; k < IV_VOICES; ++k) {
        iv_control_in_t in = bowing();
        in.bow_angle = mid_of(k);
        iv_control_update(&c, &in, &out);
        IV_CHECK(out.string == k);
        IV_CHECK(engaged(&out) == 1);
        IV_CHECK_NEAR(out.bow_share[k], 1.0, 1e-6);
        IV_CHECK_NEAR(lead_hz(&out), iv_open_hz[k], 0.01);
    }

    /* За краями раскладки смычок упирается в крайнюю струну, а не проваливается
     * в тишину: там играют, свесив или задрав руку. */
    iv_control_in_t in = bowing();
    in.bow_angle = -90.0f; /* круто вниз */
    iv_control_update(&c, &in, &out);
    IV_CHECK(engaged(&out) == 1 && out.bow_share[0] == 1.0f);
    in.bow_angle = 90.0f;  /* круто вверх */
    iv_control_update(&c, &in, &out);
    IV_CHECK(engaged(&out) == 1 && out.bow_share[IV_VOICES - 1] == 1.0f);
}

/* Граница между струнами — двойная нота, а не перескок. Ровно на ней обе
 * соседки берут по половине, а дрожь руки меняет доли на проценты: это
 * слышно как дрожь смычка, чем она и является. Ради этого гистерезис и
 * убран — перескакивать больше нечему. */
static void test_double_stop(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    /* Середина раскладки — граница D/A. */
    iv_control_in_t in = bowing();
    in.bow_angle = IV_ANGLE_CENTER_DEF;
    iv_control_update(&c, &in, &out);
    IV_CHECK(engaged(&out) == 2);
    IV_CHECK_NEAR(out.bow_share[1], 0.5, 1e-5);
    IV_CHECK_NEAR(out.bow_share[2], 0.5, 1e-5);
    /* Обе звучат своей нотой, и это квинта. */
    IV_CHECK_NEAR(out.freq_hz[2] / out.freq_hz[1], iv_open_hz[2] / iv_open_hz[1], 1e-5);

    /* Дрожь вокруг границы: доли едут плавно, скачка струны нет. */
    float prev_lo = out.bow_share[1];
    for (float d = -0.5f; d <= 0.5f; d += 0.1f) {
        in.bow_angle = IV_ANGLE_CENTER_DEF + d;
        iv_control_update(&c, &in, &out);
        IV_CHECK(engaged(&out) == 2);
        IV_CHECK(fabsf(out.bow_share[1] - prev_lo) < 0.1f);
        prev_lo = out.bow_share[1];
    }

    /* За краем зоны смешивания — снова одна струна. */
    in.bow_angle = IV_ANGLE_CENTER_DEF + IV_STRING_BLEND_DEG + 0.5f;
    iv_control_update(&c, &in, &out);
    IV_CHECK(engaged(&out) == 1);
    IV_CHECK(out.string == 2);

    /* Трёх струн разом не бывает ни при каком угле: зоны соседних границ не
     * смыкаются, и смычок остаётся смычком, а не гребёнкой. */
    for (float a = -90.0f; a <= 90.0f; a += 0.25f) {
        in.bow_angle = a;
        iv_control_update(&c, &in, &out);
        IV_CHECK(engaged(&out) <= 2);
    }
}

/* Постановка руки у всех своя: середину раскладки можно объявить где угодно,
 * и после этого крайние струны обязаны остаться достижимыми. */
static void test_angle_center(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);
    iv_control_center_angle(&c, 50.0f);

    iv_control_in_t in = bowing();
    in.bow_angle = 50.0f + 0.5f * IV_ANGLE_PER_STRING;
    iv_control_update(&c, &in, &out);
    IV_CHECK(out.string == 2);

    in.bow_angle = 50.0f - 1.5f * IV_ANGLE_PER_STRING;
    iv_control_update(&c, &in, &out);
    IV_CHECK(out.string == 0);
}

/* Смычок и палец делают разное: смычок выбирает, какая струна звучит, палец —
 * какую он зажимает. Не совпали — звучит открытая струна, и это не ошибка, а
 * то же самое, что на инструменте.
 *
 * Проба на это стоит отдельно, потому что тут легко сделать «удобно»: свести
 * зажатие к той струне, которую ведут. Тогда пальцем нельзя было бы промахнуться
 * мимо струны — а промахиваться по ней и учатся. */
static void test_finger_picks_its_own_string(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    /* Ведём по A, палец зажимает A — звучит зажатая нота. */
    iv_control_in_t in = bowing();
    press(&in, 0, 2, 0.5f);
    iv_control_update(&c, &in, &out);
    IV_CHECK(fingered_mask(&out) == (1u << 2));
    IV_CHECK(lead_hz(&out) > iv_open_hz[2] + 1.0f);

    /* Тот же палец, но на дорожке D: A звучит ОТКРЫТОЙ, хотя палец на грифе. */
    press(&in, 0, 1, 0.5f);
    iv_control_update(&c, &in, &out);
    IV_CHECK(fingered_mask(&out) == (1u << 1));
    IV_CHECK_NEAR(lead_hz(&out), iv_open_hz[2], 0.01);
    /* А зажатой оказалась D — она укоротилась, просто по ней не ведут. */
    IV_CHECK(out.freq_hz[1] > iv_open_hz[1] + 1.0f);
}

/* Палец на границе дорожек не должен мигать между зажатой нотой и открытой
 * струной: слышно такое мигание как бульканье, а не как щелчок. */
static void test_finger_lane_hysteresis(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();
    press(&in, 0, 2, 0.5f);
    iv_control_update(&c, &in, &out);
    IV_CHECK(fingered_mask(&out) == (1u << 2));

    /* Сползли на границу A/D и дрожим вокруг неё. */
    const float edge = 2.0f * IV_FINGER_LANE;
    for (int i = 0; i < 20; ++i) {
        in.touch[0].across = edge + ((i % 2) ? 0.005f : -0.005f);
        iv_control_update(&c, &in, &out);
        IV_CHECK(fingered_mask(&out) == (1u << 2));
    }

    /* А заехав за гистерезис — переходим. */
    in.touch[0].across = edge - IV_FINGER_HYST - 0.005f;
    iv_control_update(&c, &in, &out);
    IV_CHECK(fingered_mask(&out) == (1u << 1));
}

/* Два касания зажимают две струны разом — ради этого всё и делалось. Каждое
 * укорачивает свою и на своё: общей высоты у них нет, как нет её и на
 * инструменте, где пальцы стоят в разных позициях. */
static void test_two_fingers_stop_two_strings(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    /* Смычок ровно на границе D/A — задевает обе; пальцы на D и на A. */
    iv_control_in_t in = bowing();
    in.bow_angle = IV_ANGLE_CENTER_DEF;
    press(&in, 0, 1, 0.25f);
    press(&in, 1, 2, 0.75f);
    iv_control_update(&c, &in, &out);

    IV_CHECK(fingered_mask(&out) == ((1u << 1) | (1u << 2)));
    IV_CHECK(engaged(&out) == 2); /* это и есть двойная нота */

    IV_CHECK_NEAR(out.freq_hz[1],
                  iv_open_hz[1] * pow(2.0, 0.25 * IV_NECK_SEMITONES / 12.0), 0.5);
    IV_CHECK_NEAR(out.freq_hz[2],
                  iv_open_hz[2] * pow(2.0, 0.75 * IV_NECK_SEMITONES / 12.0), 0.5);

    /* Незажатые остались открытыми: палец прижимает свою струну, а не все. */
    IV_CHECK_NEAR(out.freq_hz[0], iv_open_hz[0], 0.01);
    IV_CHECK_NEAR(out.freq_hz[3], iv_open_hz[3], 0.01);
}

/* Тач не обещает, в каком слоте придёт какой палец, и переставляет их между
 * кадрами. Те же два касания в обратном порядке обязаны дать тот же звук —
 * иначе перестановка слышалась бы скачком высоты на ровном месте. */
static void test_touch_order_does_not_matter(void)
{
    iv_control_t c;
    iv_control_params_t a, b;

    iv_control_in_t in = bowing();
    iv_control_init(&c);
    press(&in, 0, 0, 0.3f);
    press(&in, 1, 3, 0.8f);
    iv_control_update(&c, &in, &a);

    iv_control_init(&c);
    press(&in, 0, 3, 0.8f);
    press(&in, 1, 0, 0.3f);
    iv_control_update(&c, &in, &b);

    for (int k = 0; k < IV_VOICES; ++k) {
        IV_CHECK_NEAR(a.freq_hz[k], b.freq_hz[k], 0.01);
        IV_CHECK_NEAR(a.finger_pos[k], b.finger_pos[k], 1e-6);
    }
}

/* Два пальца на одной струне — постановка, а не ошибка ввода: звучит тот, что
 * ближе к подставке. Верхний струну уже не укорачивает. */
static void test_two_fingers_one_string(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();
    press(&in, 0, 2, 0.3f);
    press(&in, 1, 2, 0.7f);
    iv_control_update(&c, &in, &out);

    IV_CHECK(fingered_mask(&out) == (1u << 2));
    IV_CHECK_NEAR(out.freq_hz[2],
                  iv_open_hz[2] * pow(2.0, 0.7 * IV_NECK_SEMITONES / 12.0), 0.5);
}

/* Сняли один палец из двух: его струна открылась, вторая осталась зажатой там
 * же. Проба стоит отдельно, потому что «потерять обе» тут проще всего. */
static void test_lifting_one_finger_keeps_the_other(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();
    press(&in, 0, 1, 0.25f);
    press(&in, 1, 2, 0.75f);
    iv_control_update(&c, &in, &out);

    lift(&in, 0);
    iv_control_update(&c, &in, &out);

    IV_CHECK(fingered_mask(&out) == (1u << 2));
    IV_CHECK_NEAR(out.freq_hz[1], iv_open_hz[1], 0.01);
    IV_CHECK_NEAR(out.freq_hz[2],
                  iv_open_hz[2] * pow(2.0, 0.75 * IV_NECK_SEMITONES / 12.0), 0.5);
}

/* Два пальца на соседних струнах стоят по обе стороны одной границы, и
 * гистерезис каждого смотрит на струну, занятую другим. Если дать ему туда
 * притянуть, оба окажутся на одной струне, а вторая зазвучит открытой — то
 * самое бульканье, против которого гистерезис и поставлен. */
static void test_adjacent_fingers_keep_their_strings(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    const float edge = 2.0f * IV_FINGER_LANE; /* граница D/A */

    iv_control_in_t in = bowing();
    press(&in, 0, 1, 0.4f);
    press(&in, 1, 2, 0.6f);
    iv_control_update(&c, &in, &out);
    IV_CHECK(fingered_mask(&out) == ((1u << 1) | (1u << 2)));

    /* Сдвигаем оба вплотную к границе и дрожим: каждый остаётся на своей. */
    for (int i = 0; i < 20; ++i) {
        const float jitter = (i % 2) ? 0.004f : -0.004f;
        in.touch[0].across = edge - 0.008f + jitter;
        in.touch[1].across = edge + 0.008f - jitter;
        iv_control_update(&c, &in, &out);
        IV_CHECK(fingered_mask(&out) == ((1u << 1) | (1u << 2)));
    }
}

/* Второй палец ложится у самой границы с уже зажатой струной. Гистерезис
 * тянет его туда — но там уже стоит первый, и тогда оба оказались бы на одной
 * струне, а соседняя осталась бы открытой. Дорожку, занятую другим касанием,
 * гистерезис забирать не вправе. */
static void test_new_finger_does_not_steal_a_busy_string(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    /* Первый палец прочно на A. */
    iv_control_in_t in = bowing();
    press(&in, 0, 2, 0.6f);
    iv_control_update(&c, &in, &out);
    IV_CHECK(fingered_mask(&out) == (1u << 2));

    /* Второй ложится на D, но у самой границы с A — внутри гистерезиса. */
    in.touch[1].pos    = 0.4f;
    in.touch[1].across = 2.0f * IV_FINGER_LANE - IV_FINGER_HYST / 2.0f;
    iv_control_update(&c, &in, &out);

    IV_CHECK(fingered_mask(&out) == ((1u << 1) | (1u << 2)));
    IV_CHECK_NEAR(out.finger_pos[1], 0.4, 1e-5);
    IV_CHECK_NEAR(out.finger_pos[2], 0.6, 1e-5);
}

/* Двойная нота на соседних струнах, пальцы стоят по обе стороны границы.
 * Сняли один — оставшийся обязан остаться на своей струне. Маска зажатых ещё
 * помнит снятую соседку, и гистерезис норовит утащить палец на неё: слышно
 * это как скачок высоты в момент, когда сняли ЧУЖОЙ палец. */
static void test_lifting_neighbour_does_not_move_the_other(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    const float edge = 2.0f * IV_FINGER_LANE; /* граница D/A */

    iv_control_in_t in = bowing();
    in.touch[0].pos    = 0.4f;
    in.touch[0].across = edge - IV_FINGER_HYST / 4.0f;
    in.touch[1].pos    = 0.6f;
    in.touch[1].across = edge + IV_FINGER_HYST / 4.0f;
    iv_control_update(&c, &in, &out);
    IV_CHECK(fingered_mask(&out) == ((1u << 1) | (1u << 2)));

    /* Сняли палец с D. A остаётся зажатой там же, а не уезжает на D. */
    lift(&in, 0);
    iv_control_update(&c, &in, &out);
    IV_CHECK(fingered_mask(&out) == (1u << 2));
    IV_CHECK_NEAR(out.finger_pos[2], 0.6, 1e-5);
}

/* Мёртвая зона: около нуля скорость обнуляется, но за её краем звук должен
 * возникать с нуля, а не со ступеньки. */
static void test_bow_deadzone(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();

    in.bow_velocity = IV_BOW_DEADZONE * 0.5f;
    iv_control_update(&c, &in, &out);
    IV_CHECK(out.bow_velocity == 0.0f);

    in.bow_velocity = IV_BOW_DEADZONE * 1.01f;
    iv_control_update(&c, &in, &out);
    IV_CHECK(out.bow_velocity > 0.0f && out.bow_velocity < 0.02f);

    in.bow_velocity = 1.0f;
    iv_control_update(&c, &in, &out);
    IV_CHECK_NEAR(out.bow_velocity, 1.0, 1e-6);

    /* Знак отбрасывается: простая раскладка не различает направление штриха,
     * и штрих вверх обязан звучать ровно как штрих вниз. */
    in.bow_velocity = -1.0f;
    iv_control_update(&c, &in, &out);
    IV_CHECK_NEAR(out.bow_velocity, 1.0, 1e-6);

    in.bow_velocity = -0.5f;
    iv_control_update(&c, &in, &out);
    const float down = out.bow_velocity;
    in.bow_velocity = 0.5f;
    iv_control_update(&c, &in, &out);
    IV_CHECK_NEAR(out.bow_velocity, down, 1e-6);
}

/* Две причины молчать: смычок объявил остановку или его вообще не слышно.
 * Обе обязаны давать ровный ноль — на нём держится тишина инструмента. */
static void test_silence(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();
    in.bow_at_rest = true;
    iv_control_update(&c, &in, &out);
    IV_CHECK(out.bow_velocity == 0.0f);

    /* Смычка не слышно — струн он не задевает вовсе. Главная при этом не
     * меняется: экран не должен дёргаться оттого, что пропал пакет. */
    in = bowing();
    in.have_bow = false;
    iv_control_update(&c, &in, &out);
    IV_CHECK(out.bow_velocity == 0.0f);
    IV_CHECK(out.string == 2);
    for (int k = 0; k < IV_VOICES; ++k) {
        IV_CHECK(out.bow_share[k] == 0.0f);
    }
}

/* Наклон грифа — ось выразительности: ровно посередине диапазона в покое,
 * и упирается в края, не выходя за них. */
static void test_neck_tilt_brightness(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();
    iv_control_update(&c, &in, &out);
    IV_CHECK_NEAR(out.brightness, 0.5, 1e-6);

    in.neck_tilt = 90.0f;
    iv_control_update(&c, &in, &out);
    IV_CHECK(out.brightness == 1.0f);

    in.neck_tilt = -90.0f;
    iv_control_update(&c, &in, &out);
    IV_CHECK(out.brightness == 0.0f);
}

/* Нажим в простой раскладке постоянен — и обязан не зависеть ни от угла, ни
 * от скорости: иначе выразительность потихоньку утечёт обратно в него, и
 * слышать работу двух оставшихся величин станет нельзя. */
static void test_force_is_constant(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);

    iv_control_in_t in = bowing();
    for (float a = -90.0f; a <= 90.0f; a += 5.0f) {
        in.bow_angle = a;
        for (float v = -1.0f; v <= 1.0f; v += 0.25f) {
            in.bow_velocity = v;
            iv_control_update(&c, &in, &out);
            IV_CHECK(out.bow_force == IV_BOW_FORCE);
        }
    }

    /* Нулевой нажим означал бы, что смычка на струне нет вовсе. */
    IV_CHECK(IV_BOW_FORCE > 0.0f && IV_BOW_FORCE <= 1.0f);
}


/* ---- строй --------------------------------------------------------------- */

/* Камертон двигает весь инструмент целиком: открытые струны едут все вместе,
 * и квинты между ними обязаны остаться квинтами. Меняй строй в синтезе — и
 * палец брал бы одну ноту, а звучала бы другая. */
static void test_tuning_shifts_all_strings(void)
{
    iv_control_t c;
    iv_control_init(&c);

    const float ratio = 442.0f / 440.0f;
    iv_control_set_tuning(&c, 442.0f);

    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        IV_CHECK_NEAR(iv_control_open_hz(&c, k), iv_open_hz[k] * ratio, 0.01);
    }
    /* Квинты целы: отношение соседних струн не изменилось. */
    for (int k = 1; k < IV_SYNTH_VOICES; ++k) {
        IV_CHECK_NEAR(iv_control_open_hz(&c, k) / iv_control_open_hz(&c, k - 1),
                      iv_open_hz[k] / iv_open_hz[k - 1], 0.0005);
    }
}

/* Строй приезжает из flash, и негодное значение туда попасть может. Нулевая
 * или нечисловая частота означала бы бесконечную линию задержки в волноводе,
 * поэтому такие значения обязаны отбрасываться, а не приниматься. */
static void test_tuning_rejects_nonsense(void)
{
    iv_control_t c;
    iv_control_init(&c);
    iv_control_set_tuning(&c, 442.0f);

    iv_control_set_tuning(&c, 0.0f);
    IV_CHECK_NEAR(c.a4_hz, 442.0, 0.001);
    iv_control_set_tuning(&c, -440.0f);
    IV_CHECK_NEAR(c.a4_hz, 442.0, 0.001);
    iv_control_set_tuning(&c, 0.0f / 0.0f);
    IV_CHECK_NEAR(c.a4_hz, 442.0, 0.001);
    iv_control_set_tuning(&c, 20000.0f);
    IV_CHECK_NEAR(c.a4_hz, 442.0, 0.001);
}

/* Нота, которую показывает экран, обязана совпадать с нотой, которую играет
 * синтез. Проверяется через сам слой управления: берётся частота, которую он
 * выдал, и по ней спрашивается имя. */
static void test_note_names(void)
{
    char n[8];

    iv_note_name(440.0f, 440.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "A4") == 0);
    iv_note_name(261.626f, 440.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "C4") == 0);
    iv_note_name(466.164f, 440.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "A#4") == 0);

    /* Открытые струны скрипки. Это и есть проверка «экран не врёт»: имена
     * берутся из тех же частот, которые уходят в синтез. */
    static const char *OPEN[IV_SYNTH_VOICES] = { "G3", "D4", "A4", "E5" };
    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        iv_note_name(iv_open_hz[k], 440.0f, n, sizeof(n));
        IV_CHECK(strcmp(n, OPEN[k]) == 0);
    }

    /* При другом камертоне имя не меняется: 415 Гц это по-прежнему A4,
     * просто сам A4 стал ниже. */
    iv_note_name(415.0f, 415.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "A4") == 0);

    /* Округление к ближайшему полутону, а не отбрасывание: цент ниже A4 —
     * это всё ещё A4, а не G#4. */
    iv_note_name(439.7f, 440.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "A4") == 0);

    /* Мусор на входе не имеет права стать нотой. */
    iv_note_name(0.0f, 440.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "--") == 0);
    iv_note_name(0.0f / 0.0f, 440.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "--") == 0);
}

/* Верх и низ диапазона инструмента: от открытой G до октавы на струне E.
 * Это тот самый рабочий диапазон скрипки, ради которого взята октава на
 * струну, — и проверяется он именно здесь. */
static void test_range_covers_violin(void)
{
    iv_control_t c;
    iv_control_params_t out;
    iv_control_init(&c);
    char n[8];

    iv_control_in_t in = bowing();
    in.bow_angle = mid_of(0); /* самая низкая струна */
    lift(&in, 0);          /* открытая */
    iv_control_update(&c, &in, &out);
    iv_note_name(lead_hz(&out), 440.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "G3") == 0);

    in.bow_angle = mid_of(3); /* самая высокая струна */
    press(&in, 0, 3, 1.0f);   /* у подставки */
    iv_control_update(&c, &in, &out);
    iv_note_name(lead_hz(&out), 440.0f, n, sizeof(n));
    IV_CHECK(strcmp(n, "E6") == 0);
}

/* ---- набор середины раскладки ---------------------------------------------- */

/* Кормит раскладку ведением: secs секунд на скорости v под углом angle.
 * Метки времени идут своим чередом, потому что по ним и считается шаг. */
static void feed_bow(iv_control_t *c, int64_t *t_us, float secs, float v,
                     float angle, bool at_rest)
{
    iv_control_in_t in;
    iv_control_params_t out;
    memset(&in, 0, sizeof(in));
    for (int i = 0; i < IV_TOUCH_MAX; ++i) {
        in.touch[i].pos = -1.0f;
        in.touch[i].across = -1.0f;
    }
    in.have_bow = true;

    const int64_t step = 5000; /* 200 Гц — темп пакетов смычка */
    for (int64_t k = 0; k < (int64_t)(secs * 1e6f) / step; ++k) {
        *t_us += step;
        in.stamp_us     = *t_us;
        in.from_bow     = true;
        in.bow_velocity = v;
        in.bow_angle    = angle;
        in.bow_at_rest  = at_rest;
        iv_control_update(c, &in, &out);
    }
}

/* Пока ведения не было, объявлять середину не из чего, и раскладка обязана
 * отказаться, а не взять что попало. Молча запомненный мусор живёт в NVS и
 * потом выглядит как неисправность смычка. */
static void test_center_refuses_without_bowing(void)
{
    iv_control_t c;
    iv_control_init(&c);
    int64_t t = 1000000;

    IV_CHECK(!iv_control_center_candidate(&c, NULL));

    /* Смычок стоит: угол приходит, но весит ноль. */
    feed_bow(&c, &t, 3.0f, 0.0f, 40.0f, true);
    IV_CHECK(!iv_control_center_candidate(&c, NULL));
}

/* Поводили — середина берётся из ведения. */
static void test_center_takes_bowed_angle(void)
{
    iv_control_t c;
    iv_control_init(&c);
    int64_t t = 1000000;

    feed_bow(&c, &t, 3.0f, 0.4f, 12.0f, false);

    float got = 0.0f;
    IV_CHECK(iv_control_center_candidate(&c, &got));
    IV_CHECK_NEAR(got, 12.0, 0.5);
}

/* Та самая ловушка с живого инструмента: поводили горизонтально, отложили
 * смычок — и откладывание оказалось последним движением. Раньше в центр
 * попадало именно оно (в настройках стояло +30° при горизонтальном ведении).
 * Теперь вес отдан ведению: откладывание медленное и короткое, и сдвинуть
 * середину больше чем на долю струны оно не может. */
static void test_center_survives_putting_bow_down(void)
{
    iv_control_t c;
    iv_control_init(&c);
    int64_t t = 1000000;

    feed_bow(&c, &t, 3.0f, 0.4f, 0.0f, false);   /* играли горизонтально */
    feed_bow(&c, &t, 0.3f, 0.25f, -45.0f, false); /* отложили: круто и вяло */

    float got = 0.0f;
    IV_CHECK(iv_control_center_candidate(&c, &got));
    /* Не «сдвинулось мало», а не сдвинулось вовсе: откладывание попало в свои
     * корзины и отброшено целиком, а не разбавило среднее. Гистограмма именно
     * этим и отличается от среднего, и проба требует этой разницы. */
    IV_CHECK(fabsf(got) < 1.0f);

    /* И проверка, что проба вообще про что-то: последний услышанный угол,
     * который брался раньше, увёл бы середину на две струны. */
    IV_CHECK(fabsf(-45.0f) > IV_ANGLE_PER_STRING * 2.0f);
}

/* Смычок уронили: секунда быстрого движения под одним углом. Свидетельств у
 * него больше, чем у трёх секунд вялого ведения (замер: вес 0.80 против 0.66),
 * и какое из двух движений было ведением, из данных не следует.
 *
 * Тут единственный правильный ответ — отказаться. Середина уходит в NVS и
 * переживает перепрошивку, так что угаданная неверно она потом выглядит как
 * неисправность смычка; а отказ виден сразу — значение в меню не изменилось. */
static void test_center_refuses_when_ambiguous(void)
{
    iv_control_t c;
    iv_control_init(&c);
    int64_t t = 1000000;

    feed_bow(&c, &t, 3.0f, 0.4f, 0.0f, false);
    IV_CHECK(iv_control_center_candidate(&c, NULL)); /* пока однозначно */

    feed_bow(&c, &t, 1.0f, 0.9f, -60.0f, false);
    IV_CHECK(!iv_control_center_candidate(&c, NULL));
}

/* Несвежее ведение забывается: поводил минуту назад — объявлять нечего.
 * Отказ тут лучше, чем центр, снятый неизвестно когда и неизвестно как. */
static void test_center_forgets_stale_bowing(void)
{
    iv_control_t c;
    iv_control_init(&c);
    int64_t t = 1000000;

    feed_bow(&c, &t, 3.0f, 0.4f, 12.0f, false);
    IV_CHECK(iv_control_center_candidate(&c, NULL));

    feed_bow(&c, &t, 30.0f, 0.0f, 12.0f, true); /* смычок лежит полминуты */
    IV_CHECK(!iv_control_center_candidate(&c, NULL));
}

int main(void)
{
    test_open_string();
    test_touch_raises_pitch();
    test_snap_quantizes_pitch();
    test_angle_selects_string();
    test_double_stop();
    test_angle_center();
    test_center_refuses_without_bowing();
    test_center_takes_bowed_angle();
    test_center_survives_putting_bow_down();
    test_center_refuses_when_ambiguous();
    test_center_forgets_stale_bowing();
    test_finger_picks_its_own_string();
    test_finger_lane_hysteresis();
    test_two_fingers_stop_two_strings();
    test_touch_order_does_not_matter();
    test_two_fingers_one_string();
    test_lifting_one_finger_keeps_the_other();
    test_adjacent_fingers_keep_their_strings();
    test_new_finger_does_not_steal_a_busy_string();
    test_lifting_neighbour_does_not_move_the_other();
    test_bow_deadzone();
    test_silence();
    test_neck_tilt_brightness();
    test_force_is_constant();
    test_tuning_shifts_all_strings();
    test_tuning_rejects_nonsense();
    test_note_names();
    test_range_covers_violin();
    IV_TEST_END();
}
