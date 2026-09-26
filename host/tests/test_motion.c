/* Ориентация, калибровка и оценка скорости ведения.
 *
 * Всё это на живой плате проверяется только на слух — «звук ведёт себя
 * странно», — поэтому проверяется здесь, на заведомо известном движении. */
#include "iv_test.h"
#include "iv_motion.h"
#include "iv_control.h" /* IV_BOW_DEADZONE: тишина меряется его порогом */
#include <string.h>

#define SR   500.0f          /* опрос смычка */
#define DT   (1.0f / SR)
#define G_MS2 9.80665f

/* ---- ориентация ---------------------------------------------------------- */

/* Неподвижное устройство: фильтр обязан показать угол наклона из гравитации,
 * и показать его сразу — первая же выборка принимается как есть, иначе после
 * включения инструмент полсекунды считал бы себя лежащим ровно. */
static void test_orient_takes_first_sample(void)
{
    iv_orient_t o;
    iv_orient_init(&o, SR, 0.5f);

    /* Крен 30°: гравитация распределилась между Y и Z. */
    const float a[3] = { 0.0f, 0.5f, 0.8660f };
    const float g[3] = { 0.0f, 0.0f, 0.0f };
    iv_orient_update(&o, a, g);

    IV_CHECK_NEAR(o.roll, 30.0, 0.5);
    IV_CHECK_NEAR(o.pitch, 0.0, 0.5);
}

/* Гироскоп задаёт быстрый отклик: пока акселерометр ещё показывает старое
 * положение, поворот уже виден. Это то, ради чего фьюжн и нужен. */
static void test_orient_follows_gyro(void)
{
    iv_orient_t o;
    iv_orient_init(&o, SR, 0.5f);

    const float level[3] = { 0.0f, 0.0f, 1.0f };
    const float still[3] = { 0.0f, 0.0f, 0.0f };
    iv_orient_update(&o, level, still);

    /* Полсекунды вращения по 60 °/с при «застрявшем» акселерометре. */
    const float spin[3] = { 60.0f, 0.0f, 0.0f };
    for (int i = 0; i < (int)(SR / 2); ++i) {
        iv_orient_update(&o, level, spin);
    }
    /* Гироскоп насчитал бы 30°, гравитация тянет обратно к нулю; истина
     * посередине, но ближе к гироскопу — иначе он бесполезен. */
    IV_CHECK(o.roll > 10.0f && o.roll < 30.0f);
}

/* Через несколько постоянных времени побеждает гравитация: дрейф гироскопа не
 * накапливается, и это главное свойство комплементарного фильтра. */
static void test_orient_converges_to_accel(void)
{
    iv_orient_t o;
    iv_orient_init(&o, SR, 0.5f);

    const float level[3] = { 0.0f, 0.0f, 1.0f };
    const float drift[3] = { 5.0f, 0.0f, 0.0f }; /* смещение нуля 5 °/с */
    iv_orient_update(&o, level, drift);

    for (int i = 0; i < (int)(SR * 5); ++i) {
        iv_orient_update(&o, level, drift);
    }
    /* Установившаяся ошибка от смещения нуля: примерно bias * tau. */
    IV_CHECK(fabs(o.roll) < 3.0);
}

/* Проекция гравитации обязана быть обратной к углам: из неё вычитается
 * гравитация при оценке скорости, и ошибка здесь сразу становится мнимым
 * ускорением. */
static void test_orient_gravity_roundtrip(void)
{
    iv_orient_t o;
    iv_orient_init(&o, SR, 0.5f);

    const float a[3] = { -0.3420f, 0.4698f, 0.8138f }; /* тангаж 20°, крен 30° */
    const float g[3] = { 0.0f, 0.0f, 0.0f };
    iv_orient_update(&o, a, g);

    float grav[3];
    iv_orient_gravity(&o, grav);
    for (int i = 0; i < 3; ++i) {
        IV_CHECK_NEAR(grav[i], a[i], 1e-3);
    }
}

/* ---- калибровка ---------------------------------------------------------- */

/* Калибровать по движущемуся устройству нельзя, и молчаливо получить кривую
 * поправку хуже, чем не получить никакой. */
static void test_calib_needs_stillness(void)
{
    iv_calib_acc_t acc;
    iv_calib_begin(&acc);

    const float a[3] = { 0.0f, 0.0f, 1.0f };
    const float moving[3] = { 30.0f, 0.0f, 0.0f };
    for (int i = 0; i < 1000; ++i) {
        IV_CHECK(!iv_calib_feed(&acc, a, moving, 100));
    }
}

/* На покое: усредняется смещение нуля гироскопа и масштаб акселерометра.
 * Масштаб берётся из того, что модуль ускорения в покое обязан быть 1 g —
 * на живой плате смычка он держался на 1.02. */
static void test_calib_measures_bias_and_scale(void)
{
    iv_calib_acc_t acc;
    iv_calib_begin(&acc);

    const float a[3] = { 0.0f, 0.0f, 1.02f };
    const float g[3] = { 0.7f, -0.3f, 0.1f };

    bool done = false;
    for (int i = 0; i < 200 && !done; ++i) {
        done = iv_calib_feed(&acc, a, g, 100);
    }
    IV_CHECK(done);

    iv_calib_t c;
    iv_calib_take(&acc, &c);
    IV_CHECK(c.valid);
    IV_CHECK_NEAR(c.gyro_bias[0], 0.7, 1e-3);
    IV_CHECK_NEAR(c.gyro_bias[1], -0.3, 1e-3);
    IV_CHECK_NEAR(c.accel_scale, 1.0 / 1.02, 1e-4);

    /* И поправка действительно убирает то, что измерила. */
    float ta[3] = { 0.0f, 0.0f, 1.02f };
    float tg[3] = { 0.7f, -0.3f, 0.1f };
    iv_calib_apply(&c, ta, tg);
    IV_CHECK_NEAR(ta[2], 1.0, 1e-4);
    IV_CHECK_NEAR(tg[0], 0.0, 1e-4);
}

/* Невалидная калибровка не трогает данные: до первой калибровки инструмент
 * обязан играть, пусть и грубее. */
static void test_calib_inactive_is_harmless(void)
{
    iv_calib_t c;
    iv_calib_reset(&c);

    float a[3] = { 0.1f, 0.2f, 1.0f };
    float g[3] = { 1.0f, 2.0f, 3.0f };
    iv_calib_apply(&c, a, g);
    IV_CHECK(a[0] == 0.1f && g[2] == 3.0f);
}

/* ---- скорость ведения ------------------------------------------------------ */

/* Один штрих: скорость нарастает от разворота, проходит максимум посередине и
 * падает к следующему. Ускорение при этом косинусоидально — так ведут смычок,
 * и на такой профиль оценка и рассчитана. */
/* v(t) = peak * sin(pi t / dur) — полуволна на штрих; ускорение это её
 * производная. Настоящая скорость известна, значит есть с чем сверять. */
static float stroke_velocity(float t, float dur, float peak_ms)
{
    return peak_ms * sinf(3.14159265f * t / dur);
}

static float stroke_accel(float t, float dur, float peak_ms)
{
    const float w = 3.14159265f / dur;
    return peak_ms * w * cosf(w * t);
}

/* Смычок лежит: так начинается любое ведение, и так же фильтр ориентации
 * успевает сойтись к вертикали. Без этой прелюдии проба меряла бы не оценку
 * скорости, а промах начальной ориентации. */
static void settle(iv_bowspeed_t *b, iv_orient_t *o, float secs)
{
    const float a[3] = { 0.0f, 0.0f, 1.0f };
    const float g[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < (int)(secs * SR); ++i) {
        iv_orient_update(o, a, g);
        iv_bowspeed_update(b, a, g, NULL, NULL);
    }
}

/* Настоящее ведение: туда-обратно по линии, наклонённой к горизонту на elev
 * градусов. Одиночный полувзмах для угла не годится — фьюжн за него успевает
 * принять разгон за наклон, — а на знакопеременном ходу он этого не успевает,
 * ровно как на инструменте.
 *
 * Прибор при этом лежит ровно; roll_deg поворачивает ЕГО, не меняя движения:
 * так проверяется, что наклон смычка в угол не входит.
 *
 * Возвращает средний угол за последнюю секунду. */
static float feed_bowing(float elev_deg, float roll_deg, float secs)
{
    iv_orient_t   o; iv_orient_init(&o, SR, 0.5f);
    iv_bowspeed_t b; iv_bowspeed_init(&b, SR, 1.0f);

    const float el = elev_deg * 3.14159265f / 180.0f;
    const float r  = roll_deg * 3.14159265f / 180.0f;
    const float g[3] = { 0.0f, 0.0f, 0.0f };

    /* Покой: фьюжн сходится к вертикали в той ориентации, в какой прибор
     * держат. Без этого мерился бы промах начальной ориентации. */
    for (int i = 0; i < (int)(0.5f * SR); ++i) {
        const float a[3] = { 0.0f, sinf(r), cosf(r) };
        iv_orient_update(&o, a, g);
        iv_bowspeed_update(&b, a, g, NULL, NULL);
    }

    float sum = 0.0f;
    int   n = 0;
    const int total = (int)(secs * SR);
    for (int i = 0; i < total; ++i) {
        const float t = i * DT;
        /* Скорость 0.3 м/с, период секунда: обычное ведение. */
        const float acc = 0.3f * 2.0f * 3.14159265f * cosf(2.0f * 3.14159265f * t);
        const float world_h = acc * cosf(el) / G_MS2; /* поперёк гравитации */
        const float world_v = acc * sinf(el) / G_MS2; /* вдоль неё */
        /* Мировые оси раскладываются по осям повёрнутого прибора. */
        const float a[3] = { world_h,
                             sinf(r) * (1.0f + world_v),
                             cosf(r) * (1.0f + world_v) };
        iv_orient_update(&o, a, g);
        float ang = 0.0f;
        iv_bowspeed_update(&b, a, g, &ang, NULL);
        if (t > secs - 1.0f) {
            sum += ang;
            n++;
        }
    }
    return n ? sum / (float)n : -1.0f;
}

/* Штрих по линии, наклонённой к горизонту на elev градусов. Возвращает
 * скорость на середине, а через angle — угол линии, который увидела оценка. */
static float feed_stroke_at(iv_bowspeed_t *b, iv_orient_t *o, float dur,
                            float peak_ms, float elev_deg, float *angle)
{
    const int n = (int)(dur * SR);
    const float el = elev_deg * 3.14159265f / 180.0f;
    float at_mid = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float t = i * DT;
        const float acc = stroke_accel(t, dur, peak_ms) / G_MS2;
        /* Ось X горизонтальна, Z смотрит вверх: наклон линии раскладывает
         * разгон между ними. */
        const float a[3] = { acc * cosf(el), 0.0f, 1.0f + acc * sinf(el) };
        const float g[3] = { 0.0f, 0.0f, 0.0f };
        iv_orient_update(o, a, g);
        const float v = iv_bowspeed_update(b, a, g, angle, NULL);
        if (i == n / 2) {
            at_mid = v;
        }
    }
    return at_mid;
}

static float feed_stroke(iv_bowspeed_t *b, iv_orient_t *o, float dur, float peak_ms)
{
    return feed_stroke_at(b, o, dur, peak_ms, 0.0f, NULL);
}

/* Настоящая скорость на середине штриха известна: проверяем, что оценка
 * попадает в неё, а не просто «что-то показывает».
 *
 * Заниженные семь десятых — не промах, а цена утечки интегратора: она же
 * держит дрейф. Полосу оставляем широкой сознательно, тут проверяется порядок
 * величины и знак, а точный масштаб всё равно доводится на инструменте. */
static void test_bowspeed_tracks_stroke(void)
{
    iv_orient_t o;
    iv_bowspeed_t b;
    iv_orient_init(&o, SR, 0.5f);
    iv_bowspeed_init(&b, SR, 1.0f);
    settle(&b, &o, 0.5f);

    const float mid = feed_stroke(&b, &o, 0.6f, 0.35f);
    const float want = stroke_velocity(0.3f, 0.6f, 0.35f) / IV_BOW_FULL_SPEED_MS;

    IV_CHECK(mid > want * 0.5f && mid < want * 1.2f);
}

/* Обратный штрих обязан звучать так же, как прямой: направление движения
 * инструмент не различает, наружу идёт модуль. Это не потеря информации,
 * а условие того, чтобы струна не менялась дважды за штрих (см. ниже). */
static void test_bowspeed_is_unsigned(void)
{
    iv_orient_t o;
    iv_bowspeed_t b;
    iv_orient_init(&o, SR, 0.5f);
    iv_bowspeed_init(&b, SR, 1.0f);
    settle(&b, &o, 0.5f);

    const float down = feed_stroke(&b, &o, 0.6f, 0.35f);
    settle(&b, &o, 0.5f);
    const float up = feed_stroke(&b, &o, 0.6f, -0.35f);

    IV_CHECK(down > 0.0f);
    IV_CHECK(up > 0.0f);
    IV_CHECK(fabsf(down - up) < down * 0.1f);
}

/* Наклон линии движения — то, чем выбирается струна. Обязан совпадать с тем,
 * под каким углом смычок и правда водят.
 *
 * Допуск в три градуса — замер: оценка держится в долях градуса от истины,
 * пока гироскоп ведёт оценку гравитации. Держать его широким незачем — именно
 * узкий допуск и поймает, если гироскоп оттуда опять уйдёт. */
static void test_move_angle_follows_line(void)
{
    const float want[] = { -60.0f, -30.0f, 0.0f, 30.0f, 60.0f };
    for (int k = 0; k < 5; ++k) {
        IV_CHECK_NEAR(feed_bowing(want[k], 0.0f, 3.0f), want[k], 3.0);
    }
}

/* Знак — вот ради чего угол берётся относительно самой оси смычка. Две
 * зеркальные линии — «вперёд и вверх» и «вперёд и вниз» — это разные струны,
 * и по модулю их не различить вовсе. */
static void test_move_angle_has_sign(void)
{
    const float up   = feed_bowing( 30.0f, 0.0f, 3.0f);
    const float down = feed_bowing(-30.0f, 0.0f, 3.0f);

    IV_CHECK(up > 20.0f);
    IV_CHECK(down < -20.0f);
    /* Симметрично: одинаковый наклон в разные стороны — одинаковый модуль. */
    IV_CHECK(fabsf(up + down) < 5.0f);
}

/* Смычок двигают перпендикулярно ему самому и притом вертикально — вверх-вниз
 * поперёк своей оси. Наклон линии тут 90°, но КУДА она наклонена, сказать
 * нечем: продольной составляющей нет, «вперёд» не определено. Угол обязан
 * остаться прежним, а не заскакать от шума между +90 и −90 — иначе струна
 * прыгала бы всякий раз, когда смычок переставляют, а не ведут им.
 *
 * Движение вбок (по горизонтали поперёк смычка) сюда не относится: там линия
 * и правда горизонтальна, и ноль — верный ответ, а не сбой. */
static void test_move_angle_holds_when_no_direction(void)
{
    iv_bowspeed_t b;
    iv_bowspeed_init(&b, SR, 1.0f);

    const float g[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < (int)(0.5f * SR); ++i) {
        const float a[3] = { 0.0f, 0.0f, 1.0f };
        iv_bowspeed_update(&b, a, g, NULL, NULL);
    }

    /* Сначала ведём как обычно — угол набирается. */
    float ang = 0.0f;
    const float el = 40.0f * 3.14159265f / 180.0f;
    for (int i = 0; i < (int)(2.0f * SR); ++i) {
        const float t = i * DT;
        const float acc = 0.3f * 2.0f * 3.14159265f * cosf(2.0f * 3.14159265f * t);
        const float a[3] = { acc * cosf(el) / G_MS2, 0.0f,
                             1.0f + acc * sinf(el) / G_MS2 };
        iv_bowspeed_update(&b, a, g, &ang, NULL);
    }
    const float held = ang;
    IV_CHECK_NEAR(held, 40.0, 5.0);

    /* Теперь двигаем смычок вертикально поперёк его оси: разгон по Z, вдоль
     * смычка — ничего. Знака взять неоткуда. */
    for (int i = 0; i < (int)(2.0f * SR); ++i) {
        const float t = i * DT;
        const float acc = 0.3f * 2.0f * 3.14159265f * cosf(2.0f * 3.14159265f * t);
        const float a[3] = { 0.0f, 0.0f, 1.0f + acc / G_MS2 };
        iv_bowspeed_update(&b, a, g, &ang, NULL);
    }
    IV_CHECK(ang > 0.0f);         /* остался на своей стороне */
    IV_CHECK(ang <= held + 5.0f); /* и не уехал круче, чем был */
}

/* Главное свойство линии: на обратном ходу она та же. Если брать направление,
 * а не линию, угол переворачивался бы на каждом развороте, и струна менялась
 * бы дважды за штрих. Проба ведёт четыре полных периода — восемь разворотов, —
 * и требует, чтобы угол всё это время стоял на месте. */
static void test_move_angle_survives_reversal(void)
{
    iv_orient_t   o; iv_orient_init(&o, SR, 0.5f);
    iv_bowspeed_t b; iv_bowspeed_init(&b, SR, 1.0f);
    settle(&b, &o, 0.5f);

    const float el = 40.0f * 3.14159265f / 180.0f;
    const float g[3] = { 0.0f, 0.0f, 0.0f };
    float lo = 1e9f, hi = -1e9f;

    for (int i = 0; i < (int)(4.0f * SR); ++i) {
        const float t = i * DT;
        const float acc = 0.3f * 2.0f * 3.14159265f * cosf(2.0f * 3.14159265f * t);
        const float a[3] = { acc * cosf(el) / G_MS2, 0.0f,
                             1.0f + acc * sinf(el) / G_MS2 };
        iv_orient_update(&o, a, g);
        float ang = 0.0f;
        iv_bowspeed_update(&b, a, g, &ang, NULL);
        if (t > 1.0f) { /* даём накопителю набрать линию */
            if (ang < lo) { lo = ang; }
            if (ang > hi) { hi = ang; }
        }
    }
    /* Ни разу не свалился к горизонтали и не встал вертикально: именно это
     * делало бы направление вместо линии. */
    IV_CHECK(lo > 35.0f);
    IV_CHECK(hi < 45.0f);
}

/* Наклон самого смычка на звук не влияет: та же линия движения обязана дать
 * тот же угол, как прибор ни поверни. Это и есть требование «инструмент
 * отвечает на то, что смычком делают, а не на то, как его держат», записанное
 * проверкой. */
static void test_move_angle_ignores_tilt(void)
{
    const float flat    = feed_bowing(30.0f,   0.0f, 3.0f);
    const float rolled  = feed_bowing(30.0f,  40.0f, 3.0f);
    const float rolled2 = feed_bowing(30.0f, -55.0f, 3.0f);

    IV_CHECK_NEAR(flat, 30.0, 3.0);
    IV_CHECK_NEAR(rolled, 30.0, 3.0);
    IV_CHECK_NEAR(rolled2, 30.0, 3.0);
    /* И друг с другом сходятся: важно не только попадание, но и то, что
     * поворот прибора не сдвигает оценку систематически. */
    IV_CHECK(fabsf(flat - rolled) < 2.0f);
    IV_CHECK(fabsf(flat - rolled2) < 2.0f);
}

/* Смычок остановили после штриха — звук обязан замолчать, и ровно, а не
 * «почти». На этом держится тишина инструмента в покое. */
static void test_bowspeed_zero_at_rest(void)
{
    iv_orient_t o;
    iv_bowspeed_t b;
    iv_orient_init(&o, SR, 0.5f);
    iv_bowspeed_init(&b, SR, 1.0f);
    settle(&b, &o, 0.5f);
    feed_stroke(&b, &o, 0.6f, 0.35f);

    const float a[3] = { 0.0f, 0.0f, 1.0f };
    const float g[3] = { 0.2f, -0.1f, 0.05f }; /* шум датчика */

    bool at_rest = false;
    float v = 1.0f;
    for (int i = 0; i < (int)SR; ++i) {
        iv_orient_update(&o, a, g);
        v = iv_bowspeed_update(&b, a, g, NULL, &at_rest);
    }
    IV_CHECK(v == 0.0f);
    IV_CHECK(at_rest);
}

/* Дрожащая рука не должна звучать — и это не «почти не должна».
 *
 * Проба стоит отдельно от предыдущей потому, что ловит другое: там смычок
 * замирает начисто, здесь его держат в руке. Раньше здесь допускалось 0.35
 * при мёртвой зоне 0.08, то есть «дрожь звучит, но не очень громко». Это и
 * была та самая незатухающая нота: всплески дрожи обнуляли счётчик покоя
 * дважды за период, двести миллисекунд подряд не набирались никогда, и
 * остановленный смычок вёл без конца. Порог теперь сравнивается со сглаженным
 * модулем, и требование здесь честное — ниже мёртвой зоны. */
static void test_bowspeed_ignores_tremor(void)
{
    iv_orient_t o;
    iv_bowspeed_t b;
    iv_orient_init(&o, SR, 0.5f);
    iv_bowspeed_init(&b, SR, 1.0f);
    settle(&b, &o, 0.5f);

    /* Дрожь — это мелкое качание около одного положения, а не поворот:
     * среднее нулевое. Качание задаётся углом, а показания обоих датчиков
     * выводятся из него — и гироскоп, и гравитация в акселерометре. Подавать
     * гироскопу поворот, не поворачивая гравитацию, нельзя: таких данных не
     * бывает, а оценка гравитации теперь ведётся гироскопом и на такой смеси
     * честно сойдёт с ума. */
    const float wobble_hz = 10.0f;
    const float wobble_deg = 20.0f / (2.0f * 3.14159265f * wobble_hz); /* ±20 °/с */

    uint32_t rnd = 12345;
    float worst = 0.0f;
    for (int i = 0; i < (int)(SR * 10); ++i) {
        const float t = i * DT;
        const float th = wobble_deg * sinf(2.0f * 3.14159265f * wobble_hz * t);
        const float w  = wobble_deg * 2.0f * 3.14159265f * wobble_hz
                       * cosf(2.0f * 3.14159265f * wobble_hz * t);
        const float rad = th * 3.14159265f / 180.0f;

        rnd = rnd * 1664525u + 1013904223u;
        const float n = ((float)(int32_t)rnd / 2147483648.0f) * 0.05f; /* ±0.05 g */

        const float a[3] = { n, sinf(rad), cosf(rad) };
        const float g[3] = { w, 0.0f, 0.0f };
        iv_orient_update(&o, a, g);
        const float v = iv_bowspeed_update(&b, a, g, NULL, NULL);
        if (fabsf(v) > worst) {
            worst = fabsf(v);
        }
    }
    IV_CHECK(worst < IV_BOW_DEADZONE);
}

/* Смычок ведут И поворачивают — как при каждой смене струны. Поворот обязан
 * пройти мимо оценки движения: он меняет то, как смычок держат, а не то, куда
 * он едет.
 *
 * Это та проба, которой не было и без которой инструмент не работал вовсе.
 * Пока гироскоп в компенсации гравитации не участвовал, поворот целиком уходил
 * в интеграл как настоящее ускорение: замерено, что поворот на 20° за полсекунды
 * загонял оценку скорости в потолок и она там оставалась. Смена струны и есть
 * поворот, так что орало с первого движения. */
static void test_rotation_does_not_fake_motion(void)
{
    /* Один и тот же штрих, с поворотом вокруг продольной оси и без него. */
    float peak[2] = { 0.0f, 0.0f };
    float ang[2]  = { 0.0f, 0.0f };

    for (int turn = 0; turn < 2; ++turn) {
        iv_bowspeed_t b;
        iv_bowspeed_init(&b, SR, 1.0f);

        /* Покой: оценке гравитации надо с чего-то начать. */
        for (int i = 0; i < (int)(0.5f * SR); ++i) {
            const float a[3] = { 0.0f, 0.0f, 1.0f };
            const float g[3] = { 0.0f, 0.0f, 0.0f };
            iv_bowspeed_update(&b, a, g, NULL, NULL);
        }

        const float turn_deg = turn ? 60.0f : 0.0f;
        const float turn_s = 0.5f;
        for (int i = 0; i < (int)(2.0f * SR); ++i) {
            const float t = i * DT;
            /* Поворот идёт посреди ведения, как при переходе на струну. */
            float r = 0.0f, w = 0.0f;
            if (t > 0.5f && t < 0.5f + turn_s) {
                r = turn_deg * (t - 0.5f) / turn_s;
                w = turn_deg / turn_s;
            } else if (t >= 0.5f + turn_s) {
                r = turn_deg;
            }
            const float rad = r * 3.14159265f / 180.0f;
            /* Ведение горизонтально вдоль продольной оси: поворот вокруг неё
             * движения не касается. */
            const float acc = 0.19f * G_MS2 * cosf(2.0f * 3.14159265f * t);
            const float a[3] = { acc / G_MS2, sinf(rad), cosf(rad) };
            const float g[3] = { w, 0.0f, 0.0f };
            const float v = iv_bowspeed_update(&b, a, g, &ang[turn], NULL);
            if (t > 0.5f && v > peak[turn]) {
                peak[turn] = v;
            }
        }
    }

    /* Скорость с поворотом и без обязана совпасть, и притом не упереться
     * в потолок: единица здесь означала бы ровно ту поломку. */
    IV_CHECK(peak[1] < 0.95f);
    IV_CHECK(fabsf(peak[1] - peak[0]) < 0.1f);
    /* И линия движения от поворота не уезжает: водят-то горизонтально. */
    IV_CHECK(ang[0] < 10.0f);
    IV_CHECK(ang[1] < 10.0f);
}

/* И то же самое после настоящего штриха: смычок отвели, остановили, но рука
 * дрожит. Звук обязан кончиться, а не тянуться, пока утечка съест остаток. */
static void test_bowspeed_stops_after_stroke_with_tremor(void)
{
    iv_orient_t o;
    iv_bowspeed_t b;
    iv_orient_init(&o, SR, 0.5f);
    iv_bowspeed_init(&b, SR, 1.0f);
    settle(&b, &o, 0.5f);
    feed_stroke(&b, &o, 0.6f, 0.35f);

    uint32_t rnd = 999;
    float    silent_at = -1.0f;
    for (int i = 0; i < (int)(SR * 2); ++i) {
        rnd = rnd * 1664525u + 1013904223u;
        const float n = ((float)(int32_t)rnd / 2147483648.0f) * 0.045f;
        const float a[3] = { n, 0.0f, 1.0f };
        const float g[3] = { 0.0f, 8.0f * sinf(i * DT * 62.8f), 0.0f };
        iv_orient_update(&o, a, g);
        const float v = iv_bowspeed_update(&b, a, g, NULL, NULL);
        if (v < IV_BOW_DEADZONE && silent_at < 0.0f) {
            silent_at = i * DT;
        }
        if (v >= IV_BOW_DEADZONE) {
            silent_at = -1.0f; /* снова зазвучал — засекаем заново */
        }
    }
    IV_CHECK(silent_at >= 0.0f);
    IV_CHECK(silent_at < 0.5f); /* полсекунды на затихание, не больше */
}

/* ---- дослеживание нуля гироскопа ------------------------------------------- */

/* Ведение с остановками при заданном смещении нуля гироскопа: так играют на
 * самом деле — отрезок штрихов, пауза, снова отрезок. Возвращает угол,
 * увиденный в конце последнего отрезка (когда накопитель уже полон), а через
 * bias_out — то, что петля дослеживания успела выбрать.
 *
 * track = false выключает дослеживание, не трогая ничего больше: так в одной
 * пробе видно и «как было», и «как стало». */
static float feed_bowing_with_bias(float elev_deg, float gyro_bias_dps,
                                   float secs, bool track, float *bias_out)
{
    iv_bowspeed_t b; iv_bowspeed_init(&b, SR, 1.0f);
    if (!track) { b.bias_k = 0.0f; }

    const float el = elev_deg * 3.14159265f / 180.0f;
    const float g[3] = { 0.0f, gyro_bias_dps, 0.0f };
    const float play = 4.0f, pause = 1.0f, cycle = play + pause;

    for (int i = 0; i < (int)(0.5f * SR); ++i) {
        const float a[3] = { 0.0f, 0.0f, 1.0f };
        iv_bowspeed_update(&b, a, g, NULL, NULL);
    }

    float ang = 0.0f, sum = 0.0f; int n = 0;
    for (int i = 0; i < (int)(secs * SR); ++i) {
        const float t = i * DT;
        const float ph = fmodf(t, cycle);
        float acc = 0.0f;
        if (ph < play) {
            acc = 0.3f * 2.0f * 3.14159265f * cosf(2.0f * 3.14159265f * ph);
        }
        const float a[3] = { acc * cosf(el) / G_MS2, 0.0f,
                             1.0f + acc * sinf(el) / G_MS2 };
        iv_bowspeed_update(&b, a, g, &ang, NULL);
        /* Последние полсекунды игрового отрезка — движение установилось. */
        if (t > secs - cycle && ph > play - 0.5f && ph < play) { sum += ang; n++; }
    }
    if (bias_out) { *bias_out = b.bias[1]; }
    return n ? sum / (float)n : ang;
}

/* Смещение нуля гироскопа — единственное, от чего угол линии уползает
 * по-настоящему, и цена ему несуразная: без дослеживания линия в +30° при
 * смещении 2 °/с читается как минус тридцать, то есть вместо струны E звучит
 * G — промах через весь гриф.
 *
 * Почему так дорого: смещение поворачивает оценку гравитации, остаток гравитации
 * попадает в собственное ускорение и интегрируется в мнимую скорость постоянного
 * направления — размером с настоящее ведение. Проба заодно это и показывает:
 * скорость при нетронутом смещении завышена вдвое. */
static void test_gyro_bias_wrecks_angle_without_tracking(void)
{
    const float loose = feed_bowing_with_bias(30.0f, 2.0f, 30.0f, false, NULL);
    IV_CHECK(loose < 0.0f);              /* не просто промах — другая сторона */
    IV_CHECK(fabsf(loose - 30.0f) > 40.0f);
}

/* А с дослеживанием — приходит в норму за десяток секунд игры и остаётся там.
 * Допуск в три градуса тот же, что у пробы без смещения: дослеженное смещение
 * обязано возвращать оценку в её обычную точность, а не «примерно туда». */
static void test_gyro_bias_tracked_away(void)
{
    float bias = 0.0f;
    const float tracked = feed_bowing_with_bias(30.0f, 2.0f, 30.0f, true, &bias);

    IV_CHECK_NEAR(tracked, 30.0, 3.0);
    IV_CHECK_NEAR(bias, 2.0, 0.3);  /* выбрала именно смещение, а не «что-то» */
}

/* Петля не должна выдумывать смещение там, где его нет: на честном датчике
 * дослеженное обязано остаться около нуля, а угол — там же, где был. Иначе
 * лекарство само стало бы болезнью. */
static void test_bias_tracking_invents_little(void)
{
    float bias = 0.0f;
    const float ang = feed_bowing_with_bias(30.0f, 0.0f, 60.0f, true, &bias);

    IV_CHECK_NEAR(ang, 30.0, 3.0);
    IV_CHECK(fabsf(bias) < 0.3f);
}

/* Главное свойство, ради которого смещение считается расхождением с
 * акселерометром, а не разностью «гироскоп минус ноль».
 *
 * Смычок медленно поворачивают рукой — пять градусов в секунду. Покой при этом
 * объявлен: собственного ускорения нет, гироскоп ниже своего порога. Разность с
 * нулём приняла бы весь этот поворот за смещение и увела бы ноль гироскопа на
 * несколько °/с, то есть на струну-другую. Расхождение же тут ровно ноль:
 * гироскоп повернул оценку гравитации, акселерометр показал то же самое,
 * спорить не о чем.
 *
 * Гравитация в осях поворачивающегося прибора: при вращении вокруг Y с
 * угловой скоростью w она идёт как (−sin wt, 0, cos wt) — это и подаётся на
 * акселерометр, чтобы поворот был настоящим, а не выдуманным. */
static void test_bias_tracking_ignores_real_rotation(void)
{
    iv_bowspeed_t b; iv_bowspeed_init(&b, SR, 1.0f);

    const float still[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < (int)(1.0f * SR); ++i) {
        const float a[3] = { 0.0f, 0.0f, 1.0f };
        iv_bowspeed_update(&b, a, still, NULL, NULL);
    }

    const float w_dps = 5.0f;
    const float w = w_dps * 3.14159265f / 180.0f;
    const float g[3] = { 0.0f, w_dps, 0.0f };
    bool rested = false;
    for (int i = 0; i < (int)(3.0f * SR); ++i) {
        const float t = i * DT;
        const float a[3] = { -sinf(w * t), 0.0f, cosf(w * t) };
        bool at_rest = false;
        iv_bowspeed_update(&b, a, g, NULL, &at_rest);
        if (at_rest) { rested = true; }
    }

    /* Проба имеет смысл, только если покой и правда объявлялся: иначе она
     * проверяла бы, что выключенная петля ничего не портит. */
    IV_CHECK(rested);
    IV_CHECK(fabsf(b.bias[1]) < 0.5f);
}

int main(void)
{
    test_orient_takes_first_sample();
    test_orient_follows_gyro();
    test_orient_converges_to_accel();
    test_orient_gravity_roundtrip();
    test_calib_needs_stillness();
    test_calib_measures_bias_and_scale();
    test_calib_inactive_is_harmless();
    test_bowspeed_tracks_stroke();
    test_bowspeed_is_unsigned();
    test_move_angle_follows_line();
    test_move_angle_has_sign();
    test_move_angle_holds_when_no_direction();
    test_move_angle_survives_reversal();
    test_move_angle_ignores_tilt();
    test_bowspeed_zero_at_rest();
    test_rotation_does_not_fake_motion();
    test_bowspeed_ignores_tremor();
    test_bowspeed_stops_after_stroke_with_tremor();
    test_gyro_bias_wrecks_angle_without_tracking();
    test_gyro_bias_tracked_away();
    test_bias_tracking_invents_little();
    test_bias_tracking_ignores_real_rotation();
    IV_TEST_END();
}
