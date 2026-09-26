#include "iv_control.h"
#include "iv_dsp.h"

#include <math.h>
#include <stdio.h>

void iv_control_init(iv_control_t *c)
{
    c->snap = false;
    c->a4_hz = 440.0f;
    c->angle_center = IV_ANGLE_CENTER_DEF;
    c->string = 2; /* A: с неё удобно начинать, она посередине */
    c->fingered = 0; /* пальцы сняты: все струны открыты */
    c->center_stamp = 0;
    for (int i = 0; i < IV_CENTER_BINS; ++i) {
        c->center_hist[i] = 0.0f;
    }
}

void iv_control_set_snap(iv_control_t *c, bool on)
{
    c->snap = on;
}

void iv_control_set_tuning(iv_control_t *c, float a4_hz)
{
    /* Проверка не косметическая: строй приезжает из настроек, а те — из
     * flash. Ноль или NaN здесь означал бы ноту нулевой частоты, то есть
     * бесконечную линию задержки в волноводе. */
    if (a4_hz > 300.0f && a4_hz < 600.0f) {
        c->a4_hz = a4_hz;
    }
}

float iv_control_open_hz(const iv_control_t *c, int string)
{
    if (string < 0) { string = 0; }
    if (string >= IV_SYNTH_VOICES) { string = IV_SYNTH_VOICES - 1; }
    return iv_open_hz[string] * (c->a4_hz / 440.0f);
}

void iv_control_center_angle(iv_control_t *c, float angle_deg)
{
    c->angle_center = angle_deg;
}

void iv_control_shares(const iv_control_t *c, float angle_deg, float out[IV_VOICES])
{
    /* Положение смычка поперёк струн: 0 — G, 3 — E. Середина раскладки
     * приходится на границу D/A: по полторы струны в каждую сторону. За
     * краями смычок упирается в крайнюю струну — водишь совсем полого или
     * совсем круто, а не проваливаешься в тишину. */
    const float x = iv_clampf((angle_deg - c->angle_center) / IV_ANGLE_PER_STRING + 1.5f,
                              0.0f, (float)(IV_VOICES - 1));

    /* Полуширина зоны смешивания в долях расстояния между струнами. Доля
     * растёт от нуля на дальнем краю зоны до единицы на ближнем, посередине
     * обе соседки дают по половине. Сумма долей равна единице тождественно:
     * что одна струна теряет, соседняя ровно то и получает. */
    const float half = IV_STRING_BLEND_DEG / IV_ANGLE_PER_STRING;

    for (int k = 0; k < IV_VOICES; ++k) {
        const float d = fabsf(x - (float)k);
        out[k] = iv_clampf((0.5f + half - d) / (2.0f * half), 0.0f, 1.0f);
    }
}

/* Дорожка касания без гистерезиса: экран поделён поперёк на четыре. */
static int raw_lane(float across)
{
    const float x = iv_clampf(across, 0.0f, 1.0f);
    int k = (int)(x / IV_FINGER_LANE);
    if (k >= IV_VOICES) {
        k = IV_VOICES - 1; /* x ровно единица — это ещё дорожка E, не пятая */
    }
    return k;
}

/* Куда гистерезис уводит касание с дорожки k.
 *
 * Гистерезис привязан к струне, а не к слоту касания: тач слоты переставляет,
 * а струна, на которой уже лежит палец, никуда не девается. Стоим у границы с
 * дорожкой, которая была зажата в прошлом кадре, — остаёмся на ней. Без этого
 * палец на самой границе мигал бы между двумя струнами, а слышно это как
 * дребезг высоты по нескольку раз в секунду.
 *
 * Порядок проверок тут и есть всё содержание, и каждая закрыта пробой:
 *
 * 1. Своя дорожка уже была зажата — остаёмся на ней, не глядя на границы.
 *    Гистерезис ДЕРЖИТ струну, а не тянет с неё. Без этого снятие ЧУЖОГО
 *    пальца сдёргивало бы оставшийся на освободившуюся соседку, которую маска
 *    ещё помнит зажатой, и слышно это как скачок высоты в момент, когда
 *    сняли другой палец (test_lifting_neighbour_does_not_move_the_other).
 * 2. Соседку занял другой палец в этом же кадре — не отдаём: два касания на
 *    одной струне означали бы, что вторая молчит
 *    (test_new_finger_does_not_steal_a_busy_string).
 * 3. И только потом — притяжение: у самой границы с дорожкой, которая была
 *    зажата, переходим на неё. */
static int pull_lane(const iv_control_t *c, float across, int k, uint8_t taken)
{
    if (c->fingered & (uint8_t)(1u << k)) {
        return k;
    }

    const float x = iv_clampf(across, 0.0f, 1.0f);
    for (int d = -1; d <= 1; d += 2) {
        const int n = k + d;
        if (n < 0 || n >= IV_VOICES) {
            continue;
        }
        if (!(c->fingered & (uint8_t)(1u << n))) {
            continue; /* соседка и не была зажата — тянуть не к чему */
        }
        if (taken & (uint8_t)(1u << n)) {
            continue; /* её уже держит другое касание */
        }
        const float dist = (d < 0) ? x - (float)k * IV_FINGER_LANE
                                   : (float)(k + 1) * IV_FINGER_LANE - x;
        if (dist <= IV_FINGER_HYST) {
            return n;
        }
    }
    return k;
}

/* Середина корзины в градусах. */
static float center_bin_deg(int i)
{
    return -90.0f + (float)i * IV_CENTER_BIN_DEG;
}

bool iv_control_center_candidate(const iv_control_t *c, float *angle_deg)
{
    /* Самая населённая корзина: вокруг неё и усредняем. Именно она отбрасывает
     * выброс целиком — среднее по всей гистограмме выброс лишь разбавило бы,
     * и тем хуже, чем он сильнее (замеры см. в iv_control.h). */
    int top = 0;
    for (int i = 1; i < IV_CENTER_BINS; ++i) {
        if (c->center_hist[i] > c->center_hist[top]) {
            top = i;
        }
    }

    int lo = top - IV_CENTER_SPREAD;
    int hi = top + IV_CENTER_SPREAD;
    if (lo < 0) { lo = 0; }
    if (hi > IV_CENTER_BINS - 1) { hi = IV_CENTER_BINS - 1; }

    float num = 0.0f, den = 0.0f, rest = 0.0f;
    for (int i = 0; i < IV_CENTER_BINS; ++i) {
        if (i >= lo && i <= hi) {
            num += c->center_hist[i] * center_bin_deg(i);
            den += c->center_hist[i];
        } else {
            rest += c->center_hist[i];
        }
    }

    if (!(den >= IV_CENTER_MIN_WEIGHT)) {
        return false; /* через «не >=» — так же отсеивается и NaN */
    }
    /* Второе движение сравнимого веса — значит какое из них ведение, неясно
     * (см. IV_CENTER_AMBIG_FRAC). Угадывать нельзя: центр уходит в NVS. */
    if (rest > den * IV_CENTER_AMBIG_FRAC) {
        return false;
    }
    if (angle_deg) {
        *angle_deg = num / den;
    }
    return true;
}

float iv_control_center_weight(const iv_control_t *c)
{
    float w = 0.0f;
    for (int i = 0; i < IV_CENTER_BINS; ++i) {
        w += c->center_hist[i];
    }
    return w;
}

/* Копит гистограмму углов ведения. Зовётся из iv_control_update на каждой
 * выборке смычка.
 *
 * Шаг берётся из меток времени самих пакетов, а не из частоты задачи, и это
 * не педантизм: задача управления крутится быстрее, чем приходят пакеты, и
 * один и тот же пакет она видит по нескольку раз. По меткам повтор даёт
 * нулевой шаг и не попадает в гистограмму вовсе, а считать по кадрам значило
 * бы учитывать его столько раз, сколько успели посмотреть. */
static void center_accumulate(iv_control_t *c, const iv_control_in_t *in)
{
    float dt = 0.0f;
    if (c->center_stamp != 0) {
        const int64_t d = in->stamp_us - c->center_stamp;
        /* Разрыв связи длиннее десятой секунды — не шаг, а дыра: считать по
         * нему нельзя, потому что что делал смычок в это время, неизвестно. */
        if (d > 0 && d < 100000) {
            dt = (float)d * 1e-6f;
        }
    }
    c->center_stamp = in->stamp_us;
    if (dt <= 0.0f) {
        return;
    }

    const float decay = expf(-dt / IV_CENTER_TAU_S);
    for (int i = 0; i < IV_CENTER_BINS; ++i) {
        c->center_hist[i] *= decay;
    }

    /* Вес — сама скорость смычка, а стоящий смычок и ведение ниже мёртвой
     * зоны не весят ничего: у них угол либо удержан с прошлого движения, либо
     * не определён вовсе, и копить его значило бы копить память. */
    if (in->bow_at_rest || in->bow_velocity < IV_BOW_DEADZONE) {
        return;
    }
    const float w = in->bow_velocity * dt;

    /* Вклад раскладывается между двумя соседними корзинами по расстоянию до
     * их середин. Без этого медленно ползущий угол скакал бы из корзины в
     * корзину, и разрешение упиралось бы в шаг сетки, а не в ширину окна. */
    const float x = (iv_clampf(in->bow_angle, -90.0f, 90.0f) + 90.0f)
                    / IV_CENTER_BIN_DEG;
    const int   i = (int)x;
    const float frac = x - (float)i;
    if (i >= 0 && i < IV_CENTER_BINS) {
        c->center_hist[i] += w * (1.0f - frac);
    }
    if (i + 1 >= 0 && i + 1 < IV_CENTER_BINS) {
        c->center_hist[i + 1] += w * frac;
    }
}

/* Мёртвая зона скорости смычка. Оценка скорости — интеграл, у неё всегда
 * остаётся дрожь около нуля, и без зоны нечувствительности инструмент тихо
 * гудел бы при неподвижном смычке (IV_BOW_DEADZONE).
 *
 * За краем зоны звук начинается с нуля, а не со ступеньки: остаток растягивается
 * обратно на весь диапазон. Обрезать без растяжения значило бы, что смычок,
 * едва тронувшись, сразу звучит на восемь процентов громкости — слышно как
 * щелчок в начале каждого штриха.
 *
 * Знак сохраняется, хотя раскладка берёт модуль: знак тут дешевле сохранить,
 * чем объяснять его потерю тому, кто позовёт эту функцию иначе. */
static float dead_zone(float v)
{
    const float m = fabsf(v);
    if (m <= IV_BOW_DEADZONE) {
        return 0.0f;
    }
    const float scaled = (m - IV_BOW_DEADZONE) / (1.0f - IV_BOW_DEADZONE);
    return (v < 0.0f) ? -scaled : scaled;
}

void iv_control_update(iv_control_t *c, const iv_control_in_t *in,
                       iv_control_params_t *out)
{
    /* --- струны ------------------------------------------------------------
     * Доли считаются по углу смычка. Без смычка задействованных струн нет
     * вовсе, но главная не меняется: молчащий смычок не должен «уводить»
     * струну, на которой только что играли, — экран бы дёргался. */
    if (in->have_bow) {
        center_accumulate(c, in);
        iv_control_shares(c, in->bow_angle, out->bow_share);
        for (int k = 0; k < IV_VOICES; ++k) {
            if (out->bow_share[k] > out->bow_share[c->string]) {
                c->string = k;
            }
        }
    } else {
        for (int k = 0; k < IV_VOICES; ++k) {
            out->bow_share[k] = 0.0f;
        }
    }

    /* --- зажатые струны ----------------------------------------------------
     * Каждое касание зажимает свою струну — ту, на дорожке которой лежит.
     * Касаний бывает два, и тогда зажатых струн тоже две: смычок, задевающий
     * обе, сыграет двойную ноту зажатыми нотами, а не открытыми струнами.
     *
     * Два касания на одной дорожке — это не ошибка ввода, а нормальная
     * постановка: побеждает ближнее к подставке. На скрипке нижний палец
     * укорачивает струну и верхний уже ничего не решает. */
    int raw[IV_TOUCH_MAX];
    for (int i = 0; i < IV_TOUCH_MAX; ++i) {
        raw[i] = (in->touch[i].pos < 0.0f) ? -1 : raw_lane(in->touch[i].across);
    }

    uint8_t fingered = 0;
    float   stop[IV_VOICES];
    for (int k = 0; k < IV_VOICES; ++k) {
        stop[k] = -1.0f; /* струна открыта, пока на неё не легло касание */
    }
    for (int i = 0; i < IV_TOUCH_MAX; ++i) {
        if (raw[i] < 0) {
            continue;
        }
        uint8_t taken = 0;
        for (int j = 0; j < IV_TOUCH_MAX; ++j) {
            if (j != i && raw[j] >= 0) {
                taken |= (uint8_t)(1u << raw[j]);
            }
        }
        /* Дорожка считается по старой маске c->fingered: гистерезису нужно,
         * что было зажато в прошлом кадре, а не что зажимается сейчас. */
        const int   k = pull_lane(c, in->touch[i].across, raw[i], taken);
        const float p = iv_clampf(in->touch[i].pos, 0.0f, 1.0f);
        if (p > stop[k]) {
            stop[k] = p;
        }
        fingered |= (uint8_t)(1u << k);
    }
    c->fingered = fingered;

    /* --- высота ------------------------------------------------------------
     * Незажатые струны звучат открытыми: это не «тишина», а полноценные ноты,
     * как на скрипке. Смычок ведёт своё — совпадёт с пальцем, услышим зажатую
     * ноту, не совпадёт, услышим открытую струну. */
    for (int k = 0; k < IV_VOICES; ++k) {
        float ratio = 1.0f;
        if (stop[k] >= 0.0f) {
            float semitones = stop[k] * IV_NECK_SEMITONES;
            if (c->snap) {
                semitones = roundf(semitones);
            }
            ratio = exp2f(semitones / 12.0f);
        }
        out->freq_hz[k]    = iv_control_open_hz(c, k) * ratio;
        out->finger_pos[k] = stop[k];
    }

    /* --- ведение -----------------------------------------------------------
     * Берётся модуль: направление штриха простая раскладка не различает, а
     * знак, дойдя до синтеза, менял бы фазу возбуждения на каждом развороте.
     *
     * Остановку смычок объявляет сам (ZUPT на его стороне): ждать, пока
     * оценка скорости доедет до нуля, значило бы тянуть звук после того, как
     * рука уже стоит. */
    float velocity = 0.0f;
    if (in->have_bow && !in->bow_at_rest) {
        velocity = fabsf(dead_zone(iv_clampf(in->bow_velocity, -1.0f, 1.0f)));
    }
    out->bow_velocity = velocity;

    /* --- нажим и яркость ---------------------------------------------------- */
    out->bow_force = IV_BOW_FORCE;

    out->brightness = iv_clampf(0.5f + in->neck_tilt / (2.0f * IV_NECK_TILT_DEG),
                                0.0f, 1.0f);

    out->string   = (uint8_t)c->string;
    out->stamp_us = in->stamp_us;
    out->from_bow = in->from_bow;
}

/* ---- имя ноты ------------------------------------------------------------ */

void iv_note_name(float hz, float a4_hz, char *out, size_t n)
{
    if (n == 0) {
        return;
    }
    /* Диезы, без бемолей: на экране 14 знаков в строке, а показывать одну и ту
     * же клавишу двумя именами всё равно нечем — тональности инструмент не
     * знает. */
    static const char *NAMES[12] = {
        "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
    };

    if (!(hz > 0.0f) || !(a4_hz > 0.0f)) {
        snprintf(out, n, "--");
        return;
    }

    /* Номер MIDI: A4 это 69. Округление к ближайшему полутону — см. заголовок. */
    const int midi = (int)lrintf(69.0f + 12.0f * log2f(hz / a4_hz));
    if (midi < 0 || midi > 127) {
        snprintf(out, n, "--");
        return;
    }

    /* Октава по стандарту scientific pitch: C4 — до первой октавы, и там же
     * начинается октава номер 4, поэтому деление идёт от C, а не от A. */
    snprintf(out, n, "%s%d", NAMES[midi % 12], midi / 12 - 1);
}
