#include "iv_string.h"
#include <math.h>

/* Потери за оборот. Отсюда длина затухания: амплитуда падает в 0.99 раз за
 * период, то есть на A4 постоянная времени около 0.23 с. Больше — струна
 * звенит после снятия смычка неправдоподобно долго и мешает следующей ноте,
 * меньше — нота не набирает тела и звучит как удар. */
#define LOOP_GAIN 0.99f

/* Полюс фильтра отражения. Подставка отдаёт корпусу тем больше, чем выше
 * частота: на найквисте петля теряет уже 80 % амплитуды за оборот. Это и есть
 * причина, по которой у смычковой ноты верх живёт короче низа.
 *
 * Границы — рабочий диапазон яркости. Ниже POLE_BRIGHT петля почти не
 * фильтрует, и в звуке вылезает алиасинг излома; выше POLE_DARK нота глохнет
 * до глухого стука. LOOP_POLE — середина, с которой всё начинается. */
#define POLE_BRIGHT 0.45f
#define POLE_DARK   0.75f
#define LOOP_POLE   0.66f

/* Лишняя задержка петли, не считая линий и фильтра: волновод читает выход
 * каждой линии от предыдущего такта (`last`), и это ровно один сэмпл на линию.
 * Не учесть — струна стабильно завышает: два сэмпла на A4 это почти тридцать
 * центов. */
#define LOOP_OVERHEAD 2.0f

/* Поправка настройки, замеренная на модели: без неё струна стабильно
 * завышает на 0.21 сэмпла петли — от 1.5 центов на G3 до 7 на верхах.
 * Считать её нечем: это сумма мелочей — линейная интерполяция задержки
 * укорачивает петлю на высоких частотах, и смычок своей нелинейностью тянет
 * высоту вверх (на настоящей скрипке он делает то же самое). Величина по
 * замерам постоянна в сэмплах во всём диапазоне, поэтому и вычитается
 * константой; проверяется тестом на строй. */
#define LOOP_TUNE 0.21f

/* Сглаживание длины петли. Меняется каждый сэмпл, поэтому глиссандо и вибрато
 * идут без ступенек; 10 мс — компромисс между слитностью и «резиновой» атакой
 * при смене ноты. */
#define LEN_TAU 0.010f

/* За сколько смычок снимается со струны. Меньше — щелчок на снятии, больше —
 * струна дольше остаётся защемлённой и дольше стоит процессорного времени. */
#define CONTACT_TAU 0.008f

/* Асимметрия кривой трения. Строго симметричная кривая даёт неустойчивое
 * равновесие в нуле: струна может застрять в прилипании и не сорваться вовсе.
 * Малое смещение делает срыв определённым — и оно же есть у настоящей канифоли. */
#define BOW_OFFSET 0.001f

/* Нелинейность трения смычка.
 *
 * Возвращает долю относительной скорости, которую смычок передаёт струне.
 * Пока разность скоростей мала, струна прилипла к смычку и едет с ним
 * (коэффициент 1); чем сильнее она пытается вырваться, тем быстрее трение
 * падает — и на срыве струна уходит почти свободно. Гипербола четвёртой
 * степени — стандартная аппроксимация этой кривой (STK): у неё есть и плоская
 * вершина прилипания, и быстрый спад, и она стоит трёх умножений и деления.
 *
 * slope — крутизна: нажим сильнее, значит область прилипания шире, значит
 * slope меньше. */
static float bow_friction(float dv, float slope)
{
    const float s = fabsf((dv + BOW_OFFSET) * slope) + 0.75f;
    const float s2 = s * s;
    const float r = 1.0f / (s2 * s2);
    return r > 1.0f ? 1.0f : r;
}

/* Белый шум, xorshift32: три сдвига и три xor — дешевле любой библиотечной
 * функции, а спектральных требований к шуму смычка нет. */
static float white(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return (float)(int32_t)x * (1.0f / 2147483648.0f);
}

/* Пересчёт потерь петли: собственные плюс добавочное затухание. */
static void update_loss(iv_string_t *s)
{
    s->gain = LOOP_GAIN;
    if (s->damp_tau > 0.0f) {
        /* Период ноты, делённый на постоянную времени: множитель на оборот. */
        s->gain *= expf(-1.0f / (s->freq_hz * s->damp_tau));
    }
}

/* Фильтр отражения на подставке: однополюсный ФНЧ с потерями. */
static float reflection(iv_string_t *s, float x)
{
    s->z = (1.0f - s->pole) * x + s->pole * s->z;
    if (s->z > -1e-20f && s->z < 1e-20f) {
        s->z = 0.0f; /* денормалы на хвосте затухания стоят дороже самой ноты */
    }
    return s->gain * s->z;
}

void iv_string_init(iv_string_t *s, float sample_rate, float freq_hz)
{
    s->sample_rate = sample_rate;
    s->pole = LOOP_POLE;
    s->gain = LOOP_GAIN;
    s->damp_tau = 0.0f;
    s->z = 0.0f;
    s->peak = 0.0f;
    s->noise = 0.0f;
    /* Смычок на струне: одиночная струна, которую просто тикают со скоростью,
     * должна звучать. Синтезатор всё равно задаёт сцепление каждый блок. */
    s->contact_target = 1.0f;
    iv_smooth_init(&s->contact, sample_rate, CONTACT_TAU, 1.0f);
    /* Затравка не ноль: xorshift из нуля никогда не выйдет. */
    s->rng = 0x9e3779b9u;

    s->quiet = true;
    iv_delay_init(&s->to_nut, 1.0f);
    iv_delay_init(&s->to_bridge, 1.0f);

    iv_string_set_freq(s, freq_hz);
    /* Сглаживатель сразу на цели: иначе первая нота приезжала бы глиссандо
     * с произвольной высоты. */
    iv_smooth_init(&s->len, sample_rate, LEN_TAU, s->len_target);
}

void iv_string_set_freq(iv_string_t *s, float hz)
{
    hz = iv_clampf(hz, IV_STRING_FREQ_MIN, IV_STRING_FREQ_MAX);
    s->freq_hz = hz;

    /* Период ноты — это полный оборот по петле, и в него входит всё, что в
     * петле стоит: обе линии, фазовая задержка фильтра отражения и такты,
     * которые волновод тратит на чтение прошлых значений. */
    const float period = s->sample_rate / hz;
    const float phase  = iv_onepole_phase_delay(s->pole, hz, s->sample_rate);
    const float loop   = period - LOOP_OVERHEAD + LOOP_TUNE - phase;

    /* Верхний зажим — по ёмкости длинной линии: она держит (1 - beta) петли. */
    const float max_loop = (float)(IV_DELAY_MAX - 2) / (1.0f - IV_BOW_POSITION);
    s->len_target = iv_clampf(loop, 4.0f, max_loop);

    /* Делить петлю в отношении beta нельзя: лишняя задержка распределена по
     * ней неравномерно. Со стороны порожка это один такт, со стороны подставки
     * — такт плюс фазовая задержка фильтра отражения, то есть почти три сэмпла
     * на плече, которое само длиной в тринадцать. Разделишь пропорционально —
     * и смычок фактически стоит не там, где написано: на A4 beta выходит 0.150
     * вместо 0.127, и провал в спектре, которым точка смычка и слышна,
     * съезжает с восьмой гармоники на седьмую.
     *
     * Поэтому длина плеча к подставке считается от периода и уменьшается на то,
     * что и так стоит в этом плече; остаток целиком уходит второму плечу.
     * Сумма плеч при этом не меняется, а значит строй от этой поправки не
     * зависит вовсе. */
    s->bridge_bias = IV_BOW_POSITION * (LOOP_OVERHEAD - LOOP_TUNE + phase)
                   - 1.0f - phase;

    s->len_settled = false;
    update_loss(s); /* затухание задано временем, а считается на период */
}

void iv_string_snap_freq(iv_string_t *s, float hz)
{
    iv_string_set_freq(s, hz);
    iv_smooth_reset(&s->len, s->len_target);
    s->len_settled = false; /* плечи пересчитать хотя бы раз */
}

void iv_string_set_brightness(iv_string_t *s, float brightness)
{
    const float pole = POLE_DARK
                     + (POLE_BRIGHT - POLE_DARK) * iv_clampf(brightness, 0.0f, 1.0f);
    if (pole == s->pole) {
        return; /* строй пересчитывать незачем */
    }
    s->pole = pole;
    iv_string_set_freq(s, s->freq_hz); /* полюс входит в длину петли */
}

void iv_string_set_damping(iv_string_t *s, float tau_s)
{
    s->damp_tau = tau_s;
    update_loss(s);
}

void iv_string_set_contact(iv_string_t *s, float target)
{
    s->contact_target = target;
    if (target > s->contact.y) {
        iv_smooth_reset(&s->contact, target); /* смычок опускается мгновенно */
    }
}

void iv_string_set_bow_noise(iv_string_t *s, float amp)
{
    s->noise = amp;
}

/* touched на каждом месте вызова — литерал, поэтому ветка сворачивается на
 * этапе компиляции: у догорающей струны в цикле не остаётся ни сглаживания
 * сцепления, ни кривой трения. */
static inline float tick_one(iv_string_t *s, float bow_vel, float bow_slope,
                             bool touched)
{
    /* Длина едет к цели посэмплово: скачок сдвинул бы точку чтения линии
     * задержки, а это щелчок, а не глиссандо. Когда доехала — плечи не трогаем
     * вовсе: у струны, которая просто догорает, высота не меняется, а это
     * треть её стоимости. */
    if (!s->len_settled) {
        const float len = iv_smooth_process(&s->len, s->len_target);
        if (len > s->len_target - 1e-3f && len < s->len_target + 1e-3f) {
            s->len_settled = true;
        }

        /* На самом верху диапазона плечо к подставке короче сэмпла — тогда оно
         * упирается в минимум линии, а порожковое плечо укорачивается на
         * столько же. Точка смычка там уезжает, строй — нет. */
        float to_bridge = IV_BOW_POSITION * len + s->bridge_bias;
        if (to_bridge < 1.0f) {
            to_bridge = 1.0f;
        }
        iv_delay_set(&s->to_bridge, to_bridge);
        iv_delay_set(&s->to_nut,    len - to_bridge);
    }

    /* Обе волны приходят в точку смычка с обратным знаком: и порожек, и
     * подставка — это закреплённые концы, они отражают с инверсией. */
    const float from_bridge = -reflection(s, s->to_bridge.last);
    const float from_nut    = -s->to_nut.last;

    /* Вся физика инструмента — в этих четырёх строках: насколько струна
     * отстаёт от смычка и сколько трения он успевает ей передать.
     * Со снятым смычком не считается вовсе — трению взяться неоткуда, и это
     * же снимает с процессора три четверти работы, когда струны догорают. */
    float inject = 0.0f;
    if (touched) {
        const float c = iv_smooth_process(&s->contact, s->contact_target);
        if (c > 1e-4f) {
            const float v  = bow_vel + s->noise * white(&s->rng);
            const float dv = v - (from_bridge + from_nut);
            inject = c * dv * bow_friction(dv, bow_slope);
        } else {
            /* Ниже этого сцепление уже ничего не значит (−80 дБ связи), а
             * ровный ноль пускает струну по дешёвому пути. */
            s->contact.y = 0.0f;
        }
    }

    iv_delay_tick(&s->to_nut, from_bridge + inject);
    const float out = iv_delay_tick(&s->to_bridge, from_nut + inject);

    const float mag = fabsf(out);
    if (mag > s->peak) {
        s->peak = mag;
    }
    return out;
}

void iv_string_render(iv_string_t *s, const float *bow_vel, const float *slope,
                      float *out, size_t n)
{
    /* restrict здесь не украшение: без него компилятор обязан считать, что
     * запись в out может задеть состояние струны, и перечитывает его из памяти
     * каждый сэмпл — ровно то, ради чего блок и заведён. */
    float *restrict o = out;
    const float *restrict v = bow_vel;
    const float *restrict k = slope;

    if (v != NULL) {
        for (size_t i = 0; i < n; ++i) {
            o[i] += tick_one(s, v[i], k[i], true);
        }
    } else if (s->contact.y != 0.0f) {
        /* Смычок только что ушёл: сцепление ещё гаснет. */
        for (size_t i = 0; i < n; ++i) {
            o[i] += tick_one(s, 0.0f, k[i], true);
        }
    } else {
        /* Струна просто догорает: ни смычка, ни трения. */
        for (size_t i = 0; i < n; ++i) {
            o[i] += tick_one(s, 0.0f, 0.0f, false);
        }
    }
}

float iv_string_tick(iv_string_t *s, float bow_vel, float bow_slope)
{
    return tick_one(s, bow_vel, bow_slope, true);
}

void iv_string_clear(iv_string_t *s)
{
    s->quiet = true;
    s->len_settled = false;
    iv_smooth_reset(&s->contact, s->contact_target);
    iv_delay_clear(&s->to_nut);
    iv_delay_clear(&s->to_bridge);
    s->z = 0.0f;
    s->peak = 0.0f;
    iv_smooth_reset(&s->len, s->len_target);
}
