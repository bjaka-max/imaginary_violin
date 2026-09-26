#include "iv_test.h"
#include "iv_dsp.h"

/* Сглаживатель должен за одну постоянную времени пройти ~63 % пути к цели —
 * это и есть определение tau, на него опираются настройки времён в синтезе. */
static void test_smooth_time_constant(void)
{
    const float sr = 48000.0f, tau = 0.010f;
    iv_smooth_t s;
    iv_smooth_init(&s, sr, tau, 0.0f);

    const int n = (int)(tau * sr);
    float y = 0.0f;
    for (int i = 0; i < n; ++i) {
        y = iv_smooth_process(&s, 1.0f);
    }
    IV_CHECK_NEAR(y, 0.632f, 0.01f);
}

/* Сходимость к цели — с поправкой на предел float32.
 *
 * Вплотную к 440 Гц фильтр не дойдёт: шаг d*(1-a) становится меньше половины
 * ulp(440) и округляется обратно. Проверяем то, что важно на самом деле —
 * что остаточная ошибка неслышима. Порог 0.1 цента взят с большим запасом:
 * различимо примерно от 5 центов. */
static void test_smooth_converges(void)
{
    iv_smooth_t s;
    iv_smooth_init(&s, 48000.0f, 0.005f, 0.0f);
    float y = 0.0f;
    for (int i = 0; i < 48000; ++i) {
        y = iv_smooth_process(&s, 440.0f);
    }
    const double cents = 1200.0 * log2((double)y / 440.0);
    IV_CHECK(fabs(cents) < 0.1);
}

/* Затухание к нулю обязано быть точным: на нём строится тишина при пропаже
 * связи, и «почти ноль» здесь не годится. */
static void test_smooth_decays_to_exact_zero(void)
{
    iv_smooth_t s;
    iv_smooth_init(&s, 48000.0f, 0.005f, 1.0f);
    float y = 1.0f;
    for (int i = 0; i < 48000; ++i) {
        y = iv_smooth_process(&s, 0.0f);
    }
    IV_CHECK(y == 0.0f);
}

/* Затухание к нулю не должно оставлять денормалы: они стоят десятков тактов
 * на операцию и способны сорвать бюджет аудиоблока. */
static void test_smooth_flushes_denormals(void)
{
    iv_smooth_t s;
    iv_smooth_init(&s, 48000.0f, 0.001f, 1.0f);
    float y = 1.0f;
    for (int i = 0; i < 48000; ++i) {
        y = iv_smooth_process(&s, 0.0f);
    }
    IV_CHECK(y == 0.0f);
}

/* tau <= 0 означает «без сглаживания»: значение принимается сразу. */
static void test_smooth_zero_tau(void)
{
    iv_smooth_t s;
    iv_smooth_init(&s, 48000.0f, 0.0f, 0.0f);
    IV_CHECK_NEAR(iv_smooth_process(&s, 5.0f), 5.0f, 1e-6);
}

static void test_note_to_hz(void)
{
    IV_CHECK_NEAR(iv_note_to_hz(69.0f, 440.0f), 440.0f, 0.01);  /* A4 */
    IV_CHECK_NEAR(iv_note_to_hz(81.0f, 440.0f), 880.0f, 0.01);  /* октава выше */
    IV_CHECK_NEAR(iv_note_to_hz(55.0f, 440.0f), 196.0f, 0.5);   /* G3, нижняя струна */
    IV_CHECK_NEAR(iv_note_to_hz(69.0f, 442.0f), 442.0f, 0.01);  /* другой строй */
}

static void test_clamp_lerp(void)
{
    IV_CHECK_NEAR(iv_clampf(2.0f, -1.0f, 1.0f), 1.0f, 1e-6);
    IV_CHECK_NEAR(iv_clampf(-2.0f, -1.0f, 1.0f), -1.0f, 1e-6);
    IV_CHECK_NEAR(iv_clampf(0.5f, -1.0f, 1.0f), 0.5f, 1e-6);
    IV_CHECK_NEAR(iv_lerpf(0.0f, 10.0f, 0.25f), 2.5f, 1e-6);
}

/* ---- Линия задержки ------------------------------------------------------- */

/* Целая задержка: сэмпл выходит ровно через delay тактов, ни раньше, ни позже.
 * Ошибка на такт здесь стоит десятков центов расстройки волновода. */
static void test_delay_integer(void)
{
    iv_delay_t d;
    iv_delay_init(&d, 5.0f);

    IV_CHECK(iv_delay_tick(&d, 1.0f) == 0.0f); /* линия пустая */
    for (int i = 0; i < 4; ++i) {
        IV_CHECK(iv_delay_tick(&d, 0.0f) == 0.0f);
    }
    IV_CHECK(iv_delay_tick(&d, 0.0f) == 1.0f); /* пятый такт после записи */
}

/* Дробная задержка: линейная интерполяция между соседними отсчётами. На низкой
 * частоте она почти точна — именно на этом стоит настройка струны, и именно
 * поэтому её проверяем на синусе, а не на импульсе. */
static void test_delay_fractional(void)
{
    const double sr = 48000.0, hz = 100.0, delay = 10.5;
    iv_delay_t d;
    iv_delay_init(&d, (float)delay);

    double worst = 0.0;
    for (int i = 0; i < 4800; ++i) {
        const double y = iv_delay_tick(&d, (float)sin(2.0 * M_PI * hz * i / sr));
        if (i > 100) {
            const double want = sin(2.0 * M_PI * hz * (i - delay) / sr);
            const double err = fabs(y - want);
            if (err > worst) {
                worst = err;
            }
        }
    }
    IV_CHECK(worst < 0.01);
}

/* Задержка зажимается по ёмкости буфера: за неё линия читала бы чужую память,
 * а вызывающий (волновод на низкой ноте) об этом знать не обязан. */
static void test_delay_clamped(void)
{
    iv_delay_t d;
    iv_delay_init(&d, 1e9f);
    IV_CHECK(d.delay <= (float)(IV_DELAY_MAX - 2));

    iv_delay_set(&d, -100.0f);
    IV_CHECK(d.delay >= 1.0f);

    for (int i = 0; i < 1000; ++i) {
        const float y = iv_delay_tick(&d, 1.0f);
        IV_CHECK(y == y); /* не NaN */
    }
}

/* ---- Биквад --------------------------------------------------------------- */

/* Амплитуда синуса частоты hz на выходе фильтра. */
static double biquad_gain(iv_biquad_t *f, double sr, double hz)
{
    double peak = 0.0;
    for (int i = 0; i < 20000; ++i) {
        const double y = iv_biquad_process(f, (float)sin(2.0 * M_PI * hz * i / sr));
        if (i > 5000 && fabs(y) > peak) {
            peak = fabs(y);
        }
    }
    return peak;
}

/* Резонанс корпуса поднимает свою частоту на заданные децибелы и почти не
 * трогает соседние: если перепутать местами коэффициенты числителя и
 * знаменателя, подъём превратится в провал, и это поймается здесь. */
static void test_biquad_peak(void)
{
    iv_biquad_t f;
    iv_biquad_peak(&f, 48000.0f, 500.0f, 8.0f, 9.0f);
    IV_CHECK_NEAR(20.0 * log10(biquad_gain(&f, 48000.0, 500.0)), 9.0, 0.5);

    iv_biquad_reset(&f);
    IV_CHECK_NEAR(20.0 * log10(biquad_gain(&f, 48000.0, 4000.0)), 0.0, 0.5);
}

/* Фильтр верхних частот корпуса: постоянную составляющую волновода он обязан
 * убирать полностью, а полосу инструмента пропускать без потерь. */
static void test_biquad_highpass(void)
{
    iv_biquad_t f;
    iv_biquad_highpass(&f, 48000.0f, 180.0f, 0.7f);

    float dc = 0.0f;
    for (int i = 0; i < 20000; ++i) {
        dc = iv_biquad_process(&f, 1.0f);
    }
    IV_CHECK(fabs(dc) < 1e-3);

    iv_biquad_reset(&f);
    IV_CHECK_NEAR(20.0 * log10(biquad_gain(&f, 48000.0, 2000.0)), 0.0, 0.5);
}

/* Фазовая задержка однополюсного фильтра. На нуле она равна p/(1-p), с ростом
 * частоты падает — и то, и другое входит в настройку струны напрямую. */
static void test_onepole_phase_delay(void)
{
    const float p = 0.66f;
    const double at_dc = p / (1.0 - p);

    IV_CHECK_NEAR(iv_onepole_phase_delay(p, 1.0f, 48000.0f), at_dc, at_dc * 0.01);
    IV_CHECK(iv_onepole_phase_delay(p, 440.0f, 48000.0f) < at_dc);
    IV_CHECK(iv_onepole_phase_delay(p, 4000.0f, 48000.0f)
             < iv_onepole_phase_delay(p, 440.0f, 48000.0f));
}

int main(void)
{
    test_smooth_time_constant();
    test_smooth_converges();
    test_smooth_decays_to_exact_zero();
    test_smooth_flushes_denormals();
    test_smooth_zero_tau();
    test_note_to_hz();
    test_clamp_lerp();
    test_delay_integer();
    test_delay_fractional();
    test_delay_clamped();
    test_biquad_peak();
    test_biquad_highpass();
    test_onepole_phase_delay();
    IV_TEST_END();
}
