/* Синтезатор целиком: струны, корпус, выбор струны, тишина.
 *
 * Признаки самой смычковой модели проверяются отдельно, в test_string.c;
 * здесь — то, что видит остальная прошивка: строй, границы сигнала, поведение
 * на смене ноты и штриха и, отдельно, точность тишины. */
#include "iv_test.h"
#include "iv_synth.h"
#include "iv_analysis.h"
#include <string.h>

#define SR 48000.0

static iv_synth_t g_syn;
static float      g_buf[48000];

/* Струна, на которой берётся нота: самая высокая из открытых, которой она
 * доступна. В инструменте это решает слой управления по углу смычка, здесь —
 * та же логика в одну строку, чтобы тесты играли ноты на правдоподобных
 * струнах, а не G3 на верхах. */
static uint8_t string_for(float hz)
{
    uint8_t k = 0;
    while (k + 1 < IV_SYNTH_VOICES && hz >= iv_open_hz[k + 1]) {
        k++;
    }
    return k;
}

/* Смычок целиком на одной струне: доля 1 у неё, 0 у остальных. */
static void set_on(uint8_t string, float hz, float vel, float force)
{
    iv_synth_params_t p = { .bow_velocity = vel, .bow_force = force,
                            .brightness = 0.5f };
    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        p.bow_share[k] = (k == (int)string) ? 1.0f : 0.0f;
        p.freq_hz[k]   = iv_open_hz[k];
    }
    p.freq_hz[string] = hz;
    iv_synth_set_params(&g_syn, &p);
}

/* Смычок на границе струн lo и lo+1: доли задаются, ноты берутся открытыми —
 * это двойная нота на открытых струнах, то есть чистая квинта. */
static void set_pair(int lo, float share_lo, float vel, float force)
{
    iv_synth_params_t p = { .bow_velocity = vel, .bow_force = force,
                            .brightness = 0.5f };
    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        p.bow_share[k] = 0.0f;
        p.freq_hz[k]   = iv_open_hz[k];
    }
    p.bow_share[lo]     = share_lo;
    p.bow_share[lo + 1] = 1.0f - share_lo;
    iv_synth_set_params(&g_syn, &p);
}

static void set(float hz, float vel, float force)
{
    set_on(string_for(hz), hz, vel, force);
}

/* Ведёт смычок и отдаёт установившийся тон — вторую половину секунды. */
static const float *steady(float hz, float vel, float force)
{
    iv_synth_init(&g_syn, (float)SR);
    set(hz, vel, force);
    iv_synth_render(&g_syn, g_buf, 48000);
    return g_buf + 24000;
}

/* Неподвижный смычок = полная тишина. Это не косметика: при пропаже связи
 * гриф гасит звук именно обнулением скорости, и тишина обязана быть точной. */
static void test_silent_without_bow(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set(440.0f, 0.0f, 0.0f);
    iv_synth_render(&g_syn, g_buf, 48000);
    IV_CHECK(iv_an_peak(g_buf, 48000) == 0.0);
}

/* Строй на всех четырёх струнах. Порог 5 центов — примерно порог различимости;
 * настройка волновода легко уезжает на десятки центов, если ошибиться в
 * слагаемых длины петли. */
static void test_renders_requested_pitch(void)
{
    const float notes[] = { 220.0f, 293.7f, 440.0f, 659.3f, 880.0f };
    for (size_t i = 0; i < sizeof(notes) / sizeof(*notes); ++i) {
        const float *w = steady(notes[i], 0.9f, 0.5f);
        const double f0 = iv_an_f0(w, 24000, SR, 80.0, 2000.0);
        const double cents = 1200.0 * log2(f0 / notes[i]);
        IV_CHECK(fabs(cents) < 5.0);
        IV_CHECK(iv_an_rms(w, 24000) > 0.02); /* и это слышно, а не шуршит */
        if (fabs(cents) >= 5.0) {
            printf("  %.1f Гц -> %.1f Гц (%+.1f центов)\n", (double)notes[i], f0, cents);
        }
    }
}

/* Направление штриха само по себе громкости не меняет: вверх и вниз звучат
 * одинаково. Разница между ними — только в момент разворота. */
static void test_bow_direction_symmetric(void)
{
    const double up = iv_an_rms(steady(440.0f, 0.7f, 0.5f), 24000);
    const double down = iv_an_rms(steady(440.0f, -0.7f, 0.5f), 24000);
    IV_CHECK_NEAR(up, down, up * 0.1);
}

/* Нажим смычка обязан менять тембр, а не только громкость.
 *
 * Меряем наклоном спектра, а не центроидом: при лёгком нажиме модель добавляет
 * шума, центроид от этого растёт, и по нему выходило бы, что лёгкий нажим
 * ярче тяжёлого. Наклон смотрит только на гармоники и показывает то, что
 * слышно как тембр. */
static void test_force_changes_timbre(void)
{
    const float *w = steady(440.0f, 0.9f, 0.25f);
    double f0 = iv_an_f0(w, 24000, SR, 80.0, 2000.0);
    const double light = iv_an_harmonic_tilt(w, 24000, SR, f0);

    w = steady(440.0f, 0.9f, 1.00f);
    f0 = iv_an_f0(w, 24000, SR, 80.0, 2000.0);
    const double heavy = iv_an_harmonic_tilt(w, 24000, SR, f0);

    IV_CHECK(heavy > light + 2.0);
}

/* Выход обязан оставаться в -1..1 и не содержать NaN даже при абсурдных
 * входных данных: в аудиопоток попадает то, что придёт по радио. */
static void test_output_bounded(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set(20000.0f, 50.0f, 99.0f);
    iv_synth_render(&g_syn, g_buf, 24000);
    set(-5.0f, -50.0f, -3.0f);
    iv_synth_render(&g_syn, g_buf + 24000, 24000);

    for (size_t i = 0; i < 48000; ++i) {
        IV_CHECK(g_buf[i] == g_buf[i]);                   /* не NaN */
        IV_CHECK(g_buf[i] >= -1.0f && g_buf[i] <= 1.0f);
    }
}

/* Высота внутри одной струны меняется постепенно: это глиссандо, а не скачок.
 * 440 и 620 Гц лежат на одной струне (A), поэтому переезд идёт длиной линии
 * задержки — её и проверяем, замеряя высоту сразу после прыжка. */
static void test_pitch_glides(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set(440.0f, 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 24000);

    set(620.0f, 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 768); /* 16 мс: полтора времени сглаживания */

    const double f0 = iv_an_f0(g_buf, 768, SR, 300.0, 900.0);
    IV_CHECK(f0 > 460.0 && f0 < 600.0);
    if (!(f0 > 460.0 && f0 < 600.0)) {
        printf("  через 16 мс после прыжка 440->620: %.1f Гц\n", f0);
    }
}

/* Синтез играет ту струну, которую ему назвали, и не выбирает сам: разбор
 * жестов — дело слоя управления (см. iv_control). Проверяем, что параметр
 * действительно доходит: одна и та же нота на разных струнах звучит по-разному,
 * потому что струны разной длины и по-разному ярки. */
static void test_plays_the_named_string(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set_on(1, 440.0f, 0.9f, 0.5f); /* ля на струне D */
    iv_synth_render(&g_syn, g_buf, 24000);
    IV_CHECK(g_syn.selected == 1);

    set_on(2, 440.0f, 0.9f, 0.5f); /* та же нота на открытой A */
    iv_synth_render(&g_syn, g_buf, 24000);
    IV_CHECK(g_syn.selected == 2);

    /* И обе звучат: брошенная струна догорает, новая говорит. */
    IV_CHECK(iv_an_rms(g_buf + 12000, 12000) > 0.02);
}

/* Двойная нота. Смычок стоит на границе A и E, и в звуке обязаны быть обе:
 * ради этого доли и заведены. Проверяется по спектру — обе основные частоты
 * на месте, — а не по громкости: громкость тут ничего не различает.
 *
 * И обратное не менее важно: когда смычок целиком на A, струна E звучать не
 * должна вовсе. Иначе «двойная нота» окажется не раскладкой, а протечкой. */
static void test_double_stop_sounds_both(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set_pair(2, 0.5f, 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 48000);

    const double a_both = iv_an_mag(g_buf + 24000, 24000, SR, iv_open_hz[2]);
    const double e_both = iv_an_mag(g_buf + 24000, 24000, SR, iv_open_hz[3]);
    IV_CHECK(a_both > 0.0 && e_both > 0.0);
    /* Ни одна не тонет: разница меньше 20 дБ — это две ноты, а не нота
     * с призвуком. */
    IV_CHECK(fabs(20.0 * log10(e_both / a_both)) < 20.0);

    /* Тот же смычок целиком на A: E молчит. */
    iv_synth_init(&g_syn, (float)SR);
    set_on(2, iv_open_hz[2], 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 48000);
    const double e_alone = iv_an_mag(g_buf + 24000, 24000, SR, iv_open_hz[3]);
    IV_CHECK(e_alone < e_both * 0.1);
}

/* Двойная нота не имеет права быть громче одинарной вдвое: доли делят смычок,
 * а не размножают его. Иначе переход между струнами слышался бы наплывом. */
static void test_double_stop_keeps_level(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set_on(2, iv_open_hz[2], 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 48000);
    const double alone = iv_an_rms(g_buf + 24000, 24000);

    iv_synth_init(&g_syn, (float)SR);
    set_pair(2, 0.5f, 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 48000);
    const double both = iv_an_rms(g_buf + 24000, 24000);

    const double db = 20.0 * log10(both / alone);
    IV_CHECK(db < 6.0);   /* не сложились в полный рост */
    IV_CHECK(db > -12.0); /* и не провалились в шёпот */
}

/* Яркость — отдельная ось, и она обязана менять тембр, не трогая высоту:
 * полюс фильтра отражения входит в длину петли, и если это не учесть,
 * инструмент будет фальшивить при повороте грифа. */
static void test_brightness_keeps_pitch(void)
{
    double f_dark = 0.0, f_bright = 0.0;
    double tilt_dark = 0.0, tilt_bright = 0.0;

    for (int bright = 0; bright < 2; ++bright) {
        iv_synth_init(&g_syn, (float)SR);
        iv_synth_params_t p = { .bow_velocity = 0.9f, .bow_force = 0.5f,
                                .brightness = bright ? 1.0f : 0.0f };
        for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
            p.bow_share[k] = (k == 2) ? 1.0f : 0.0f;
            p.freq_hz[k]   = iv_open_hz[k];
        }
        p.freq_hz[2] = 440.0f;
        iv_synth_set_params(&g_syn, &p);
        iv_synth_render(&g_syn, g_buf, 48000);

        const double f0 = iv_an_f0(g_buf + 24000, 24000, SR, 80.0, 2000.0);
        const double tilt = iv_an_harmonic_tilt(g_buf + 24000, 24000, SR, f0);
        if (bright) {
            f_bright = f0;
            tilt_bright = tilt;
        } else {
            f_dark = f0;
            tilt_dark = tilt;
        }
    }

    IV_CHECK(tilt_bright > tilt_dark + 3.0); /* тембр поехал заметно */
    IV_CHECK(fabs(1200.0 * log2(f_bright / 440.0)) < 5.0); /* а высота — нет */
    IV_CHECK(fabs(1200.0 * log2(f_dark / 440.0)) < 5.0);
}

/* Смена струны не должна щёлкать: новая струна начинает со своей ноты, а
 * старая догорает. Щелчок — это разрыв в сигнале, поэтому и меряем самый
 * крутой скачок между соседними сэмплами, сравнивая с обычным для этого звука. */
static void test_string_change_without_click(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set(440.0f, 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 24000);

    double normal = 0.0;
    for (size_t i = 23001; i < 24000; ++i) {
        const double d = fabs((double)g_buf[i] - g_buf[i - 1]);
        if (d > normal) normal = d;
    }

    set_on(1, 300.0f, 0.9f, 0.5f); /* прыжок через границу струн */
    iv_synth_render(&g_syn, g_buf, 4800);

    double worst = 0.0;
    for (size_t i = 1; i < 4800; ++i) {
        const double d = fabs((double)g_buf[i] - g_buf[i - 1]);
        if (d > worst) worst = d;
    }
    IV_CHECK(worst < normal * 2.0);
}

/* Разворот смычка слышен: звук на мгновение проваливается и набирается заново.
 * Это и есть атака на смене штриха — без неё легато и деташе звучат одинаково. */
static void test_bow_reversal_articulates(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set(440.0f, 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 24000);
    const double before = iv_an_rms(g_buf + 23000, 1000);

    set(440.0f, -0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 4800); /* 100 мс после разворота */

    double dip = before;
    for (size_t i = 0; i + 240 <= 1440; i += 240) { /* первые 30 мс */
        const double r = iv_an_rms(g_buf + i, 240);
        if (r < dip) dip = r;
    }
    IV_CHECK(dip < before * 0.7);

    const double after = iv_an_rms(g_buf + 3800, 1000);
    IV_CHECK(after > before * 0.7); /* и звук возвращается */
}

/* Снятие смычка обязано приводить к точной тишине, а не к «почти нулю»:
 * на этом держится гашение звука при пропаже смычка. */
static void test_release_to_silence(void)
{
    iv_synth_init(&g_syn, (float)SR);
    set(440.0f, 0.9f, 0.5f);
    iv_synth_render(&g_syn, g_buf, 24000);

    set(440.0f, 0.0f, 0.5f);
    /* Две секунды: струна гаснет за десятые доли, но полный ноль наступает
     * только когда и струны, и корпус проваливаются ниже -100 дБ. */
    for (int i = 0; i < 40; ++i) {
        iv_synth_render(&g_syn, g_buf, 2400);
        if (i == 0) {
            IV_CHECK(iv_an_peak(g_buf, 2400) > 0.0); /* затухание, а не обрыв */
        }
    }
    IV_CHECK(iv_an_peak(g_buf, 2400) == 0.0);
}

int main(void)
{
    test_silent_without_bow();
    test_renders_requested_pitch();
    test_bow_direction_symmetric();
    test_force_changes_timbre();
    test_output_bounded();
    test_pitch_glides();
    test_plays_the_named_string();
    test_double_stop_sounds_both();
    test_double_stop_keeps_level();
    test_brightness_keeps_pitch();
    test_string_change_without_click();
    test_bow_reversal_articulates();
    test_release_to_silence();
    IV_TEST_END();
}
