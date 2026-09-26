/* Смычковая струна: сверка с эталоном.
 *
 * Эталон здесь не чужая программа, а физика, ради которой волновод и взят.
 * Установившееся движение смычковой струны — движение Гельмгольца: по струне
 * бегает один излом, сила на подставке пилообразна, и спектр у неё падает как
 * 1/n, то есть на 6 дБ при удвоении номера гармоники. Это проверяемо числами,
 * и именно этим модель отличается от «синтезаторной подкладки», которая может
 * звучать похоже, но устроена иначе.
 *
 * Модель STK `Bowed`, взятая за образец структуры, проверяет ровно то же
 * самое — сравнивать с ней побитово нечего и незачем: у неё свои константы
 * потерь и своя настройка. Сравнивать надо с законом, который обе модели
 * обязаны воспроизводить. */
#include "iv_test.h"
#include "iv_string.h"
#include "iv_analysis.h"
#include <string.h>

#define SR 48000.0
#define N  ((size_t)48000)

static iv_string_t g_str;
static float       g_buf[N];

/* Ведём смычок секунду и отдаём вторую половину — установившийся тон. */
static const float *bow(float hz, float vel, float slope, float noise)
{
    iv_string_init(&g_str, (float)SR, hz);
    iv_string_set_bow_noise(&g_str, noise);
    for (size_t i = 0; i < N; ++i) {
        g_buf[i] = iv_string_tick(&g_str, vel, slope);
    }
    return g_buf + N / 2;
}

static double harmonic_db(const float *w, double f0, int h)
{
    const double a = iv_an_mag(w, N / 2, SR, f0 * h);
    const double a1 = iv_an_mag(w, N / 2, SR, f0);
    return 20.0 * log10(a / (a1 + 1e-12) + 1e-12);
}

/* Доля энергии, не попавшая ни в одну гармонику: шум смычка и всё, что не
 * повторяется из периода в период. */
static double noise_fraction(const float *w, double f0)
{
    const double rms = iv_an_rms(w, N / 2);
    double harm = 0.0;
    for (int h = 1; h <= 24 && f0 * h < SR / 2; ++h) {
        const double a = iv_an_mag(w, N / 2, SR, f0 * h);
        harm += a * a / 2.0;
    }
    const double r = 1.0 - harm / (rms * rms + 1e-18);
    return r < 0.0 ? 0.0 : r;
}

/* Строй. Период волновода складывается из длин линий, фазовой задержки фильтра
 * отражения и тактов на чтение прошлых значений; ошибиться в любом слагаемом
 * значит стабильно фальшивить. Порог 5 центов — примерно порог различимости. */
static void test_tuning(void)
{
    const float notes[] = { 196.0f, 293.7f, 440.0f, 659.3f, 880.0f, 1318.5f };
    for (size_t i = 0; i < sizeof(notes) / sizeof(*notes); ++i) {
        const float *w = bow(notes[i], 0.25f, 1.65f, 0.0f);
        const double f0 = iv_an_f0(w, N / 2, SR, 80.0, 2000.0);
        const double cents = 1200.0 * log2(f0 / notes[i]);
        IV_CHECK(fabs(cents) < 5.0);
        if (fabs(cents) >= 5.0) {
            printf("  %.1f Гц -> %.1f Гц (%+.1f центов)\n", (double)notes[i], f0, cents);
        }
    }
}

/* Спектр Гельмгольца: 1/n. Допуск широкий не от неуверенности, а потому что
 * фильтр отражения заваливает верх — реальная струна делает то же самое, и
 * гармоники обязаны падать не медленнее пилы, но могут падать быстрее. */
static void test_helmholtz_spectrum(void)
{
    const float *w = bow(440.0f, 0.25f, 1.65f, 0.0f);
    const double f0 = iv_an_f0(w, N / 2, SR, 80.0, 2000.0);
    IV_CHECK(f0 > 0.0);

    const double ideal[] = { -6.0, -9.5, -12.0, -14.0 };
    for (int h = 2; h <= 5; ++h) {
        const double db = harmonic_db(w, f0, h);
        IV_CHECK_NEAR(db, ideal[h - 2], 2.0);
    }
}

/* Форма волны: один срыв за период, и он короткий.
 *
 * Так устроено движение Гельмгольца: по струне бегает один излом, и каждый раз,
 * проходя точку смычка, он срывает струну — ровно раз за период. Срыв длится
 * примерно IV_BOW_POSITION периода, то есть пока излом добегает до подставки и
 * обратно; всё остальное время струна прилипла к смычку и едет с ним, а сила на
 * подставке нарастает.
 *
 * Отсюда две проверки: за окно длиной в долю периода, равную точке смычка,
 * сигнал успевает провалиться почти на весь свой размах, и таких провалов
 * ровно один на период. Двойное проскальзывание, в которое модель срывается
 * при лёгком нажиме, дало бы полтора-два.
 *
 * Считать доли растущих и падающих сэмплов, как хочется поначалу, нельзя: на
 * пилу наложена мелкая рябь с периодом в ту же долю (это излом, отражающийся
 * от точки смычка), она настоящая, её видно и на измерениях живых скрипок, но
 * знак приращения она переворачивает десятки раз за период. */
static void check_helmholtz_slip(float hz)
{
    const float *w = bow(hz, 0.25f, 1.65f, 0.0f);
    const size_t n = N / 2;

    const double f0 = iv_an_f0(w, n, SR, 80.0, 2000.0);
    const size_t win = (size_t)(IV_BOW_POSITION * SR / f0 + 0.5);

    double lo = w[0], hi = w[0];
    for (size_t i = 0; i < n; ++i) {
        if (w[i] < lo) lo = w[i];
        if (w[i] > hi) hi = w[i];
    }

    double deepest = 0.0;
    for (size_t i = win; i < n; ++i) {
        const double drop = (double)w[i - win] - w[i];
        if (drop > deepest) {
            deepest = drop;
        }
    }

    /* Считаем срывы: подряд идущие сэмплы глубже половины самого глубокого
     * провала — это один срыв, а не несколько. */
    int slips = 0, inside = 0;
    for (size_t i = win; i < n; ++i) {
        if ((double)w[i - win] - w[i] > 0.5 * deepest) {
            if (!inside) {
                slips++;
                inside = 1;
            }
        } else {
            inside = 0;
        }
    }
    const double per_period = slips / ((double)n * f0 / SR);

    IV_CHECK(deepest > 0.8 * (hi - lo));
    IV_CHECK(per_period > 0.95 && per_period < 1.05);
    if (deepest <= 0.8 * (hi - lo) || per_period <= 0.95 || per_period >= 1.05) {
        printf("  %.0f Гц: срывов на период %.3f, глубина %.2f размаха\n",
               (double)hz, per_period, deepest / (hi - lo));
    }
}

static void test_helmholtz_waveform(void)
{
    /* Ведём смычок «вниз», то есть в плюс: при обратном штрихе форма волны
     * зеркальна, и провал стал бы подъёмом. */
    check_helmholtz_slip(196.0f);
    check_helmholtz_slip(440.0f);
}

/* Медленный смычок скрипит. Шум трения растёт как корень из скорости, значит
 * относительно полезного сигнала на медленном ведении его заметно больше —
 * это и есть скрип, и он тут не украшение, а признак живого штриха. */
static void test_slow_bow_is_noisy(void)
{
    const float fast_v = 0.25f, slow_v = 0.03f;
    const float noise_at = 0.035f; /* размах шума при полной скорости */

    const float *w = bow(440.0f, fast_v, 1.65f, noise_at);
    double f0 = iv_an_f0(w, N / 2, SR, 80.0, 2000.0);
    const double fast = noise_fraction(w, f0);

    w = bow(440.0f, slow_v, 1.65f, noise_at * sqrtf(slow_v / fast_v));
    f0 = iv_an_f0(w, N / 2, SR, 80.0, 2000.0);
    const double slow = noise_fraction(w, f0);

    IV_CHECK(slow > fast * 5.0);
}

/* Лёгкий нажим при быстром смычке ломает движение Гельмгольца: струна
 * срывается дважды за период, и вместо ноты выходит стеклянный призвук
 * октавой выше. Проверяем, что модель это умеет — на скрипке это происходит
 * ровно при тех же условиях, и раскладка жестов в фазе 5 должна знать, где
 * граница. */
static void test_light_bow_breaks_up(void)
{
    const float *w = bow(440.0f, 0.40f, 5.0f, 0.0f);
    const double f0 = iv_an_f0(w, N / 2, SR, 80.0, 2000.0);
    /* Либо высота ушла вверх, либо вторая гармоника перебила первую —
     * оба признака означают одно: чистого Гельмгольца больше нет. */
    IV_CHECK(f0 > 440.0 * 1.5 || harmonic_db(w, 440.0, 2) > -3.0);
}

/* Затухание: сняли смычок — струна должна замолчать, и не когда-нибудь.
 * На этом держится тишина при пропаже связи. */
static void test_damping_stops_the_string(void)
{
    bow(440.0f, 0.25f, 1.65f, 0.0f);
    iv_string_set_contact(&g_str, 0.0f); /* смычок снят */
    iv_string_set_damping(&g_str, 0.080f);

    for (size_t i = 0; i < N / 2; ++i) { /* полсекунды без смычка */
        g_buf[i] = iv_string_tick(&g_str, 0.0f, 1.65f);
    }
    IV_CHECK(iv_an_peak(g_buf, 1000) > 0.01);              /* сначала звучит */
    IV_CHECK(iv_an_peak(g_buf + N / 2 - 1000, 1000) < 1e-4); /* потом молчит */
}

/* Полное гашение обязано давать ровный ноль, а не «почти»: молчащая струна
 * дальше просто не считается. */
static void test_clear_is_exact_zero(void)
{
    bow(440.0f, 0.25f, 1.65f, 0.0f);
    iv_string_set_contact(&g_str, 0.0f);
    iv_string_clear(&g_str);

    for (int i = 0; i < 1000; ++i) {
        IV_CHECK(iv_string_tick(&g_str, 0.0f, 1.65f) == 0.0f);
    }
}

int main(void)
{
    test_tuning();
    test_helmholtz_spectrum();
    test_helmholtz_waveform();
    test_slow_bow_is_noisy();
    test_light_bow_breaks_up();
    test_damping_stops_the_string();
    test_clear_is_exact_zero();
    IV_TEST_END();
}
