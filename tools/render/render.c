/* Оффлайн-рендер жеста в WAV.
 *
 * Настройка звучания — самая длинная часть проекта, и через перепрошивку её
 * гонять нельзя: цикл «поправил → залил → послушал» занимает минуты и убивает
 * всякое желание пробовать. Здесь тот же самый синтезатор, что и в прошивке,
 * играет записанный жест на компьютере за доли секунды — можно слушать, можно
 * смотреть спектр, можно сравнивать варианты между собой.
 *
 * Жест — это то, что слой управления присылает синтезу: высота, скорость
 * смычка, нажим, как функции времени. Встроенные жесты покрывают то, по чему
 * судят о смычковой модели (нота, разворот штриха, медленное ведение, гамма
 * со сменой струн), а свой жест можно подать текстовым файлом — в том же
 * формате, в котором его когда-нибудь запишет живой смычок. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h> /* strcasecmp */

#include "iv_synth.h"
#include "iv_analysis.h"

#define SR IV_SAMPLE_RATE

/* Точка излома жеста. Между точками всё меняется линейно.
 *
 * blend — куда смычок съехал от своей струны к следующей, 0..1: 0 — весь на
 * ней, 0.5 — ровно на границе, обе звучат поровну. На инструменте это угол
 * смычка; здесь угла нет, а слышать переход надо. */
typedef struct {
    float t, hz, vel, force, brightness, blend;
} bp_t;

typedef struct {
    const char *name;
    const char *what;
    const bp_t *bp;
    int         n;
} gesture_t;

/* Нота целиком: смычок трогается, ведёт, останавливается. Слышно атаку,
 * установившийся тон и то, как звук гаснет. */
static const bp_t G_NOTE[] = {
    { 0.00f, 440.0f, 0.0f, 0.5f, 0.5f, 0.0f },
    { 0.02f, 440.0f, 0.9f, 0.5f, 0.5f, 0.0f },
    { 1.20f, 440.0f, 0.9f, 0.5f, 0.5f, 0.0f },
    { 1.25f, 440.0f, 0.0f, 0.5f, 0.5f, 0.0f },
    { 2.00f, 440.0f, 0.0f, 0.5f, 0.5f, 0.0f },
};

/* Разворот смычка: тот самый момент, по которому слышно, живой инструмент или
 * генератор. Скорость проходит через ноль и меняет знак. */
static const bp_t G_REVERSAL[] = {
    { 0.00f, 440.0f,  0.0f, 0.5f, 0.5f, 0.0f },
    { 0.02f, 440.0f,  0.9f, 0.5f, 0.5f, 0.0f },
    { 0.80f, 440.0f,  0.9f, 0.5f, 0.5f, 0.0f },
    { 0.86f, 440.0f, -0.9f, 0.5f, 0.5f, 0.0f },
    { 1.60f, 440.0f, -0.9f, 0.5f, 0.5f, 0.0f },
    { 1.66f, 440.0f,  0.9f, 0.5f, 0.5f, 0.0f },
    { 2.40f, 440.0f,  0.9f, 0.5f, 0.5f, 0.0f },
    { 2.45f, 440.0f,  0.0f, 0.5f, 0.5f, 0.0f },
    { 3.00f, 440.0f,  0.0f, 0.5f, 0.5f, 0.0f },
};

/* Медленное ведение с постепенным нажимом: здесь модель должна скрипеть, а не
 * петь. Шум трения при малой скорости соизмерим с полезным сигналом. */
static const bp_t G_SLOW[] = {
    { 0.00f, 293.7f, 0.00f, 0.3f, 0.5f, 0.0f },
    { 0.10f, 293.7f, 0.06f, 0.3f, 0.5f, 0.0f },
    { 1.50f, 293.7f, 0.06f, 0.9f, 0.5f, 0.0f },
    { 3.00f, 293.7f, 0.20f, 0.9f, 0.5f, 0.0f },
    { 3.20f, 293.7f, 0.00f, 0.9f, 0.5f, 0.0f },
    { 3.80f, 293.7f, 0.00f, 0.9f, 0.5f, 0.0f },
};

/* Гамма от G3 вверх на две октавы: проверка строя и того, что смена струны
 * проходит без щелчка. */
static const bp_t G_SCALE[] = {
    { 0.00f, 196.0f, 0.0f, 0.5f, 0.5f, 0.0f },
    { 0.02f, 196.0f, 0.8f, 0.5f, 0.5f, 0.0f },
    { 3.50f, 784.0f, 0.8f, 0.5f, 0.5f, 0.0f },
    { 3.60f, 784.0f, 0.0f, 0.5f, 0.5f, 0.0f },
    { 4.20f, 784.0f, 0.0f, 0.5f, 0.5f, 0.0f },
};

/* Двойная нота: смычок съезжает с A на границу с E и обратно. Здесь слышно
 * главное, ради чего доли и заведены, — что на границе звучат обе струны,
 * а не одна из них с перескоком. */
static const bp_t G_DOUBLE[] = {
    { 0.00f, 440.0f, 0.0f, 0.5f, 0.5f, 0.0f },
    { 0.02f, 440.0f, 0.8f, 0.5f, 0.5f, 0.0f },
    { 0.80f, 440.0f, 0.8f, 0.5f, 0.5f, 0.0f },
    { 1.40f, 440.0f, 0.8f, 0.5f, 0.5f, 0.5f },
    { 2.20f, 440.0f, 0.8f, 0.5f, 0.5f, 0.5f },
    { 2.80f, 440.0f, 0.8f, 0.5f, 0.5f, 0.0f },
    { 3.20f, 440.0f, 0.8f, 0.5f, 0.5f, 0.0f },
    { 3.25f, 440.0f, 0.0f, 0.5f, 0.5f, 0.0f },
    { 4.00f, 440.0f, 0.0f, 0.5f, 0.5f, 0.0f },
};

/* Крещендо на одной ноте: скорость и нажим растут вместе, как при настоящем
 * усилении звука. */
static const bp_t G_DYNAMICS[] = {
    { 0.00f, 392.0f, 0.00f, 0.2f, 0.5f, 0.0f },
    { 0.05f, 392.0f, 0.10f, 0.2f, 0.5f, 0.0f },
    { 2.50f, 392.0f, 1.00f, 0.9f, 0.5f, 0.0f },
    { 2.60f, 392.0f, 0.00f, 0.9f, 0.5f, 0.0f },
    { 3.20f, 392.0f, 0.00f, 0.9f, 0.5f, 0.0f },
};

#define G(x, w) { #x, w, G_##x, (int)(sizeof(G_##x) / sizeof(bp_t)) }
static const gesture_t GESTURES[] = {
    G(NOTE,     "нота целиком: атака, ведение, снятие"),
    G(REVERSAL, "два разворота смычка"),
    G(SLOW,     "медленное ведение с нажимом: скрип"),
    G(SCALE,    "гамма на две октавы со сменой струн"),
    G(DOUBLE,   "двойная нота: смычок на границе двух струн"),
    G(DYNAMICS, "крещендо"),
};
#undef G
#define N_GESTURES ((int)(sizeof(GESTURES) / sizeof(GESTURES[0])))

/* ---- жест из файла -------------------------------------------------------- */

#define MAX_BP 4096
static bp_t g_file_bp[MAX_BP];

/* Формат: строки «время высота скорость нажим [яркость [доля]]»,
 * # — комментарий.
 * Ровно то, что слой управления отдаёт синтезу, — значит, сюда же ляжет и
 * запись живого жеста, когда её будет чем снимать. */
static int load_gesture(const char *path, bp_t **out)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "не открыть %s\n", path);
        return -1;
    }

    char line[256];
    int  n = 0, lineno = 0;
    while (fgets(line, sizeof(line), f)) {
        lineno++;
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '#' || *p == '\n' || *p == '\0') {
            continue;
        }
        if (n >= MAX_BP) {
            fprintf(stderr, "слишком много точек, больше %d не берём\n", MAX_BP);
            break;
        }
        bp_t b;
        b.brightness = 0.5f; /* необязательное пятое число */
        b.blend      = 0.0f; /* необязательное шестое: съезд к соседней струне */
        const int got = sscanf(p, "%f %f %f %f %f %f",
                               &b.t, &b.hz, &b.vel, &b.force, &b.brightness, &b.blend);
        if (got < 4) {
            fprintf(stderr,
                    "%s:%d: жду «время высота скорость нажим [яркость [доля]]»\n",
                    path, lineno);
            fclose(f);
            return -1;
        }
        g_file_bp[n++] = b;
    }
    fclose(f);

    if (n < 2) {
        fprintf(stderr, "%s: нужно хотя бы две точки\n", path);
        return -1;
    }
    *out = g_file_bp;
    return n;
}

/* ---- рендер --------------------------------------------------------------- */

static iv_synth_params_t at_time(const bp_t *bp, int n, float t)
{
    int i = 0;
    while (i + 2 < n && bp[i + 1].t <= t) {
        i++;
    }
    const bp_t *a = &bp[i], *b = &bp[i + 1];
    float u = (b->t > a->t) ? (t - a->t) / (b->t - a->t) : 0.0f;
    u = iv_clampf(u, 0.0f, 1.0f);

    const float hz = iv_lerpf(a->hz, b->hz, u);

    iv_synth_params_t p;
    p.bow_velocity = iv_lerpf(a->vel, b->vel, u);
    p.bow_force    = iv_lerpf(a->force, b->force, u);
    p.brightness   = iv_lerpf(a->brightness, b->brightness, u);

    /* Струну здесь выбирает нота, а не угол смычка: угла в записи жеста нет.
     * Правило то же, что на инструменте выбрал бы играющий — самая высокая из
     * открытых струн, которой нота доступна. */
    int k = 0;
    while (k + 1 < IV_SYNTH_VOICES && hz >= iv_open_hz[k + 1]) {
        k++;
    }

    for (int i = 0; i < IV_SYNTH_VOICES; ++i) {
        p.bow_share[i] = 0.0f;
        p.freq_hz[i]   = iv_open_hz[i];
    }

    /* Съезд к соседней струне сверху. Её нота — та же позиция пальца, то есть
     * тот же сдвиг от открытой: на квинтовом строе это и даёт квинту. */
    const float blend = iv_clampf(iv_lerpf(a->blend, b->blend, u), 0.0f, 1.0f);
    p.bow_share[k] = 1.0f - blend;
    p.freq_hz[k]   = hz;
    if (k + 1 < IV_SYNTH_VOICES) {
        p.bow_share[k + 1] = blend;
        p.freq_hz[k + 1]   = hz * (iv_open_hz[k + 1] / iv_open_hz[k]);
    }
    return p;
}

static iv_synth_t g_synth;

static size_t render(const bp_t *bp, int n, float secs, float *out, size_t cap)
{
    size_t total = (size_t)(secs * SR);
    if (total > cap) {
        total = cap;
    }
    iv_synth_init(&g_synth, (float)SR);

    /* Параметры обновляются раз в блок — ровно как в прошивке, где их приносит
     * слой управления. Иначе хостовый звук отличался бы от железного именно
     * тем, что здесь труднее всего услышать. */
    for (size_t i = 0; i < total; i += IV_BLOCK_SAMPLES) {
        const size_t k = (total - i < IV_BLOCK_SAMPLES) ? total - i : IV_BLOCK_SAMPLES;
        const iv_synth_params_t p = at_time(bp, n, (float)i / (float)SR);
        iv_synth_set_params(&g_synth, &p);
        iv_synth_render(&g_synth, out + i, k);
    }
    return total;
}

/* ---- WAV ------------------------------------------------------------------ */

static void put32(FILE *f, unsigned v) { fputc(v & 255, f); fputc((v >> 8) & 255, f);
                                         fputc((v >> 16) & 255, f); fputc((v >> 24) & 255, f); }
static void put16(FILE *f, unsigned v) { fputc(v & 255, f); fputc((v >> 8) & 255, f); }

static int write_wav(const char *path, const float *x, size_t n)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "не создать %s\n", path);
        return -1;
    }
    fwrite("RIFF", 1, 4, f); put32(f, (unsigned)(36 + 2 * n));
    fwrite("WAVEfmt ", 1, 8, f); put32(f, 16);
    put16(f, 1); put16(f, 1);           /* PCM, моно */
    put32(f, SR); put32(f, SR * 2);
    put16(f, 2); put16(f, 16);
    fwrite("data", 1, 4, f); put32(f, (unsigned)(2 * n));

    for (size_t i = 0; i < n; ++i) {
        const float v = iv_clampf(x[i], -1.0f, 1.0f);
        put16(f, (unsigned)(short)lrintf(v * 32767.0f));
    }
    fclose(f);
    return 0;
}

/* ---- отчёт ---------------------------------------------------------------- */

/* Считаем по самому громкому полусекундному окну: в жесте есть и тишина, и
 * атака, а признаки смычкового звука ищутся в установившемся тоне. */
static void report(const float *x, size_t n)
{
    const size_t win = SR / 2;
    if (n < win) {
        printf("слишком короткий кусок для отчёта\n");
        return;
    }

    size_t best = 0;
    double best_rms = -1.0;
    for (size_t i = 0; i + win <= n; i += SR / 20) {
        const double r = iv_an_rms(x + i, win);
        if (r > best_rms) {
            best_rms = r;
            best = i;
        }
    }

    const float *w = x + best;
    const double f0 = iv_an_f0(w, win, SR, 80.0, 2000.0);

    printf("\nотчёт по окну %.2f–%.2f с\n", best / (double)SR, (best + win) / (double)SR);
    printf("  пик %.3f, rms %.3f (%.1f дБ)\n",
           iv_an_peak(x, n), best_rms, 20.0 * log10(best_rms + 1e-12));
    if (f0 <= 0.0) {
        printf("  высота не определяется — сигнал непериодический\n");
        return;
    }
    printf("  основной тон %.1f Гц, периодичность %.3f, наклон спектра %+.1f дБ\n",
           f0, iv_an_periodicity(w, win, SR, f0),
           iv_an_harmonic_tilt(w, win, SR, f0));

    /* Гармоники: у волны Гельмгольца они падают примерно как 1/n, то есть
     * -6 дБ на удвоение номера. Отклонения от этого — и есть характер. */
    const double h1 = iv_an_mag(w, win, SR, f0);
    printf("  гармоники со второй, дБ к первой (у пилы -6 -9.5 -12 -14 -15.6):\n   ");
    for (int h = 2; h <= 8 && f0 * h < SR / 2; ++h) {
        printf(" %5.1f", 20.0 * log10(iv_an_mag(w, win, SR, f0 * h) / (h1 + 1e-12) + 1e-12));
    }
    printf("\n");

    /* Доля энергии мимо гармоник — шум смычка и всё непериодическое. */
    double harm = 0.0;
    for (int h = 1; h <= 24 && f0 * h < SR / 2; ++h) {
        const double a = iv_an_mag(w, win, SR, f0 * h);
        harm += a * a / 2.0;
    }
    const double noise = 1.0 - harm / (best_rms * best_rms + 1e-18);
    printf("  шумность %.3f\n", noise < 0.0 ? 0.0 : noise);
    if (noise > 0.3) {
        printf("  (высота внутри окна менялась — гармоники и шумность тут ни о чём"
               " не говорят)\n");
    }
}

/* ---- точка входа ---------------------------------------------------------- */

static void usage(void)
{
    printf("iv_render — оффлайн-рендер жеста в WAV тем же синтезатором, что в прошивке\n\n"
           "  iv_render [--gesture ИМЯ | --file ФАЙЛ] [--out ФАЙЛ.wav]"
           " [--seconds N] [--report]\n\n"
           "встроенные жесты:\n");
    for (int i = 0; i < N_GESTURES; ++i) {
        printf("  %-9s %s\n", GESTURES[i].name, GESTURES[i].what);
    }
    printf("\nсвой жест — текстовый файл, строки «время высота скорость нажим"
           " [яркость]»,\n"
           "между точками всё меняется линейно, # — комментарий.\n");
}

#define MAX_SECONDS 60
static float g_buf[SR * MAX_SECONDS];

int main(int argc, char **argv)
{
    const char *out = "gesture.wav";
    const char *file = NULL;
    const char *name = "NOTE";
    float secs = 0.0f;
    int   want_report = 0;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--out") && i + 1 < argc) {
            out = argv[++i];
        } else if (!strcmp(argv[i], "--gesture") && i + 1 < argc) {
            name = argv[++i];
        } else if (!strcmp(argv[i], "--file") && i + 1 < argc) {
            file = argv[++i];
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            secs = (float)atof(argv[++i]);
        } else if (!strcmp(argv[i], "--report")) {
            want_report = 1;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            usage();
            return 0;
        } else {
            fprintf(stderr, "не понимаю «%s»\n\n", argv[i]);
            usage();
            return 1;
        }
    }

    const bp_t *bp = NULL;
    int nbp = 0;
    if (file) {
        nbp = load_gesture(file, (bp_t **)&bp);
        if (nbp < 0) {
            return 1;
        }
    } else {
        for (int i = 0; i < N_GESTURES; ++i) {
            if (!strcasecmp(name, GESTURES[i].name)) {
                bp = GESTURES[i].bp;
                nbp = GESTURES[i].n;
                break;
            }
        }
        if (!bp) {
            fprintf(stderr, "нет такого жеста: %s\n\n", name);
            usage();
            return 1;
        }
    }

    if (secs <= 0.0f) {
        secs = bp[nbp - 1].t; /* до последней точки жеста */
    }
    if (secs > MAX_SECONDS) {
        secs = MAX_SECONDS;
    }

    const size_t n = render(bp, nbp, secs, g_buf, sizeof(g_buf) / sizeof(*g_buf));
    if (write_wav(out, g_buf, n) != 0) {
        return 1;
    }
    printf("%s: %.2f с, %zu сэмплов, %d Гц\n", out, n / (double)SR, n, SR);

    if (want_report) {
        report(g_buf, n);
    }
    return 0;
}
