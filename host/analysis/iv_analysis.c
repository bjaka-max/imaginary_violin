#include "iv_analysis.h"
#include <math.h>
#include <string.h>

double iv_an_peak(const float *x, size_t n)
{
    double m = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double a = fabs((double)x[i]);
        if (a > m) m = a;
    }
    return m;
}

double iv_an_rms(const float *x, size_t n)
{
    double s = 0.0;
    for (size_t i = 0; i < n; ++i) {
        s += (double)x[i] * (double)x[i];
    }
    return n ? sqrt(s / (double)n) : 0.0;
}

double iv_an_mag(const float *x, size_t n, double sr, double hz)
{
    /* Гёрцель: два состояния и один косинус на всю частоту. */
    const double w = 2.0 * M_PI * hz / sr;
    const double c = 2.0 * cos(w);
    double s1 = 0.0, s2 = 0.0;

    for (size_t i = 0; i < n; ++i) {
        const double s0 = (double)x[i] + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    const double re = s1 - s2 * cos(w);
    const double im = s2 * sin(w);
    return 2.0 * sqrt(re * re + im * im) / (double)n;
}

/* Автокорреляция на лаге lag, нормированная на энергию окна. */
static double norm_corr(const float *x, size_t n, size_t lag)
{
    double num = 0.0, ea = 0.0, eb = 0.0;
    for (size_t i = 0; i + lag < n; ++i) {
        const double a = x[i], b = x[i + lag];
        num += a * b;
        ea  += a * a;
        eb  += b * b;
    }
    const double den = sqrt(ea * eb);
    return den > 0.0 ? num / den : 0.0;
}

double iv_an_f0(const float *x, size_t n, double sr, double lo, double hi)
{
    const size_t min_lag = (size_t)(sr / hi);
    const size_t max_lag = (size_t)(sr / lo);
    if (max_lag + 2 >= n || min_lag < 2) {
        return 0.0;
    }

    double best_v = -2.0;
    for (size_t lag = min_lag; lag <= max_lag; ++lag) {
        const double v = norm_corr(x, n, lag);
        if (v > best_v) {
            best_v = v;
        }
    }
    if (best_v < 0.2) {
        return 0.0;
    }

    /* Берём не самый высокий максимум, а самый ранний из почти таких же.
     * У строго периодичного сигнала автокорреляция одинаково высока и на
     * периоде, и на всех его кратных, а какой из них окажется выше — решает
     * шум в последнем знаке. Без этого правила оценка регулярно съезжает на
     * октаву-две вниз, и потом кажется, будто модель сорвалась в субгармонику. */
    size_t best = 0;
    for (size_t lag = min_lag + 1; lag + 1 <= max_lag; ++lag) {
        const double v = norm_corr(x, n, lag);
        if (v >= 0.85 * best_v &&
            v >= norm_corr(x, n, lag - 1) && v >= norm_corr(x, n, lag + 1)) {
            best = lag;
            best_v = v;
            break;
        }
    }
    if (best == 0) {
        return 0.0;
    }

    /* Парабола по трём точкам вокруг максимума. */
    const double ym = norm_corr(x, n, best - 1);
    const double y0 = best_v;
    const double yp = norm_corr(x, n, best + 1);
    const double den = ym - 2.0 * y0 + yp;
    double shift = 0.0;
    if (fabs(den) > 1e-12) {
        shift = 0.5 * (ym - yp) / den;
        if (shift < -1.0 || shift > 1.0) {
            shift = 0.0;
        }
    }
    return sr / ((double)best + shift);
}

double iv_an_periodicity(const float *x, size_t n, double sr, double f0)
{
    if (f0 <= 0.0) {
        return 0.0;
    }
    const size_t lag = (size_t)(sr / f0 + 0.5);
    if (lag == 0 || lag + 2 >= n) {
        return 0.0;
    }
    return norm_corr(x, n, lag);
}

void iv_an_spectrum(const float *x, size_t n, double *mag, size_t nfft)
{
    static double re[1 << 14], im[1 << 14];
    if (nfft > (1u << 14) || n < nfft) {
        memset(mag, 0, (nfft / 2) * sizeof(*mag));
        return;
    }

    for (size_t i = 0; i < nfft; ++i) {
        /* Окно Ханна: без него боковые лепестки прямоугольного окна съедают
         * разницу между гармониками, ради которой всё и считается. */
        const double w = 0.5 - 0.5 * cos(2.0 * M_PI * (double)i / (double)nfft);
        re[i] = (double)x[i] * w;
        im[i] = 0.0;
    }

    /* Перестановка по обратным битам, потом бабочки: обычный радикс-2. */
    for (size_t i = 1, j = 0; i < nfft; ++i) {
        size_t bit = nfft >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            double t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }

    for (size_t len = 2; len <= nfft; len <<= 1) {
        const double ang = -2.0 * M_PI / (double)len;
        for (size_t i = 0; i < nfft; i += len) {
            for (size_t k = 0; k < len / 2; ++k) {
                const double c = cos(ang * (double)k), s = sin(ang * (double)k);
                const double ur = re[i + k], ui = im[i + k];
                const double vr = re[i + k + len / 2] * c - im[i + k + len / 2] * s;
                const double vi = re[i + k + len / 2] * s + im[i + k + len / 2] * c;
                re[i + k] = ur + vr;
                im[i + k] = ui + vi;
                re[i + k + len / 2] = ur - vr;
                im[i + k + len / 2] = ui - vi;
            }
        }
    }

    for (size_t i = 0; i < nfft / 2; ++i) {
        mag[i] = 2.0 * sqrt(re[i] * re[i] + im[i] * im[i]) / (double)nfft;
    }
}

double iv_an_centroid(const float *x, size_t n, double sr, size_t nfft)
{
    static double mag[1 << 13];
    if (nfft / 2 > (1u << 13)) {
        return 0.0;
    }
    iv_an_spectrum(x, n, mag, nfft);

    double num = 0.0, den = 0.0;
    for (size_t i = 1; i < nfft / 2; ++i) {
        const double f = (double)i * sr / (double)nfft;
        num += f * mag[i];
        den += mag[i];
    }
    return den > 0.0 ? num / den : 0.0;
}

double iv_an_harmonic_tilt(const float *x, size_t n, double sr, double f0)
{
    if (f0 <= 0.0) {
        return 0.0;
    }
    double lo = 0.0, hi = 0.0;
    for (int h = 1; h <= 12 && f0 * h < sr * 0.5; ++h) {
        const double a = iv_an_mag(x, n, sr, f0 * h);
        if (h <= 3) {
            lo += a * a;
        } else {
            hi += a * a;
        }
    }
    return 10.0 * log10(hi / (lo + 1e-18) + 1e-18);
}
