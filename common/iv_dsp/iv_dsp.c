#include "iv_dsp.h"
#include <math.h>

void iv_smooth_init(iv_smooth_t *s, float sample_rate, float tau_s, float initial)
{
    s->y = initial;
    if (tau_s <= 0.0f || sample_rate <= 0.0f) {
        s->a = 0.0f; /* без сглаживания: значение принимается мгновенно */
    } else {
        s->a = expf(-1.0f / (tau_s * sample_rate));
    }
}

void iv_smooth_reset(iv_smooth_t *s, float value)
{
    s->y = value;
}

void iv_lpf_init(iv_lpf_t *f, float sample_rate, float cutoff_hz)
{
    /* Состояние обнуляется до расчёта коэффициента: set_cutoff сохраняет
     * текущее значение фильтра, и читать его неинициализированным нельзя. */
    f->s.y = 0.0f;
    iv_lpf_set_cutoff(f, sample_rate, cutoff_hz);
}

void iv_lpf_set_cutoff(iv_lpf_t *f, float sample_rate, float cutoff_hz)
{
    /* Срез зажат снизу и сверху: нулевая частота дала бы полюс на единице
     * (фильтр перестал бы пропускать что-либо и завис на своём состоянии),
     * а выше половины частоты дискретизации срез не имеет смысла. */
    const float ny = sample_rate * 0.5f;
    const float fc = iv_clampf(cutoff_hz, 1.0f, ny * 0.99f);
    const float y  = f->s.y;

    iv_smooth_init(&f->s, sample_rate, 1.0f / (6.283185307f * fc), y);
}

void iv_lpf_reset(iv_lpf_t *f, float value)
{
    f->s.y = value;
}

float iv_note_to_hz(float midi_note, float a4_hz)
{
    return a4_hz * powf(2.0f, (midi_note - 69.0f) / 12.0f);
}

float iv_onepole_phase_delay(float pole, float hz, float sample_rate)
{
    const float w = 6.283185307f * hz / sample_rate;
    if (w <= 1e-6f) {
        return pole / (1.0f - pole); /* предел на нулевой частоте */
    }
    /* Фаза знаменателя 1 - p*exp(-jw), делённая на частоту: это и есть
     * задержка в сэмплах, которую фильтр добавляет к петле. */
    return atan2f(pole * sinf(w), 1.0f - pole * cosf(w)) / w;
}

/* ---- Линия задержки ------------------------------------------------------ */

void iv_delay_init(iv_delay_t *d, float delay)
{
    d->w = 0;
    d->last = 0.0f;
    iv_delay_clear(d);
    iv_delay_set(d, delay);
}

void iv_delay_clear(iv_delay_t *d)
{
    for (int i = 0; i < IV_DELAY_MAX; ++i) {
        d->buf[i] = 0.0f;
    }
    d->last = 0.0f;
}

/* ---- Биквад -------------------------------------------------------------- */

void iv_biquad_reset(iv_biquad_t *f)
{
    f->z1 = 0.0f;
    f->z2 = 0.0f;
}

void iv_biquad_peak(iv_biquad_t *f, float sample_rate, float f0, float q, float gain_db)
{
    const float a  = powf(10.0f, gain_db / 40.0f);
    const float w0 = 6.283185307f * iv_clampf(f0, 1.0f, sample_rate * 0.49f) / sample_rate;
    const float cw = cosf(w0);
    const float al = sinf(w0) / (2.0f * q);

    const float a0 = 1.0f + al / a;
    f->b0 = (1.0f + al * a) / a0;
    f->b1 = (-2.0f * cw) / a0;
    f->b2 = (1.0f - al * a) / a0;
    f->a1 = (-2.0f * cw) / a0;
    f->a2 = (1.0f - al / a) / a0;
    iv_biquad_reset(f);
}

void iv_biquad_highpass(iv_biquad_t *f, float sample_rate, float f0, float q)
{
    const float w0 = 6.283185307f * iv_clampf(f0, 1.0f, sample_rate * 0.49f) / sample_rate;
    const float cw = cosf(w0);
    const float al = sinf(w0) / (2.0f * q);

    const float a0 = 1.0f + al;
    f->b0 = ((1.0f + cw) * 0.5f) / a0;
    f->b1 = (-(1.0f + cw)) / a0;
    f->b2 = f->b0;
    f->a1 = (-2.0f * cw) / a0;
    f->a2 = (1.0f - al) / a0;
    iv_biquad_reset(f);
}

