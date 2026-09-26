#include "iv_synth.h"
#include <math.h>

/* Открытые струны скрипки: G3-D4-A4-E5, квинтами. */
const float iv_open_hz[IV_SYNTH_VOICES] = { 196.00f, 293.66f, 440.00f, 659.26f };

/* Скорость смычка при полном размахе управления. Величина не в метрах в
 * секунду, а в тех же единицах, что и волны в струне: важно её отношение к
 * ширине области прилипания. Больше — звук грубеет и начинает срываться,
 * меньше — не набирает силы. */
#define BOW_MAX_VELOCITY 0.25f

/* Нажим смычка → крутизна кривой трения. Сильнее нажим — шире область
 * прилипания, значит кривая положе.
 *
 * Границы выбраны по замерам, а не на глаз: между ними модель держит движение
 * Гельмгольца, за ними срывается — и оба срыва настоящие, их слышно и на
 * скрипке. Легче BOW_SLOPE_LIGHT при быстром смычке — струна начинает
 * проскальзывать дважды за период, и вместо ноты получается стеклянный призвук
 * октавой выше (то самое, что выходит, когда ведёшь быстро и не нажимаешь).
 * Тяжелее BOW_SLOPE_HEAVY при медленном — смычок просто тащит струну за собой,
 * срыва нет, звук давится. Играбельная область между ними — это диаграмма
 * Шеллинга, и она здесь получилась сама, из физики модели. */
#define BOW_SLOPE_LIGHT 3.0f /* нажим 0 */
#define BOW_SLOPE_HEAVY 0.3f /* нажим 1 */

/* Размах шума смычка при полной скорости. Растёт как корень из скорости, то
 * есть на медленном ведении шума относительно звука больше — и нота на нём
 * скрипит, как на настоящем инструменте. */
#define BOW_NOISE 0.035f

/* Сглаживание управления. Скорость — быстро, иначе смазывается атака;
 * нажим — вчетверо медленнее, потому что его скачок меняет форму кривой
 * трения, а это слышно как ступеньку в тембре. */
#define VEL_TAU   0.005f
#define FORCE_TAU 0.020f

/* Затухание при снятом смычке. Две разные постоянные времени, и разница
 * содержательная:
 *  - смычок остановлен — гасим быстро: «ноль скорости = тишина» это правило
 *    раскладки жестов, и на нём же держится тишина при пропаже связи;
 *  - струна просто осталась без смычка (перешли на соседнюю) — она звенит
 *    свободно, как на настоящем инструменте. */
#define DAMP_STOP_TAU 0.080f
#define DAMP_FREE_TAU 0.350f

/* Ниже этого уровня (-100 дБ) звук считается кончившимся: состояние обнуляется
 * начисто. Это даёт точный ноль на выходе — не «почти», а ровный — и заодно
 * снимает с процессора хвост, который уже никто не услышит. */
#define QUIET_LEVEL 1e-5f

/* Выходное усиление. Подобрано так, чтобы нота с нормальным нажимом и полной
 * скоростью смычка выходила примерно на -6 дБ: есть куда деться пикам атаки,
 * и ограничитель ниже почти не работает. */
#define OUTPUT_GAIN 0.28f

/* Мягкое ограничение. Атака на развороте смычка и совпадение резонансов
 * корпуса дают короткие выбросы; жёсткое обрезание превращает их в треск,
 * а этот кубический ограничитель гладко упирается ровно в единицу при |x|=1.5
 * (в этой точке и значение, и производная сходятся). */
static float soft_clip(float x)
{
    if (x >=  1.5f) return  1.0f;
    if (x <= -1.5f) return -1.0f;
    return x - (4.0f / 27.0f) * x * x * x;
}

void iv_synth_init(iv_synth_t *s, float sample_rate)
{
    s->sample_rate = sample_rate;
    s->selected = 2; /* A: показывать до первого пакета что-то надо */
    s->silent = true;

    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        iv_string_init(&s->strings[k], sample_rate, iv_open_hz[k]);
        s->share[k] = 0.0f;
        s->target.bow_share[k] = 0.0f;
        s->target.freq_hz[k]   = iv_open_hz[k];
    }
    iv_body_init(&s->body, sample_rate);

    iv_smooth_init(&s->vel,   sample_rate, VEL_TAU,   0.0f);
    iv_smooth_init(&s->force, sample_rate, FORCE_TAU, 0.0f);

    s->target.bow_velocity = 0.0f;
    s->target.bow_force    = 0.0f;
    s->target.brightness   = 0.5f;
}

void iv_synth_set_params(iv_synth_t *s, const iv_synth_params_t *p)
{
    /* Только запоминаем цель. Сглаживание идёт посэмплово в render(),
     * то есть уже в аудиопотоке. */
    s->target = *p;
}

static void render_block(iv_synth_t *s, float *out, size_t n)
{
    const float target_vel = BOW_MAX_VELOCITY
                           * iv_clampf(s->target.bow_velocity, -1.0f, 1.0f);
    const float target_force = iv_clampf(s->target.bow_force, 0.0f, 1.0f);

    /* Доли смычка приносит слой управления. Синтез в них не вникает, но
     * проверяет: испорченная доля — это контакт больше единицы, то есть
     * усиление в петле трения. Ноль скорости снимает смычок со всех струн
     * разом, и это правило раскладки, а не следствие арифметики. */
    float share[IV_SYNTH_VOICES];
    bool  bowing = (target_vel != 0.0f);
    if (bowing) {
        bool any = false;
        for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
            share[k] = iv_clampf(s->target.bow_share[k], 0.0f, 1.0f);
            any = any || (share[k] > 0.0f);
        }
        bowing = any;
    }
    if (!bowing) {
        for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
            share[k] = 0.0f;
        }
    }

    /* Всё погашено и смычок снят — считать нечего. Ровный ноль здесь важнее
     * экономии: именно он уходит в кодек при пропаже связи. */
    if (s->silent && !bowing) {
        for (size_t i = 0; i < n; ++i) {
            out[i] = 0.0f;
        }
        return;
    }
    s->silent = false;

    const float brightness = iv_clampf(s->target.brightness, 0.0f, 1.0f);
    /* Шум гаснет вместе со скоростью — снятый смычок обязан давать тишину,
     * а не шипение, — и делится между струнами вместе со смычком. */
    const float noise = BOW_NOISE * sqrtf(fabsf(target_vel) / BOW_MAX_VELOCITY);

    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        iv_string_set_brightness(&s->strings[k], brightness);
        iv_string_set_contact(&s->strings[k], share[k]);
        iv_string_set_bow_noise(&s->strings[k], noise * share[k]);

        if (share[k] > 0.0f) {
            s->strings[k].quiet = false; /* смычок вернул её к жизни */
            /* Струна, которой смычок только что коснулся, обязана заговорить
             * на своей ноте, а не приехать на неё глиссандо с той, что звучала
             * на ней в прошлый раз. Уже ведомая, наоборот, идёт за пальцем. */
            if (s->share[k] > 0.0f) {
                iv_string_set_freq(&s->strings[k], s->target.freq_hz[k]);
            } else {
                iv_string_snap_freq(&s->strings[k], s->target.freq_hz[k]);
            }
        }

        float tau = DAMP_FREE_TAU;
        if (!bowing) {
            tau = DAMP_STOP_TAU;
        } else if (share[k] > 0.0f) {
            tau = 0.0f; /* под смычком — только собственные потери струны */
        }
        iv_string_set_damping(&s->strings[k], tau);

        /* Главная струна — для экрана. Сравнение строгое, поэтому на ровной
         * границе (доли поровну) показ не мечется между соседками. */
        if (share[k] > share[s->selected]) {
            s->selected = k;
        }
    }

    /* Управление — на весь блок вперёд. */
    for (size_t i = 0; i < n; ++i) {
        s->vel_buf[i] = iv_smooth_process(&s->vel, target_vel);
        const float f = iv_smooth_process(&s->force, target_force);
        s->slope_buf[i] = BOW_SLOPE_LIGHT + (BOW_SLOPE_HEAVY - BOW_SLOPE_LIGHT) * f;
        out[i] = 0.0f;
    }

    /* Струна за струной по всему блоку, а не сэмпл за сэмплом по всем струнам:
     * так состояние каждой живёт в регистрах, а не перечитывается из памяти на
     * каждом сэмпле. Считаются все звучащие: под смычком одна или две,
     * остальные догорают после смены струны; обнулённую считать незачем — она
     * отдаёт ровный ноль.
     *
     * Скорость смычка получает не только та струна, что под смычком, но и та,
     * с которой он только что ушёл: смычок при переходе не останавливается,
     * у него гаснет сцепление. Обрубить скорость разом значило бы щёлкнуть. */
    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        if (s->strings[k].quiet) {
            continue;
        }
        const bool touched = (share[k] > 0.0f) || (s->strings[k].contact.y > 1e-4f);
        iv_string_render(&s->strings[k], touched ? s->vel_buf : NULL,
                         s->slope_buf, out, n);
    }

    /* Доли этого блока — чтобы в следующем отличить «смычок уже вёл» от
     * «только что коснулся»: от этого зависит, ехать высоте глиссандо или
     * встать сразу. */
    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        s->share[k] = share[k];
    }

    iv_body_render(&s->body, out, n);

    float block_peak = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const float y = soft_clip(OUTPUT_GAIN * out[i]);
        out[i] = y;

        const float mag = fabsf(y);
        if (mag > block_peak) {
            block_peak = mag;
        }
    }

    /* Гашение в два шага: сначала обнуляются струны, доигравшие до -100 дБ,
     * и только когда после этого замолкает и корпус — весь синтезатор.
     * Одним шагом нельзя: корпус звенит своей добротностью ещё десятки
     * миллисекунд после того, как струна отдала последнюю волну. */
    bool strings_quiet = true;
    for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
        if (s->strings[k].peak >= QUIET_LEVEL) {
            strings_quiet = false;
        }
        s->strings[k].peak = 0.0f;
    }

    if (!bowing && strings_quiet) {
        for (int k = 0; k < IV_SYNTH_VOICES; ++k) {
            iv_string_clear(&s->strings[k]);
        }
        if (block_peak < QUIET_LEVEL) {
            iv_body_reset(&s->body);
            s->silent = true;
        }
    }
}

void iv_synth_render(iv_synth_t *s, float *out, size_t n)
{
    /* Режем на блоки сами: параметры и гашение обновляются раз в блок, и от
     * длины куска, которым позвали render, результат зависеть не должен —
     * иначе хостовый рендер звучал бы не так, как прошивка, и настраивать
     * звук на компьютере было бы бессмысленно. */
    while (n > 0) {
        const size_t k = n > IV_BLOCK_SAMPLES ? (size_t)IV_BLOCK_SAMPLES : n;
        render_block(s, out, k);
        out += k;
        n   -= k;
    }
}
