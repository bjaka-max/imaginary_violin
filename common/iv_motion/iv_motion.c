#include "iv_motion.h"
#include "iv_dsp.h"

#include <math.h>
#include <string.h>

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329252f
#define G_MS2      9.80665f

static float vec_mag(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/* ---- калибровка ----------------------------------------------------------- */

void iv_calib_reset(iv_calib_t *c)
{
    memset(c, 0, sizeof(*c));
    c->accel_scale = 1.0f;
    c->valid = false;
}

void iv_calib_apply(const iv_calib_t *c, float a[3], float g[3])
{
    if (!c->valid) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        a[i] *= c->accel_scale;
        g[i] -= c->gyro_bias[i];
    }
}

void iv_calib_begin(iv_calib_acc_t *acc)
{
    memset(acc, 0, sizeof(*acc));
}

bool iv_calib_feed(iv_calib_acc_t *acc, const float a[3], const float g[3], uint32_t need)
{
    /* Первая выборка серии задаёт опорное положение; дальше сверяемся с ней.
     * Сравнивать с предыдущей выборкой было бы неверно: медленный увод
     * (кто-то держит смычок в руке) прошёл бы незамеченным. */
    if (acc->n == 0) {
        acc->a_ref[0] = a[0];
        acc->a_ref[1] = a[1];
        acc->a_ref[2] = a[2];
    } else {
        for (int i = 0; i < 3; ++i) {
            if (fabsf(a[i] - acc->a_ref[i]) > IV_CALIB_ACCEL_TOL_G) {
                iv_calib_begin(acc); /* дрогнуло — считаем заново */
                return false;
            }
        }
    }
    if (vec_mag(g) > IV_CALIB_GYRO_TOL_DPS) {
        iv_calib_begin(acc);
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        acc->g_sum[i] += g[i];
    }
    acc->a_mag_sum += vec_mag(a);
    acc->n++;

    return acc->n >= need;
}

void iv_calib_take(const iv_calib_acc_t *acc, iv_calib_t *out)
{
    iv_calib_reset(out);
    if (acc->n == 0) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        out->gyro_bias[i] = acc->g_sum[i] / (float)acc->n;
    }

    /* В покое модуль ускорения обязан быть ровно 1 g: всё, что сверх, —
     * ошибка масштаба датчика. Поправка зажата: если среднее уехало далеко,
     * это не масштаб, а движение, и «исправлять» такое опасно. */
    const float mag = acc->a_mag_sum / (float)acc->n;
    if (mag > 0.8f && mag < 1.2f) {
        out->accel_scale = 1.0f / mag;
        out->valid = true;
    }
}

/* ---- ориентация ----------------------------------------------------------- */

void iv_orient_init(iv_orient_t *o, float sample_rate, float tau_s)
{
    o->roll = 0.0f;
    o->pitch = 0.0f;
    o->dt = (sample_rate > 0.0f) ? 1.0f / sample_rate : 0.0f;
    /* Доля гироскопа за шаг: чем длиннее постоянная времени, тем дольше
     * держится его быстрый отклик и тем медленнее гравитация правит дрейф. */
    o->k = (tau_s > 0.0f) ? tau_s / (tau_s + o->dt) : 0.0f;
    o->started = false;
}

/* Приводит угол в пределы ±180 относительно опорного: без этого переход через
 * ±180° даёт скачок на полный оборот, и фильтр на нём срывается. */
static float unwrap_near(float angle, float reference)
{
    while (angle - reference > 180.0f)  { angle -= 360.0f; }
    while (angle - reference < -180.0f) { angle += 360.0f; }
    return angle;
}

void iv_orient_update(iv_orient_t *o, const float a[3], const float g[3])
{
    const float flat = sqrtf(a[1] * a[1] + a[2] * a[2]);
    const float roll_acc  = atan2f(a[1], a[2]) * RAD_TO_DEG;
    const float pitch_acc = atan2f(-a[0], flat) * RAD_TO_DEG;

    if (!o->started) {
        /* Первая выборка принимается как есть: сходиться к ней с нуля значило
         * бы полсекунды врать сразу после включения. */
        o->roll = roll_acc;
        o->pitch = pitch_acc;
        o->started = true;
        return;
    }

    const float roll_gyro  = o->roll  + g[0] * o->dt;
    const float pitch_gyro = o->pitch + g[1] * o->dt;

    /* Доверие к акселерометру: чем сильнее показания расходятся с ожидаемой
     * гравитацией, тем вероятнее, что это не наклон, а собственное ускорение
     * устройства — и тем меньше его вес в поправке. */
    float grav[3];
    iv_orient_gravity(o, grav);
    const float dev = sqrtf((a[0] - grav[0]) * (a[0] - grav[0])
                          + (a[1] - grav[1]) * (a[1] - grav[1])
                          + (a[2] - grav[2]) * (a[2] - grav[2]));

    float trust = 1.0f;
    if (dev > IV_ORIENT_TRUST_LO_G) {
        trust = 1.0f - (dev - IV_ORIENT_TRUST_LO_G)
                     / (IV_ORIENT_TRUST_HI_G - IV_ORIENT_TRUST_LO_G);
        trust = iv_clampf(trust, 0.0f, 1.0f);
    }
    trust = IV_ORIENT_TRUST_MIN + (1.0f - IV_ORIENT_TRUST_MIN) * trust;

    const float w = (1.0f - o->k) * trust;
    o->roll  = roll_gyro  + w * (unwrap_near(roll_acc, roll_gyro) - roll_gyro);
    o->pitch = pitch_gyro + w * (unwrap_near(pitch_acc, pitch_gyro) - pitch_gyro);

    o->roll  = unwrap_near(o->roll, 0.0f);
    o->pitch = unwrap_near(o->pitch, 0.0f);
}

void iv_orient_gravity(const iv_orient_t *o, float out[3])
{
    const float r = o->roll * DEG_TO_RAD;
    const float p = o->pitch * DEG_TO_RAD;
    const float cp = cosf(p);

    out[0] = -sinf(p);
    out[1] = sinf(r) * cp;
    out[2] = cosf(r) * cp;
}

/* ---- скорость ведения ------------------------------------------------------ */

void iv_bowspeed_init(iv_bowspeed_t *b, float sample_rate, float leak_tau_s)
{
    b->grav[0] = b->grav[1] = b->grav[2] = 0.0f;
    b->started = false;
    b->vh[0] = b->vh[1] = b->vh[2] = 0.0f;
    b->vv = 0.0f;
    b->dt = (sample_rate > 0.0f) ? 1.0f / sample_rate : 0.0f;
    b->leak = (leak_tau_s > 0.0f) ? expf(-b->dt / leak_tau_s) : 1.0f;
    b->grav_k = (IV_BOW_GRAV_TAU_S > 0.0f)
                    ? b->dt / (IV_BOW_GRAV_TAU_S + b->dt) : 1.0f;
    b->grav_k_rest = (IV_BOW_GRAV_REST_TAU_S > 0.0f)
                    ? b->dt / (IV_BOW_GRAV_REST_TAU_S + b->dt) : 1.0f;
    /* Усиление задано в 1/с², а смещение считается в °/с: отсюда и шаг, и
     * перевод радианов расхождения в градусы. */
    b->bias_k = IV_BOW_BIAS_GAIN * b->dt * RAD_TO_DEG;
    b->bias[0] = b->bias[1] = b->bias[2] = 0.0f;
    b->env_k = (IV_BOW_ENV_TAU_S > 0.0f) ? b->dt / (IV_BOW_ENV_TAU_S + b->dt) : 1.0f;
    b->angle_k = (IV_BOW_ANGLE_TAU_S > 0.0f)
                     ? b->dt / (IV_BOW_ANGLE_TAU_S + b->dt) : 1.0f;
    b->acc_env = 0.0f;
    b->acc_v = 0.0f;
    b->gyro_env = 0.0f;
    b->vv_env = 0.0f;
    b->vh_env = 0.0f;
    b->angle = 0.0f;
    b->still = 0;
    b->still_need = (int)(IV_BOW_REST_MS * sample_rate / 1000.0f);
    if (b->still_need < 1) {
        b->still_need = 1;
    }
}

float iv_bowspeed_update(iv_bowspeed_t *b, const float a[3], const float gyro[3],
                         float *angle_deg, bool *at_rest)
{
    /* Гироскоп с дослеженным смещением нуля. Дальше по функции используется
     * только он: на нём и поворот гравитации, и признак покоя — смещённый ноль
     * задирал бы огибающую и мешал объявить покой. */
    const float g[3] = { gyro[0] - b->bias[0],
                         gyro[1] - b->bias[1],
                         gyro[2] - b->bias[2] };

    /* Оценка гравитации в осях прибора. Первая выборка принимается как есть:
     * сходиться к ней с нуля значило бы секунду считать движением саму
     * гравитацию. */
    if (!b->started) {
        b->grav[0] = a[0];
        b->grav[1] = a[1];
        b->grav[2] = a[2];
        b->started = true;
    } else {
        /* Поворот вектора гироскопом. Прибор повернулся на ω·dt — значит
         * неподвижный в мире вектор в его осях повернулся на −ω·dt. При
         * 500 Гц угол за шаг мал, и малоугольного приближения хватает.
         *
         * Без этого гироскоп в компенсации гравитации не участвовал вовсе, и
         * поворот смычка целиком уходил в интеграл как настоящее ускорение.
         * Замерено: поворот на 20° за полсекунды загонял оценку скорости в
         * потолок, и она там оставалась — то есть инструмент начинал орать с
         * первой же смены струны, потому что смена струны и есть поворот. */
        const float wx = g[0] * DEG_TO_RAD * b->dt;
        const float wy = g[1] * DEG_TO_RAD * b->dt;
        const float wz = g[2] * DEG_TO_RAD * b->dt;
        const float gx = b->grav[0], gy = b->grav[1], gz = b->grav[2];
        b->grav[0] = gx - (wy * gz - wz * gy);
        b->grav[1] = gy - (wz * gx - wx * gz);
        b->grav[2] = gz - (wx * gy - wy * gx);

        /* Акселерометр правит медленно: его дело — не дать уплыть гироскопу,
         * а не поспевать за движением. Быстрая коррекция вернула бы ту самую
         * беду, ради которой фьюжн отсюда и убран: собственное ускорение
         * втягивалось бы в оценку «низа».
         *
         * Пока смычок стоит — правит быстро: собственного ускорения на покое
         * нет, втягиваться нечему, а накопленное за штрих снимается к началу
         * следующего. Покой тут прошлого шага: свой считается ниже, по этой же
         * оценке гравитации, и ждать его значило бы считать её дважды. */
        const float k = (b->still >= b->still_need) ? b->grav_k_rest : b->grav_k;
        for (int i = 0; i < 3; ++i) {
            b->grav[i] += k * (a[i] - b->grav[i]);
        }
    }

    /* Единичная вертикаль и её величина. Смычок в свободном падении сюда не
     * попадёт, но проверка стоит копейки, а деление на ноль — нет. */
    const float gmag = vec_mag(b->grav);
    if (gmag < 0.1f) {
        b->acc_v = 0.0f;
        if (angle_deg) { *angle_deg = b->angle; }
        if (at_rest)   { *at_rest = false; }
        return 0.0f;
    }
    const float up[3] = { b->grav[0] / gmag, b->grav[1] / gmag, b->grav[2] / gmag };

    /* Собственное ускорение и его разложение на горизонталь и вертикаль
     * (почему именно так — см. iv_motion.h). */
    const float lin[3] = { a[0] - b->grav[0], a[1] - b->grav[1], a[2] - b->grav[2] };
    const float lin_v = lin[0] * up[0] + lin[1] * up[1] + lin[2] * up[2];

    float acc_h[3];
    float ah2 = 0.0f;
    for (int i = 0; i < 3; ++i) {
        acc_h[i] = lin[i] - lin_v * up[i];
        ah2 += acc_h[i] * acc_h[i];
    }

    const float amag2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    const float vert2 = amag2 - ah2;
    const float acc_v = (vert2 > 0.0f ? sqrtf(vert2) : 0.0f) - gmag;
    b->acc_v = acc_v;

    b->vv = (b->vv + acc_v * G_MS2 * b->dt) * b->leak;
    for (int i = 0; i < 3; ++i) {
        b->vh[i] = (b->vh[i] + acc_h[i] * G_MS2 * b->dt) * b->leak;
    }

    /* Покой — по сглаженным модулям, а не по мгновенным: см. IV_BOW_ENV_TAU_S.
     * Признаки в «и», и вертикаль считается отдельно от горизонтали, потому
     * что видна чище (см. IV_BOW_REST_AMAG_G). */
    b->acc_env  += b->env_k * (sqrtf(ah2) - b->acc_env);
    b->gyro_env += b->env_k * (vec_mag(g) - b->gyro_env);
    const bool quiet = b->acc_env    < IV_BOW_REST_ACCEL_G
                    && b->gyro_env   < IV_BOW_REST_GYRO_DPS
                    && fabsf(acc_v)  < IV_BOW_REST_AMAG_G;

    if (quiet) {
        if (b->still < b->still_need) {
            b->still++;
        }
    } else {
        /* Из покоя выходим мгновенно: задержка здесь — это задержка атаки,
         * а она в бюджете. Копится же покой постепенно, чтобы середина
         * штриха, где ускорение проходит через ноль, не сошла за остановку. */
        b->still = 0;
    }

    const bool resting = (b->still >= b->still_need);
    if (resting) {
        /* ZUPT: дрейфу негде накопиться. */
        b->vh[0] = b->vh[1] = b->vh[2] = 0.0f;
        b->vv = 0.0f;

        /* Дослеживание нуля гироскопа (см. IV_BOW_BIAS_GAIN). Смычок стоит —
         * значит акселерометр показывает чистую гравитацию, и расхождение с
         * ним оценки гравитации есть ровно накопленная ошибка гироскопа.
         *
         * Мера расхождения — векторное произведение двух единичных векторов:
         * для малых углов это сам вектор поворота, которым одна оценка «низа»
         * отличается от другой, а его направление указывает ось ошибки. От
         * разности «гироскоп минус ноль» это отличается тем, что настоящий
         * медленный поворот даёт здесь ноль: оценка за ним поспевает, спорить
         * не о чем. Двигает смещение только несогласие. */
        const float amag = sqrtf(amag2);
        if (amag > 0.1f) {
            const float ax = a[0] / amag, ay = a[1] / amag, az = a[2] / amag;
            const float err[3] = { up[1] * az - up[2] * ay,
                                   up[2] * ax - up[0] * az,
                                   up[0] * ay - up[1] * ax };
            for (int i = 0; i < 3; ++i) {
                b->bias[i] = iv_clampf(b->bias[i] + b->bias_k * err[i],
                                       -IV_BOW_BIAS_MAX_DPS, IV_BOW_BIAS_MAX_DPS);
            }
        }
    }
    if (at_rest) {
        *at_rest = resting;
    }

    const float horiz = vec_mag(b->vh);
    const float speed = sqrtf(horiz * horiz + b->vv * b->vv);

    /* Наклон линии движения к горизонту, со знаком.
     *
     * Знак — от того, куда движение идёт относительно самого смычка: ось X
     * прибора продольная, её знак и говорит, к концу смычок сейчас едет или к
     * колодке. Приводим вектор к «вперёд» — и обратный ход перестаёт
     * отличаться от прямого: на нём переворачиваются и продольная
     * составляющая, и вертикальная, множитель их разворот и снимает.
     *
     * Накапливаются составляющие, а не угол: у разворота обе проходят через
     * ноль, и мгновенное отношение там показывает что угодно. */
    const float along = b->vh[0] + b->vv * up[0]; /* вдоль смычка, со знаком */
    if (speed > 0.0f && fabsf(along) >= IV_BOW_ALONG_MIN * speed) {
        const float fwd = (along >= 0.0f) ? 1.0f : -1.0f;
        b->vv_env += b->angle_k * (fwd * b->vv - b->vv_env);
        b->vh_env += b->angle_k * (horiz - b->vh_env);
    }

    /* Движение поперёк смычка сюда не попадает: там «вперёд» не определено, и
     * угол остаётся прежним, а не скачет от шума. */
    if (fabsf(b->vv_env) + b->vh_env > IV_BOW_ANGLE_MIN_MS) {
        b->angle = atan2f(b->vv_env, b->vh_env) * RAD_TO_DEG;
    }
    if (angle_deg) {
        *angle_deg = b->angle;
    }

    return iv_clampf(speed / IV_BOW_FULL_SPEED_MS, 0.0f, 1.0f);
}

/* ---- грубый признак движения ---------------------------------------------- */

bool iv_motion_is_moving(const float a[3], const float g[3])
{
    const float am = vec_mag(a);
    return fabsf(am - 1.0f) > IV_MOVE_ACCEL_G || vec_mag(g) > IV_MOVE_GYRO_DPS;
}
