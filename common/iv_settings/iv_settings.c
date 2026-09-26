#include "iv_settings.h"

#include <math.h>
#include <string.h>

/* 'I','V','S' и номер поколения формата. Меняется только если старую запись
 * читать больше нельзя; version меняется, когда состав полей поправим, но
 * прочитать старое ещё можно. */
#define IV_SETTINGS_MAGIC   0x53564901u
/* 2: середина раскладки стала углом к горизонту вместо крена. Число то же по
 * смыслу, но снятое с другой оси, и старая калибровка увела бы струны — такую
 * запись честнее не прочитать вовсе, чем прочитать неверно. */
#define IV_SETTINGS_VERSION 3u

/* Версия, которую ещё читаем, но правим при чтении.
 *
 * Между второй и третьей версией поменялся не формат, а СМЫСЛ поля
 * angle_center: оси смычка развернули (bsp_bow_imu.c), и угол линии движения
 * сменил знак. Записанная до переворота середина раскладки описывает прежние
 * оси и теперь просто неверна — с нею горизонтальное ведение попадает не на
 * границу D/A, а на крайнюю струну.
 *
 * Отвергать всю запись из-за одного поля незачем: строй, громкость, яркость и
 * полутоны переворот не затронул, и терять их обидно. Поэтому запись второй
 * версии читается целиком, а середина берётся умолчанием — то есть ноль,
 * «горизонтальное ведение приходится на границу D/A». Это ровно то, с чего
 * настройку и надо начинать заново. */
#define IV_SETTINGS_VERSION_PREV 2u

/* Запись фиксированной раскладки, а не сама структура настроек: у структуры
 * состав полей и выравнивание — дело компилятора, а во flash лежит формат. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;       /* длина всей записи: часть проверки при чтении */
    float    a4_hz;
    float    angle_center;
    uint8_t  volume;
    uint8_t  brightness;
    uint8_t  snap;
    uint8_t  pad;        /* до кратности четырём: место для следующего флага */
} iv_settings_blob_t;

_Static_assert(sizeof(iv_settings_blob_t) == 20, "формат записи изменился — обнови версию");

void iv_settings_defaults(iv_settings_t *s)
{
    s->a4_hz      = IV_A4_DEFAULT;
    s->angle_center = IV_ANGLE_CENTER_DEFAULT;
    s->volume     = IV_VOLUME_DEFAULT;
    s->brightness = IV_BRIGHT_DEFAULT;
    s->snap       = false;
}

/* Число вне границ или вовсе не число заменяется на fallback, а не на
 * ближайшую границу: NaN в строе нельзя «прижать» — у него нет стороны. */
static bool fix_float(float *v, float lo, float hi, float fallback)
{
    if (!isfinite(*v)) {
        *v = fallback;
        return true;
    }
    if (*v < lo) { *v = lo; return true; }
    if (*v > hi) { *v = hi; return true; }
    return false;
}

static bool fix_u8(uint8_t *v, uint8_t lo, uint8_t hi)
{
    if (*v < lo) { *v = lo; return true; }
    if (*v > hi) { *v = hi; return true; }
    return false;
}

bool iv_settings_sanitize(iv_settings_t *s)
{
    bool fixed = false;
    fixed |= fix_float(&s->a4_hz, IV_A4_MIN, IV_A4_MAX, IV_A4_DEFAULT);
    fixed |= fix_float(&s->angle_center, -IV_ANGLE_CENTER_MAX, IV_ANGLE_CENTER_MAX,
                       IV_ANGLE_CENTER_DEFAULT);
    fixed |= fix_u8(&s->volume, IV_VOLUME_MIN, IV_VOLUME_MAX);
    fixed |= fix_u8(&s->brightness, IV_BRIGHT_MIN, IV_BRIGHT_MAX);
    return fixed;
}

bool iv_settings_equal(const iv_settings_t *a, const iv_settings_t *b)
{
    /* Побайтового сравнения структур избегаем сознательно: между полями есть
     * невидимые байты выравнивания, и их содержимое не определено. */
    return a->a4_hz == b->a4_hz
        && a->angle_center == b->angle_center
        && a->volume == b->volume
        && a->brightness == b->brightness
        && a->snap == b->snap;
}

size_t iv_settings_blob_size(void)
{
    return sizeof(iv_settings_blob_t);
}

size_t iv_settings_encode(const iv_settings_t *s, void *buf, size_t len)
{
    if (len < sizeof(iv_settings_blob_t)) {
        return 0;
    }
    const iv_settings_blob_t b = {
        .magic       = IV_SETTINGS_MAGIC,
        .version     = IV_SETTINGS_VERSION,
        .size        = (uint16_t)sizeof(iv_settings_blob_t),
        .a4_hz       = s->a4_hz,
        .angle_center = s->angle_center,
        .volume      = s->volume,
        .brightness  = s->brightness,
        .snap        = s->snap ? 1u : 0u,
        .pad         = 0,
    };
    memcpy(buf, &b, sizeof(b));
    return sizeof(b);
}

bool iv_settings_decode(iv_settings_t *s, const void *buf, size_t len)
{
    if (len != sizeof(iv_settings_blob_t)) {
        return false;
    }
    iv_settings_blob_t b;
    memcpy(&b, buf, sizeof(b));

    if (b.magic != IV_SETTINGS_MAGIC || b.size != sizeof(iv_settings_blob_t)) {
        return false;
    }
    const bool legacy = (b.version == IV_SETTINGS_VERSION_PREV);
    if (b.version != IV_SETTINGS_VERSION && !legacy) {
        return false;
    }

    s->a4_hz      = b.a4_hz;
    s->angle_center = b.angle_center;
    s->volume     = b.volume;
    s->brightness = b.brightness;
    s->snap       = b.snap != 0;

    /* Середина прежней версии снята в прежних осях смычка — см.
     * IV_SETTINGS_VERSION_PREV. Остальные поля переворот не затронул. */
    if (legacy) {
        s->angle_center = IV_ANGLE_CENTER_DEFAULT;
    }

    iv_settings_sanitize(s);
    return true;
}
