/* Настройки: умолчания, границы, формат записи.
 *
 * Проверяется здесь не «сохранилось ли», а то, что нельзя проверить на
 * плате иначе как порчей flash: что испорченная запись не пролезет, а
 * пролезшая не сделает инструмент неиграбельным. Ошибка тут выглядит на
 * железе как «инструмент после включения молчит» — и искать её будут где
 * угодно, только не в настройках. */
#include "iv_settings.h"
#include "iv_test.h"

#include <string.h>

static void test_defaults(void)
{
    iv_settings_t s;
    iv_settings_defaults(&s);

    IV_CHECK_NEAR(s.a4_hz, IV_A4_DEFAULT, 0.001);
    IV_CHECK(s.volume == IV_VOLUME_DEFAULT);
    IV_CHECK(s.brightness == IV_BRIGHT_DEFAULT);
    IV_CHECK(s.snap == false);
    IV_CHECK_NEAR(s.angle_center, IV_ANGLE_CENTER_DEFAULT, 0.001);

    /* Умолчания обязаны быть внутри границ — иначе первое же сохранение
     * поменяло бы их за спиной пользователя. */
    iv_settings_t copy = s;
    IV_CHECK(iv_settings_sanitize(&copy) == false);
    IV_CHECK(iv_settings_equal(&s, &copy));
}

static void test_sanitize_clamps(void)
{
    iv_settings_t s;
    iv_settings_defaults(&s);
    s.a4_hz      = 1000.0f;
    s.volume     = 250;
    s.brightness = 0;
    s.angle_center = -400.0f;

    IV_CHECK(iv_settings_sanitize(&s) == true);
    IV_CHECK_NEAR(s.a4_hz, IV_A4_MAX, 0.001);
    IV_CHECK(s.volume == IV_VOLUME_MAX);
    IV_CHECK(s.brightness == IV_BRIGHT_MIN);
    IV_CHECK_NEAR(s.angle_center, -IV_ANGLE_CENTER_MAX, 0.001);
}

/* Главный случай: не число. Прижать NaN к границе нельзя — у него нет
 * стороны, — поэтому он обязан заменяться умолчанием. Пропустить его значило
 * бы получить ноту частотой NaN, то есть бесконечную линию задержки в
 * волноводе и тишину. */
static void test_sanitize_nan(void)
{
    iv_settings_t s;
    iv_settings_defaults(&s);
    s.a4_hz = 0.0f / 0.0f;
    s.angle_center = 1.0f / 0.0f;

    IV_CHECK(iv_settings_sanitize(&s) == true);
    IV_CHECK_NEAR(s.a4_hz, IV_A4_DEFAULT, 0.001);
    IV_CHECK_NEAR(s.angle_center, IV_ANGLE_CENTER_DEFAULT, 0.001);
}

static void test_roundtrip(void)
{
    iv_settings_t src;
    iv_settings_defaults(&src);
    src.a4_hz      = 442.0f;
    src.volume     = 60;
    src.brightness = 30;
    src.snap       = true;
    src.angle_center = -12.5f;

    uint8_t buf[64];
    const size_t len = iv_settings_encode(&src, buf, sizeof(buf));
    IV_CHECK(len == iv_settings_blob_size());

    iv_settings_t dst;
    iv_settings_defaults(&dst);
    IV_CHECK(iv_settings_decode(&dst, buf, len) == true);
    IV_CHECK(iv_settings_equal(&src, &dst));
}

static void test_encode_rejects_small_buffer(void)
{
    iv_settings_t s;
    iv_settings_defaults(&s);
    uint8_t small[4];
    IV_CHECK(iv_settings_encode(&s, small, sizeof(small)) == 0);
}

/* Мусор в NVS не должен превращаться в настройки. Проверяются все три
 * рубежа: длина, magic и версия. */
static void test_decode_rejects_garbage(void)
{
    iv_settings_t src;
    iv_settings_defaults(&src);
    src.volume = 55;

    uint8_t buf[64];
    const size_t len = iv_settings_encode(&src, buf, sizeof(buf));

    iv_settings_t dst;
    iv_settings_defaults(&dst);
    const iv_settings_t untouched = dst;

    /* Короче и длиннее, чем надо. */
    IV_CHECK(iv_settings_decode(&dst, buf, len - 1) == false);
    IV_CHECK(iv_settings_decode(&dst, buf, len + 1) == false);

    /* Чужой magic. */
    uint8_t bad[64];
    memcpy(bad, buf, len);
    bad[0] ^= 0xFF;
    IV_CHECK(iv_settings_decode(&dst, bad, len) == false);

    /* Другая версия формата. */
    memcpy(bad, buf, len);
    bad[4] = 0x7F;
    IV_CHECK(iv_settings_decode(&dst, bad, len) == false);

    /* Ни одна из неудач не имеет права испортить то, что уже лежит. */
    IV_CHECK(iv_settings_equal(&dst, &untouched));
}

/* Запись с правильным заголовком, но невозможными значениями: так выглядит
 * настройка, сохранённая другой сборкой с другими границами. Пролезть она
 * обязана, но уже поправленной. */
static void test_decode_sanitizes(void)
{
    iv_settings_t src;
    iv_settings_defaults(&src);
    src.a4_hz  = 440.0f;
    src.volume = 100;

    uint8_t buf[64];
    const size_t len = iv_settings_encode(&src, buf, sizeof(buf));

    /* Поле a4_hz лежит сразу за заголовком из восьми байт. */
    const float insane = 12345.0f;
    memcpy(buf + 8, &insane, sizeof(insane));

    iv_settings_t dst;
    iv_settings_defaults(&dst);
    IV_CHECK(iv_settings_decode(&dst, buf, len) == true);
    IV_CHECK(dst.a4_hz >= IV_A4_MIN && dst.a4_hz <= IV_A4_MAX);
}

/* Запись, снятая до переворота осей смычка, читается целиком, но середина
 * раскладки из неё не берётся: она описывает прежние оси, и с нею
 * горизонтальное ведение попадает на крайнюю струну вместо границы D/A.
 *
 * Отвергать из-за неё всю запись нельзя — строй, громкость и яркость
 * переворот не затронул, и терять их незачем. */
static void test_legacy_record_drops_center(void)
{
    iv_settings_t src;
    iv_settings_defaults(&src);
    src.a4_hz       = 442.0f;
    src.volume      = 71;
    src.brightness  = 33;
    src.snap        = true;
    src.angle_center = 30.0f;

    uint8_t buf[32];
    const size_t len = iv_settings_encode(&src, buf, sizeof(buf));
    IV_CHECK(len > 0);

    /* Подменяем номер версии на прежний: поле лежит сразу за magic. */
    buf[4] = 2;
    buf[5] = 0;

    iv_settings_t got;
    iv_settings_defaults(&got);
    IV_CHECK(iv_settings_decode(&got, buf, len));

    /* Середина сброшена... */
    IV_CHECK_NEAR(got.angle_center, IV_ANGLE_CENTER_DEFAULT, 0.001);
    /* ...а всё остальное уцелело. */
    IV_CHECK_NEAR(got.a4_hz, 442.0, 0.001);
    IV_CHECK(got.volume == 71);
    IV_CHECK(got.brightness == 33);
    IV_CHECK(got.snap);
}

int main(void)
{
    test_defaults();
    test_sanitize_clamps();
    test_sanitize_nan();
    test_roundtrip();
    test_encode_rejects_small_buffer();
    test_decode_rejects_garbage();
    test_decode_sanitizes();
    test_legacy_record_drops_center();
    IV_TEST_END();
}
