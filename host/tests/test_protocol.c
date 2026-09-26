#include "iv_test.h"
#include "iv_protocol.h"
#include <string.h>

/* Размер и раскладка пакета — контракт между двумя прошивками. Если он поедет
 * молча, гриф будет читать мусор; тест ловит это на сборке. */
static void test_layout(void)
{
    IV_CHECK(sizeof(iv_bow_packet_t) == 20);
    IV_CHECK(sizeof(iv_bow_packet_t) <= 250); /* лимит полезной нагрузки ESP-NOW */

    iv_bow_packet_t p;
    iv_bow_packet_init(&p, 42, 123456);
    IV_CHECK(p.magic == IV_PROTO_MAGIC);
    IV_CHECK(p.version == IV_PROTO_VERSION);
    IV_CHECK(p.seq == 42);
    IV_CHECK(p.t_us == 123456);
    IV_CHECK(p.flags == 0);
    IV_CHECK(p.speed == 0);
    IV_CHECK(p.move_angle == 0);
}

static void test_validation(void)
{
    iv_bow_packet_t p;
    iv_bow_packet_init(&p, 1, 0);
    IV_CHECK(iv_bow_packet_valid(&p, sizeof(p)));

    IV_CHECK(!iv_bow_packet_valid(NULL, sizeof(p)));
    IV_CHECK(!iv_bow_packet_valid(&p, sizeof(p) - 1)); /* обрезанный */
    IV_CHECK(!iv_bow_packet_valid(&p, sizeof(p) + 1)); /* длиннее */

    iv_bow_packet_t bad = p;
    bad.magic = 0x1234;                                 /* чужой трафик */
    IV_CHECK(!iv_bow_packet_valid(&bad, sizeof(bad)));

    bad = p;
    bad.version = IV_PROTO_VERSION + 1;                 /* рассинхрон версий */
    IV_CHECK(!iv_bow_packet_valid(&bad, sizeof(bad)));
}

/* Валидация не должна требовать выравнивания: буфер приёмника ESP-NOW
 * выровнен не обязательно. */
static void test_unaligned(void)
{
    unsigned char buf[sizeof(iv_bow_packet_t) + 1];
    iv_bow_packet_t p;
    iv_bow_packet_init(&p, 7, 0);
    memcpy(buf + 1, &p, sizeof(p));
    IV_CHECK(iv_bow_packet_valid(buf + 1, sizeof(p)));
}

/* Счётчик потерь обязан переживать переполнение uint16: на 200 Гц оно
 * случается каждые пять с половиной минут. */
static void test_seq_gap(void)
{
    IV_CHECK(iv_seq_gap(10, 11) == 0);      /* подряд */
    IV_CHECK(iv_seq_gap(10, 13) == 2);      /* два потеряно */
    IV_CHECK(iv_seq_gap(65535, 0) == 0);    /* переполнение, потерь нет */
    IV_CHECK(iv_seq_gap(65534, 2) == 3);    /* переполнение с потерями */
}

/* Обратный канал: тот же заголовок, другая длина. Пакет смычка не должен
 * проходить проверку ответа и наоборот — иначе перепутанные роли дадут
 * молчаливый разбор мусора. */
static void test_neck_packet(void)
{
    iv_neck_packet_t n;
    iv_neck_packet_init(&n, IV_NECK_MSG_ECHO, 999, 250);
    IV_CHECK(sizeof(n) == 10);
    IV_CHECK(n.magic == IV_PROTO_MAGIC);
    IV_CHECK(n.type == IV_NECK_MSG_ECHO);
    IV_CHECK(n.echo_t_us == 999);
    IV_CHECK(n.proc_us == 250);
    IV_CHECK(iv_neck_packet_valid(&n, sizeof(n)));

    /* Время обработки насыщается: 16 бит — это 65 мс, и если гриф провозился
     * дольше, точное число уже ничего не добавит. */
    iv_neck_packet_init(&n, IV_NECK_MSG_ECHO, 0, 1000000);
    IV_CHECK(n.proc_us == 65535);

    iv_bow_packet_t p;
    iv_bow_packet_init(&p, 1, 0);
    IV_CHECK(!iv_neck_packet_valid(&p, sizeof(p)));
    IV_CHECK(!iv_bow_packet_valid(&n, sizeof(n)));
}

/* Упаковка в целые. Проверяем и точность обратного хода, и насыщение:
 * на резком движении величина обязана упереться в предел, а не сменить знак. */
static void test_packing(void)
{
    /* Углы: сотые доли градуса. */
    IV_CHECK_NEAR(iv_unpack_angle(iv_pack_angle(0.0f)), 0.0, 1e-6);
    IV_CHECK_NEAR(iv_unpack_angle(iv_pack_angle(37.42f)), 37.42, 0.005);
    IV_CHECK_NEAR(iv_unpack_angle(iv_pack_angle(-92.71f)), -92.71, 0.005);
    IV_CHECK(iv_pack_angle(200.0f) == iv_pack_angle(180.0f));
    IV_CHECK(iv_pack_angle(-200.0f) == iv_pack_angle(-180.0f));
    IV_CHECK(iv_pack_angle(180.0f) == 18000);   /* влезает в int16 */
    IV_CHECK(iv_pack_angle(-180.0f) == -18000);

    /* Скорость смычка нормирована: шаг 1e-4 мельче любого слышимого. Знака у
     * неё в пакете больше нет — шлётся модуль, — но упаковка знак держит:
     * поле общее с прочими нормированными величинами. */
    IV_CHECK_NEAR(iv_unpack_vel(iv_pack_vel(0.3456f)), 0.3456, 1e-4);
    IV_CHECK_NEAR(iv_unpack_vel(iv_pack_vel(-0.9999f)), -0.9999, 1e-4);
    IV_CHECK(iv_pack_vel(5.0f) == 10000);
    IV_CHECK(iv_pack_vel(-5.0f) == -10000);

    /* Модуль ускорения: милли-g, без знака. */
    IV_CHECK_NEAR(iv_unpack_accel(iv_pack_accel(1.0f)), 1.0, 1e-3);
    IV_CHECK_NEAR(iv_unpack_accel(iv_pack_accel(8.125f)), 8.125, 1e-3);
    IV_CHECK(iv_pack_accel(-1.0f) == 0);
    IV_CHECK(iv_pack_accel(1e6f) == 65535);
}

int main(void)
{
    test_layout();
    test_validation();
    test_unaligned();
    test_seq_gap();
    test_neck_packet();
    test_packing();
    IV_TEST_END();
}
