#include "iv_test.h"
#include "iv_params.h"

#include <pthread.h>
#include <string.h>

static void test_round_trip(void)
{
    const iv_control_params_t zero = {0};
    iv_params_t p;
    iv_params_init(&p, &zero);

    const iv_control_params_t src = {
        .freq_hz = { 196.0f, 293.66f, 440.0f, 659.26f },
        .bow_share = { 0.0f, 0.0f, 0.75f, 0.25f },
        .bow_velocity = 0.5f, .bow_force = 0.25f, .stamp_us = 12345,
    };
    iv_params_publish(&p, &src);

    iv_control_params_t dst = {0};
    IV_CHECK(iv_params_read(&p, &dst));
    for (int k = 0; k < IV_VOICES; ++k) {
        IV_CHECK(dst.freq_hz[k] == src.freq_hz[k]);
        IV_CHECK(dst.bow_share[k] == src.bow_share[k]);
    }
    IV_CHECK(dst.bow_velocity == 0.5f);
    IV_CHECK(dst.bow_force == 0.25f);
    IV_CHECK(dst.stamp_us == 12345);

    /* Счётчик за публикацию проходит через нечётное и возвращается в чётное. */
    IV_CHECK((p.seq & 1u) == 0);
}

/* Если писатель вытеснен посреди записи, читатель обязан сдаться, а не ждать:
 * в аудиопотоке ожидание дороже устаревших параметров. */
static void test_reader_gives_up(void)
{
    const iv_control_params_t init = { .freq_hz = { 220.0f } };
    iv_params_t p;
    iv_params_init(&p, &init);

    p.seq = 1; /* писатель «внутри» записи */

    iv_control_params_t dst = { .freq_hz = { -1.0f } };
    IV_CHECK(!iv_params_read(&p, &dst));
    IV_CHECK(dst.freq_hz[0] == -1.0f); /* прежнее значение не тронуто */
}

/* Главная проверка: на двух потоках рвущийся снимок не должен пролезать.
 *
 * Писатель публикует согласованные наборы, где все поля выражены одним числом;
 * читатель проверяет, что поля согласованы между собой. Односторонний тест на
 * одном потоке такую ошибку не увидит в принципе — здесь именно поэтому нити. */
#define STRESS_ROUNDS 300000

static volatile int s_stop;

static void *writer(void *arg)
{
    iv_params_t *p = arg;
    for (int64_t k = 1; k <= STRESS_ROUNDS; ++k) {
        /* Массивы заполняются тем же числом: снимок стал больше, и рвётся он
         * теперь как раз посреди них. */
        iv_control_params_t src = {
            .bow_velocity = (float)(k % 1000),
            .bow_force = (float)(k % 1000),
            .stamp_us   = k % 1000,
            .from_bow   = (k % 2) == 0,
        };
        for (int i = 0; i < IV_VOICES; ++i) {
            src.freq_hz[i]   = (float)(k % 1000);
            src.bow_share[i] = (float)(k % 1000);
        }
        iv_params_publish(p, &src);
    }
    s_stop = 1;
    return NULL;
}

static void test_no_torn_reads(void)
{
    /* Начальное значение — это набор для k = 0, иначе читатель поймает на
     * несогласованности не разорванный снимок, а собственную инициализацию:
     * метка 0 чётная, значит и признак обязан быть true. */
    const iv_control_params_t start = { .from_bow = true };
    iv_params_t p;
    iv_params_init(&p, &start);

    pthread_t th;
    IV_CHECK(pthread_create(&th, NULL, writer, &p) == 0);

    int torn = 0, reads = 0, refusals = 0;
    while (!s_stop) {
        iv_control_params_t dst;
        if (!iv_params_read(&p, &dst)) {
            refusals++;
            continue;
        }
        reads++;
        bool ok = dst.bow_velocity == dst.bow_force
               && dst.bow_velocity == (float)dst.stamp_us
               && dst.from_bow == ((dst.stamp_us % 2) == 0);
        for (int i = 0; i < IV_VOICES; ++i) {
            ok = ok && dst.freq_hz[i] == dst.bow_velocity
                    && dst.bow_share[i] == dst.bow_velocity;
        }
        if (!ok) {
            torn++;
        }
    }
    pthread_join(th, NULL);

    IV_CHECK(torn == 0);
    IV_CHECK(reads > 0); /* иначе тест ничего не проверил */
    (void)refusals;      /* отказы законны и ошибкой не считаются */
}

int main(void)
{
    test_round_trip();
    test_reader_gives_up();
    test_no_torn_reads();
    IV_TEST_END();
}
