#include "iv_test.h"
#include "iv_link_stats.h"

#define US_MS 1000LL

/* Обычный поток: пакеты идут подряд, потерь нет. */
static void test_clean_stream(void)
{
    iv_link_stats_t s;
    iv_link_stats_reset(&s);

    int64_t t = 0;
    for (uint16_t seq = 0; seq < 100; ++seq) {
        IV_CHECK(!iv_link_stats_on_packet(&s, seq, t));
        t += 5 * US_MS; /* 200 Гц */
        IV_CHECK(!iv_link_stats_tick(&s, t));
    }

    IV_CHECK(s.received == 100);
    IV_CHECK(s.lost == 0);
    IV_CHECK(s.outages == 0);
    IV_CHECK(s.online);
    IV_CHECK_NEAR(iv_link_loss_ratio(&s), 0.0, 1e-9);
}

/* Дырка в номерах при живой связи — это потери, и они должны считаться. */
static void test_losses(void)
{
    iv_link_stats_t s;
    iv_link_stats_reset(&s);

    iv_link_stats_on_packet(&s, 10, 0);
    iv_link_stats_on_packet(&s, 13, 5 * US_MS);  /* 11 и 12 не доехали */
    IV_CHECK(s.lost == 2);
    IV_CHECK(s.received == 2);
    IV_CHECK_NEAR(iv_link_loss_ratio(&s), 2.0 / 4.0, 1e-6);

    /* Переполнение номера случается каждые 5.5 минут на 200 Гц и не должно
     * выглядеть как обрыв. */
    iv_link_stats_reset(&s);
    iv_link_stats_on_packet(&s, 65534, 0);
    iv_link_stats_on_packet(&s, 65535, 5 * US_MS);
    iv_link_stats_on_packet(&s, 0, 10 * US_MS);
    iv_link_stats_on_packet(&s, 1, 15 * US_MS);
    IV_CHECK(s.lost == 0);
    IV_CHECK(s.resets == 0);
}

/* Watchdog: молчание дольше порога — явное событие, ровно одно. */
static void test_watchdog(void)
{
    iv_link_stats_t s;
    iv_link_stats_reset(&s);

    iv_link_stats_on_packet(&s, 1, 0);
    IV_CHECK(s.online);

    IV_CHECK(!iv_link_stats_tick(&s, IV_LINK_TIMEOUT_MS * US_MS)); /* ровно порог — ещё живы */
    IV_CHECK(iv_link_stats_tick(&s, IV_LINK_TIMEOUT_MS * US_MS + 1));
    IV_CHECK(!s.online);
    IV_CHECK(s.outages == 1);
    /* Повторные вызовы не должны множить событие. */
    IV_CHECK(!iv_link_stats_tick(&s, 10 * IV_LINK_TIMEOUT_MS * US_MS));
    IV_CHECK(s.outages == 1);
}

/* Пауза в связи не должна записываться в потери: иначе один выход из зоны
 * приёма испортит измерение качества канала. */
static void test_outage_is_not_loss(void)
{
    iv_link_stats_t s;
    iv_link_stats_reset(&s);

    iv_link_stats_on_packet(&s, 100, 0);
    IV_CHECK(iv_link_stats_tick(&s, 500 * US_MS));

    /* Пять секунд молчания — тысяча неотправленных пакетов. */
    const bool back = iv_link_stats_on_packet(&s, (uint16_t)(100 + 1000), 5000 * US_MS);
    IV_CHECK(back);
    IV_CHECK(s.online);
    IV_CHECK(s.lost == 0);
    IV_CHECK(s.outages == 1);
    IV_CHECK(s.received == 2);
}

/* Перезапуск смычка при живой связи: номер прыгает так, как потеря прыгнуть
 * не может. Это не потери, это сброс счётчика. */
static void test_reset_detection(void)
{
    iv_link_stats_t s;
    iv_link_stats_reset(&s);

    iv_link_stats_on_packet(&s, 30000, 0);
    iv_link_stats_on_packet(&s, 0, 5 * US_MS);
    IV_CHECK(s.resets == 1);
    IV_CHECK(s.lost == 0);

    /* А разрыв на границе порога всё ещё считается потерями. */
    iv_link_stats_reset(&s);
    iv_link_stats_on_packet(&s, 0, 0);
    iv_link_stats_on_packet(&s, IV_SEQ_GAP_MAX + 1, 5 * US_MS);
    IV_CHECK(s.resets == 0);
    IV_CHECK(s.lost == IV_SEQ_GAP_MAX);
}

/* Первый пакет — не «связь восстановилась»: сообщать о восстановлении того,
 * чего ещё не было, значит врать в логе. */
static void test_first_packet(void)
{
    iv_link_stats_t s;
    iv_link_stats_reset(&s);
    IV_CHECK(!iv_link_stats_on_packet(&s, 0, 0));
    IV_CHECK(s.online);
}

int main(void)
{
    test_clean_stream();
    test_losses();
    test_watchdog();
    test_outage_is_not_loss();
    test_reset_detection();
    test_first_packet();
    IV_TEST_END();
}
