/* Заряд по напряжению: монотонность кривой, края, «неизвестно».
 *
 * Проверять это на железе — значит разряжать батарею и смотреть на экран
 * час подряд, причём один раз для грифа и один для смычка. */
#include "iv_power.h"
#include "iv_test.h"

static void test_edges(void)
{
    /* Полный заряд и выше: сто процентов, а не переполнение. */
    IV_CHECK(iv_batt_percent(4200) == 100);
    IV_CHECK(iv_batt_percent(4250) == 100);

    /* Слишком высоко для батареи — значит меряется не она. */
    IV_CHECK(iv_batt_percent(5000) == IV_BATT_UNKNOWN);
    /* Ноль и обрыв делителя — тоже «неизвестно», а не «разряжено». Разница
     * содержательная: разряженное показывают красным, неизвестное молчит. */
    IV_CHECK(iv_batt_percent(0) == IV_BATT_UNKNOWN);
    IV_CHECK(iv_batt_percent(1000) == IV_BATT_UNKNOWN);

    /* Живая, но пустая банка: единица, а не ноль — ноль означает
     * «неизвестно». */
    IV_CHECK(iv_batt_percent(3300) == 1);
    IV_CHECK(iv_batt_percent(3200) == 1);
}

/* Кривая обязана быть монотонной: иначе показания скакали бы вверх на
 * разряде, а объяснить это пользователю нечем. */
static void test_monotonic(void)
{
    int prev = -1;
    for (int mv = 3300; mv <= 4200; mv += 10) {
        const int pct = iv_batt_percent((uint16_t)mv);
        IV_CHECK(pct >= prev);
        IV_CHECK(pct >= 1 && pct <= 100);
        prev = pct;
    }
}

/* Середина разряда у литиевой банки — почти полка, и кривая обязана это
 * учитывать. Линейный пересчёт по напряжению давал бы здесь около половины
 * заряда почти всё время, а потом падал бы с сорока процентов до нуля за
 * минуту. */
static void test_curve_is_not_linear(void)
{
    const int at_390 = iv_batt_percent(3900);
    const int at_370 = iv_batt_percent(3700);
    const int at_350 = iv_batt_percent(3500);

    /* От 3.9 до 3.7 В уходит 30 % заряда, а от 3.7 до 3.5 — только 25 %,
     * при том что напряжение падает одинаково. */
    IV_CHECK(at_390 - at_370 > at_370 - at_350);
}

static void test_thresholds(void)
{
    IV_CHECK(iv_batt_is_low(IV_BATT_LOW_PCT));
    IV_CHECK(!iv_batt_is_low(IV_BATT_LOW_PCT + 1));
    IV_CHECK(iv_batt_is_critical(IV_BATT_CRIT_PCT));
    IV_CHECK(!iv_batt_is_critical(IV_BATT_CRIT_PCT + 1));

    /* «Неизвестно» — не «разряжено»: иначе гриф без смычка кричал бы о
     * разряде чужой батареи. */
    IV_CHECK(!iv_batt_is_low(IV_BATT_UNKNOWN));
    IV_CHECK(!iv_batt_is_critical(IV_BATT_UNKNOWN));
}

int main(void)
{
    test_edges();
    test_monotonic();
    test_curve_is_not_linear();
    test_thresholds();
    IV_TEST_END();
}
