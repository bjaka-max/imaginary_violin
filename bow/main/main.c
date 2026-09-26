/* Смычок — фаза 6: энергосбережение и заряд.
 *
 * Оценка движения не менялась (фаза 5): ориентация фьюжном, скорость ведения
 * интегрированием с обнулением на остановках, калибровка покоя в NVS.
 * Добавилось то, без чего инструмент упирается не в звук, а в батарею:
 * смычок замечает, что его отложили, и перестаёт тратить эфир и процессор,
 * а заряд отдаёт грифу в пакете.
 *
 * Три режима и переходы между ними:
 *
 *   ИГРА     опрос 500 Гц, передача 200 Гц. Всё как в фазе 5.
 *   ДРЁМА    смычок лежит пять секунд. Опрос 25 Гц, датчик 50 Гц, передача
 *            20 Гц с флагом IDLE. Связь жива, заряд идёт, гриф молчит.
 *   СОН      смычок лежит две минуты. Радио выключено, процессор в light
 *            sleep, просыпается четыре раза в секунду только чтобы спросить
 *            датчик, шевелят ли его.
 *
 * Обратно в игру — от первого же движения; из дрёмы мгновенно, из сна за
 * время подъёма Wi-Fi (порядка сотни миллисекунд). Поэтому пороги такие
 * разные: дрёма дешева и незаметна, сон дорог на выходе и потому объявляется
 * минутами.
 *
 * Почему движение считается здесь, а не на грифе: скорость нужна каждую
 * выборку, то есть 500 раз в секунду, а в эфир уходит 200 пакетов. Считать на
 * грифе значило бы либо гнать сырые данные вдвое чаще, либо интегрировать по
 * прореженным — а интегрирование по прореженным врёт. */
#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "bsp_bow.h"
#include "iv_protocol.h"
#include "iv_link.h"
#include "iv_motion.h"
#include "iv_power.h"

static const char *TAG = "bow";

/* Постоянные времени оценки движения.
 *
 * Ориентация: полсекунды — компромисс между быстрым откликом гироскопа и
 * тем, как скоро гравитация вычистит его дрейф.
 * Утечка интегратора скорости: секунда. Короче — сама съедает длинный штрих,
 * длиннее — не успевает съесть дрейф между остановками. */
#define ORIENT_TAU_S 0.5f
#define SPEED_LEAK_S 1.0f

/* Калибровка при включении: секунда неподвижности. Держать смычок в это время
 * не надо — достаточно положить. */
#define CALIB_SAMPLES (BSP_BOW_IMU_HZ)

/* Пороги энергосбережения.
 *
 * Пять секунд до дрёмы — это заметно дольше самой длинной паузы внутри пьесы,
 * но короче, чем время, за которое инструмент откладывают. Две минуты до сна —
 * порог «ушёл и не вернулся»: выход из сна стоит подъёма Wi-Fi, и платить эту
 * сотню миллисекунд за каждую паузу между частями было бы обидно. */
#define DROWSY_AFTER_MS  5000
#define SLEEP_AFTER_MS   120000

/* Передача в дрёме. Реже нельзя: гриф считает связь потерянной через
 * IV_LINK_TIMEOUT_MS, а лежащий смычок не должен выглядеть пропавшим —
 * заряд с него ещё нужен. */
#define DROWSY_TX_HZ     20
_Static_assert(1000 / DROWSY_TX_HZ < IV_LINK_TIMEOUT_MS,
               "в дрёме гриф будет считать смычок пропавшим");

/* Как часто во сне просыпаться и спрашивать датчик. Четверть секунды: столько
 * длится задержка от «взял смычок» до «инструмент ожил», и на фоне подъёма
 * Wi-Fi она незаметна. */
#define SLEEP_POLL_MS    250

/* Как часто читать контроллер питания. Раз в секунду: заряд меняется
 * минутами, а каждая транзакция I2C стоит места в горячем пути. */
#define BATT_PERIOD_MS   1000

#define NVS_NAMESPACE "iv_bow"
#define NVS_KEY_CALIB "calib"

typedef enum {
    BOW_MODE_ACTIVE = 0,
    BOW_MODE_DROWSY,
    BOW_MODE_SLEEP,
} bow_mode_t;

/* Готовая телеметрия: пишет задача опроса (500 Гц), читает задача передачи
 * (200 Гц). Частоты не кратны, поэтому это именно снимок, а не очередь:
 * передатчику нужно последнее известное состояние, а не история.
 *
 * Считается тоже здесь, в задаче опроса: интегрировать надо каждую выборку,
 * а не каждый пакет. */
typedef struct {
    float speed;      /* модуль скорости, 0..1 */
    float move_angle; /* угол линии движения к горизонту, 0..90° */
    float tilt;       /* наклон корпуса смычка, градусы — только на диагностику */
    float amag;       /* модуль ускорения, g */
    bool  at_rest;

    /* Дальше — только для лога. В пакет не идёт ничего из этого: это то, из
     * чего оценка движения делает свои выводы, и смотреть на это приходится
     * всякий раз, когда инструмент ведёт себя не так, как ждёшь. */
    float acc[3];     /* ускорение по осям после калибровки, g */
    float grav[3];    /* оценка гравитации (медленная часть показаний), g */
    float amag_peak;  /* наибольшее |‌|a| - g| за секунду: видно дрожь руки */
    float env_h;      /* сглаженная горизонталь собственного ускорения, g */
    float env_v;      /* вертикаль его же — ровно та, по которой идёт решение */
    float env_gyro;   /* сглаженный модуль угловой скорости, °/с */
    float bias[3];    /* дослеженный ноль гироскопа, °/с */
    int   still;      /* счётчик покоя */
    int   still_need;
} bow_state_t;

static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static bow_state_t  s_state;
static bool         s_state_valid;

/* Калибровка живёт рядом с задачей опроса, которая одна её и правит.
 *
 * s_calibrating означает не «идёт калибровка», а «поправки нет вообще, данным
 * верить нельзя»: по этому флагу гриф молчит начисто. Пока в NVS лежит
 * прошлая калибровка, флаг снят — она и работает, а свежая набирается фоном и
 * заменяет её, когда смычок положат.
 *
 * Разница не косметическая. Взводить флаг до конца свежей калибровки значило
 * бы молчать, пока смычок не положат неподвижно: iv_calib_feed начинает счёт
 * заново от любого шевеления, и в руке секунда подряд может не набраться
 * вовсе — дрожь руки как раз того порядка, что и допуск. Инструмент молчал бы
 * бесконечно, и по нему нельзя было бы понять, почему. */
static iv_calib_t s_calib;
static volatile bool s_calibrating = true;

/* Режим питания. Пишет задача опроса, читает задача передачи: одно слово,
 * рвущегося чтения здесь быть не может. */
static volatile bow_mode_t s_mode = BOW_MODE_ACTIVE;
static TaskHandle_t        s_tx_task;

/* Задача передачи встала и больше не трогает радио. Нужно ровно для одного:
 * выключать радио можно только после того, как передатчик это заметил, иначе
 * он успеет отправить пакет в уже остановленный Wi-Fi и запишет себе ошибку,
 * которой не было. */
static volatile bool s_tx_parked;

/* Заряд. Пишет задача передачи (она же и опрашивает контроллер), читает она
 * же и лог. */
static volatile uint8_t s_batt_pct;
static volatile bool    s_usb;
static volatile uint16_t s_batt_mv;

/* ---- калибровка в NVS ------------------------------------------------------
 *
 * Поправки переживают перезагрузку: смещение нуля гироскопа у конкретного
 * экземпляра почти не меняется, и заставлять класть смычок при каждом
 * включении незачем. Свежая калибровка всё равно снимается при старте — эта
 * лишь работает, пока она набирается. */

static void calib_save(const iv_calib_t *c)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_set_blob(h, NVS_KEY_CALIB, c, sizeof(*c)) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

static bool calib_load(iv_calib_t *c)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = sizeof(*c);
    const esp_err_t err = nvs_get_blob(h, NVS_KEY_CALIB, c, &len);
    nvs_close(h);
    return err == ESP_OK && len == sizeof(*c) && c->valid;
}

/* ---- режимы питания --------------------------------------------------------- */

/* Пересчёт фильтра ориентации под другую частоту опроса с сохранением угла.
 * Просто переинициализировать нельзя: после дрёмы фильтр заново сходился бы
 * к вертикали полсекунды, и всё это время нажим со струной были бы взяты с
 * потолка. */
static void orient_retune(iv_orient_t *o, float rate_hz)
{
    const float roll = o->roll, pitch = o->pitch;
    const bool  started = o->started;
    iv_orient_init(o, rate_hz, ORIENT_TAU_S);
    o->roll    = roll;
    o->pitch   = pitch;
    o->started = started;
}

/* Перезапуск оценки скорости с сохранением дослеженного нуля гироскопа.
 *
 * Оценка перезапускается при каждом выходе из дрёмы и сна — интегратор и
 * оценка гравитации после простоя не значат ничего. А вот дослеженное смещение
 * нуля значит: оно про сам датчик, а не про движение, и за время простоя никуда
 * не делось. Терять его значило бы после каждой отложенной на минуту скрипки
 * заново собирать его секунд двадцать игры — то есть ровно тогда, когда
 * инструмент снова берут в руки. */
static void speed_restart(iv_bowspeed_t *b, float rate_hz)
{
    float bias[3] = { b->bias[0], b->bias[1], b->bias[2] };
    iv_bowspeed_init(b, rate_hz, SPEED_LEAK_S);
    b->bias[0] = bias[0];
    b->bias[1] = bias[1];
    b->bias[2] = bias[2];
}

static void publish(const bow_state_t *st)
{
    portENTER_CRITICAL(&s_state_lock);
    s_state       = *st;
    s_state_valid = true;
    portEXIT_CRITICAL(&s_state_lock);
}

/* Во сне телеметрии нет: снимок обнуляется, чтобы проснувшийся гриф не поймал
 * последнюю скорость двухминутной давности. */
static void publish_still(float tilt, const float a[3])
{
    /* Угол движения у стоящего смычка не определён — движения-то нет. Нулём
     * его подменять нельзя: ноль это «ведёт горизонтально», то есть вполне
     * конкретная струна. Но и врать некому: пакет уйдёт с at_rest, а по нему
     * гриф молчит независимо от угла. */
    const bow_state_t st = {
        .speed = 0.0f, .move_angle = 0.0f, .tilt = tilt,
        .amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]),
        .at_rest = true,
        /* Оси кладём и в дрёме: смотреть в лог тем нужнее, чем меньше
         * инструмент отзывается, а «все нули» читались бы как неисправность. */
        .acc = { a[0], a[1], a[2] },
    };
    publish(&st);
}

/* ---- опрос и счёт --------------------------------------------------------- */

/* Сон: радио выключено, процессор спит между опросами датчика.
 *
 * Возвращается, когда смычок шевельнули. Light sleep может не состояться —
 * его вправе отклонить любой драйвер, держащий блокировку, — и тогда режим
 * вырождается в обычное ожидание: экономия меньше, работа та же. */
static void sleep_until_moved(void)
{
    ESP_LOGI(TAG, "смычок лежит %d с — радио выключено, засыпаю",
             SLEEP_AFTER_MS / 1000);

    /* Ждём, пока передатчик встанет. Секунды с запасом хватает: его период в
     * дрёме 50 мс. Не дождались — засыпаем всё равно, лишний неотправленный
     * пакет дешевле, чем незасыпающий смычок. */
    for (int i = 0; i < 100 && !s_tx_parked; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    iv_link_radio_pause();
    bsp_bow_imu_set_rate(BSP_BOW_IMU_IDLE);

    bool warned = false;
    while (1) {
        esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_POLL_MS * 1000);
        if (esp_light_sleep_start() != ESP_OK) {
            if (!warned) {
                ESP_LOGW(TAG, "light sleep отклонён, сплю обычным ожиданием");
                warned = true;
            }
            vTaskDelay(pdMS_TO_TICKS(SLEEP_POLL_MS));
        }

        bsp_bow_imu_t s;
        if (bsp_bow_imu_read(&s) != ESP_OK) {
            continue;
        }
        float a[3] = { s.ax, s.ay, s.az };
        float g[3] = { s.gx, s.gy, s.gz };
        iv_calib_apply(&s_calib, a, g);
        if (iv_motion_is_moving(a, g)) {
            break;
        }
    }

    ESP_LOGI(TAG, "смычок шевельнули — поднимаю радио");
    bsp_bow_imu_set_rate(BSP_BOW_IMU_FULL);
    iv_link_radio_resume();
}

static void imu_task(void *arg)
{
    (void)arg;
    TickType_t next = xTaskGetTickCount();

    iv_orient_t   orient;
    iv_bowspeed_t speed;
    iv_calib_acc_t acc;

    iv_orient_init(&orient, (float)BSP_BOW_IMU_HZ, ORIENT_TAU_S);
    iv_bowspeed_init(&speed, (float)BSP_BOW_IMU_HZ, SPEED_LEAK_S);
    iv_calib_begin(&acc);

    bool calibrated = false;
    /* Окно пика ускорения для лога: копим секунду, показываем накопленное. */
    float peak_dev = 0.0f, peak_shown = 0.0f;
    int   peak_ms = 0;
    /* Сколько миллисекунд подряд смычок не подаёт признаков жизни. Считается
     * в миллисекундах, а не в выборках: частота опроса меняется вместе с
     * режимом, и счётчик выборок означал бы в дрёме совсем другое время. */
    int  rest_ms = 0;

    while (1) {
        const bow_mode_t mode = s_mode;
        const int period_ms = (mode == BOW_MODE_ACTIVE)
                                  ? 1000 / BSP_BOW_IMU_HZ
                                  : 1000 / BSP_BOW_IDLE_HZ;

        bsp_bow_imu_t s;
        if (bsp_bow_imu_read(&s) == ESP_OK) {
            float a[3] = { s.ax, s.ay, s.az };
            float g[3] = { s.gx, s.gy, s.gz };

            /* Сначала калибровка по сырым данным, потом поправка — иначе она
             * измеряла бы саму себя. */
            if (!calibrated && mode == BOW_MODE_ACTIVE
                && iv_calib_feed(&acc, a, g, CALIB_SAMPLES)) {
                iv_calib_take(&acc, &s_calib);
                calibrated = true;
                s_calibrating = false;
                calib_save(&s_calib);
                ESP_LOGI(TAG, "калибровка набрана: смещение гироскопа "
                              "%.2f/%.2f/%.2f °/с, масштаб акселерометра %.4f",
                         s_calib.gyro_bias[0], s_calib.gyro_bias[1],
                         s_calib.gyro_bias[2], s_calib.accel_scale);
            }

            iv_calib_apply(&s_calib, a, g);
            iv_orient_update(&orient, a, g);

            if (mode == BOW_MODE_ACTIVE) {
                bool  at_rest = false;
                float angle = 0.0f;
                const float v = iv_bowspeed_update(&speed, a, g, &angle, &at_rest);

                const float amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
                const float gmag = sqrtf(speed.grav[0] * speed.grav[0]
                                       + speed.grav[1] * speed.grav[1]
                                       + speed.grav[2] * speed.grav[2]);
                /* Пик за секунду копится здесь, а не в задаче передачи: она
                 * видит одну выборку из двадцати пяти, и дрожь руки прошла бы
                 * мимо неё целиком. Окно своё, потому что и обнулять его
                 * должен тот, кто копит. */
                const float dev = fabsf(amag - gmag);
                if (dev > peak_dev) {
                    peak_dev = dev;
                }
                if ((peak_ms += period_ms) >= 1000) {
                    peak_ms = 0;
                    peak_shown = peak_dev;
                    peak_dev = 0.0f;
                }

                const bow_state_t st = {
                    .speed      = v,
                    .move_angle = angle,
                    .tilt       = orient.pitch,
                    .amag       = amag,
                    .at_rest    = at_rest,
                    .acc        = { a[0], a[1], a[2] },
                    .grav       = { speed.grav[0], speed.grav[1], speed.grav[2] },
                    .amag_peak  = peak_shown,
                    .env_h      = speed.acc_env,
                    .env_v      = fabsf(speed.acc_v),
                    .env_gyro   = speed.gyro_env,
                    .bias       = { speed.bias[0], speed.bias[1], speed.bias[2] },
                    .still      = speed.still,
                    .still_need = speed.still_need,
                };
                publish(&st);

                rest_ms = at_rest ? rest_ms + period_ms : 0;
                if (rest_ms >= DROWSY_AFTER_MS) {
                    /* В дрёме оценки скорости нет: интегрировать по 25 Гц
                     * бессмысленно. Ориентацию продолжаем считать — она нужна
                     * сразу же, как смычок возьмут в руку. */
                    ESP_LOGI(TAG, "смычок лежит %d с — перехожу на пониженную частоту",
                             DROWSY_AFTER_MS / 1000);
                    bsp_bow_imu_set_rate(BSP_BOW_IMU_IDLE);
                    orient_retune(&orient, (float)BSP_BOW_IDLE_HZ);
                    s_mode = BOW_MODE_DROWSY;
                    next = xTaskGetTickCount();
                }
            } else {
                /* Дрёма. Единственный вопрос — шевелят или нет. */
                publish_still(orient.pitch, a);

                if (iv_motion_is_moving(a, g)) {
                    bsp_bow_imu_set_rate(BSP_BOW_IMU_FULL);
                    orient_retune(&orient, (float)BSP_BOW_IMU_HZ);
                    speed_restart(&speed, (float)BSP_BOW_IMU_HZ);
                    rest_ms = 0;
                    s_mode = BOW_MODE_ACTIVE;
                    next = xTaskGetTickCount();
                    xTaskNotifyGive(s_tx_task);
                    ESP_LOGI(TAG, "смычок в руке — полная частота");
                } else {
                    rest_ms += period_ms;
                    if (rest_ms >= SLEEP_AFTER_MS) {
                        s_mode = BOW_MODE_SLEEP;
                        sleep_until_moved();
                        orient_retune(&orient, (float)BSP_BOW_IMU_HZ);
                        speed_restart(&speed, (float)BSP_BOW_IMU_HZ);
                        rest_ms = 0;
                        s_mode = BOW_MODE_ACTIVE;
                        next = xTaskGetTickCount();
                        xTaskNotifyGive(s_tx_task);
                    }
                }
            }
        }
        vTaskDelayUntil(&next, pdMS_TO_TICKS(period_ms));
    }
}

/* ---- передача -------------------------------------------------------------- */

static void fill_motion(iv_bow_packet_t *p, const bow_state_t *st)
{
    p->speed      = iv_pack_vel(st->speed);
    p->move_angle = iv_pack_angle(st->move_angle);
    p->tilt       = iv_pack_angle(st->tilt);
    p->accel_mag  = iv_pack_accel(st->amag);
    if (st->at_rest) {
        p->flags |= IV_FLAG_AT_REST;
    }
}

static void battery_poll(void)
{
    uint16_t mv = 0;
    bool     usb = false;
    if (!bsp_bow_power_read(&mv, &usb)) {
        return;
    }
    s_batt_mv  = mv;
    s_usb      = usb;
    s_batt_pct = iv_batt_percent(mv);
}

static void tx_task(void *arg)
{
    (void)arg;
    TickType_t next = xTaskGetTickCount();

    uint16_t seq = 0;
    int      tick = 0;

    /* Время оборота: гриф отвечает эхом на каждый IV_PING_EVERY-й пакет и
     * сообщает, сколько эхо пролежало у него. За вычетом этого половина
     * оборота — время в эфире в одну сторону. */
    uint32_t rtt_min = UINT32_MAX, rtt_max = 0, rtt_sum = 0, rtt_n = 0;
    uint32_t proc_sum = 0;
    uint32_t sent_prev = 0;

    int64_t next_second = esp_timer_get_time() + 1000000;
    int64_t next_batt   = 0;
    bool    warned_low  = false;

    while (1) {
        const bow_mode_t mode = s_mode;

        /* Во сне передавать нечего и нечем: радио выключено. Ждём, пока
         * задача опроса разбудит — иначе после сна vTaskDelayUntil наверстывал
         * бы пропущенные периоды сотнями пустых проходов. */
        if (mode == BOW_MODE_SLEEP) {
            s_tx_parked = true;
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            s_tx_parked = false;
            next = xTaskGetTickCount();
            tick = 0;
            continue;
        }

        const bool paired  = iv_link_bow_paired();
        const bool drowsy  = (mode == BOW_MODE_DROWSY);
        const int  rate_hz = drowsy ? DROWSY_TX_HZ : IV_BOW_RATE_HZ;

        /* Без пары шлём редко: спаривание не горячий путь. */
        const int divider = paired ? 1 : (rate_hz / IV_PAIR_RATE_HZ);
        if (divider <= 0 || tick % divider == 0) {
            bow_state_t st;
            bool have;
            portENTER_CRITICAL(&s_state_lock);
            st   = s_state;
            have = s_state_valid;
            portEXIT_CRITICAL(&s_state_lock);

            iv_bow_packet_t p;
            iv_bow_packet_init(&p, seq++, (uint32_t)esp_timer_get_time());
            if (have) {
                fill_motion(&p, &st);
            }
            if (!have || s_calibrating) {
                /* Данным ещё нельзя верить: либо IMU не отдал ни одной
                 * выборки, либо не набралась калибровка. */
                p.flags |= IV_FLAG_CALIBRATING;
            }
            if (!paired) {
                p.flags |= IV_FLAG_PAIRING;
            }
            if (drowsy) {
                p.flags |= IV_FLAG_IDLE;
            }
            if (s_usb) {
                p.flags |= IV_FLAG_CHARGING;
            }
            if (p.seq % IV_PING_EVERY == 0) {
                p.flags |= IV_FLAG_PING;
            }
            p.batt = s_batt_pct;

            iv_link_bow_send(&p);
        }

        uint32_t rtt, proc;
        if (iv_link_bow_take_rtt(&rtt, &proc)) {
            if (rtt < rtt_min) { rtt_min = rtt; }
            if (rtt > rtt_max) { rtt_max = rtt; }
            rtt_sum  += rtt;
            proc_sum += proc;
            rtt_n++;
        }

        const int64_t now = esp_timer_get_time();
        if (now >= next_batt) {
            next_batt = now + (int64_t)BATT_PERIOD_MS * 1000;
            battery_poll();
        }

        if (now >= next_second) {
            next_second = now + 1000000;

            bow_state_t st;
            portENTER_CRITICAL(&s_state_lock);
            st = s_state;
            portEXIT_CRITICAL(&s_state_lock);
            ESP_LOGI(TAG, "движение: скорость %.2f, линия %+.1f° к горизонту, "
                          "(наклон корпуса %+.1f°, в звук не идёт), %s%s%s",
                     st.speed, st.move_angle, st.tilt,
                     /* Про флаг покоя, а не про звук: гриф молчит и когда
                      * покой не объявлен, но скорость ниже мёртвой зоны. */
                     st.at_rest ? "покой" : "движение",
                     drowsy ? ", дремлет" : "",
                     s_calibrating ? ", калибровки нет — молчу" : "");

            /* Оси уже приведены к смычку в bsp_bow_imu_read: X продольная и
             * смотрит на конец смычка. Сверяться с этим приходится каждый
             * раз, когда угол выходит не тот — там же и правится монтаж.
             * Рядом — оценка гравитации: если она разошлась с показаниями
             * лежащего смычка, врать будет всё остальное. */
            ESP_LOGI(TAG, "ускорение: (%+.3f %+.3f %+.3f) g, модуль %.3f; "
                          "гравитация (%+.3f %+.3f %+.3f), пик за секунду %.3f g",
                     st.acc[0], st.acc[1], st.acc[2], st.amag,
                     st.grav[0], st.grav[1], st.grav[2], st.amag_peak);

            /* Три признака покоя со своими порогами и счётчик. Это ответ на
             * вопрос «почему звучит (или молчит), когда не должен»: покой
             * объявляется, только когда все три ниже порога подряд столько
             * выборок, сколько показывает счётчик. */
            ESP_LOGI(TAG, "покой: гор %.4f/%.2f g, вер %.4f/%.2f g, "
                          "гиро %.2f/%.0f °/с → счётчик %d/%d",
                     st.env_h, IV_BOW_REST_ACCEL_G,
                     st.env_v, IV_BOW_REST_AMAG_G,
                     st.env_gyro, IV_BOW_REST_GYRO_DPS,
                     st.still, st.still_need);

            /* Дослеженный на остановках ноль гироскопа — поверх калибровки при
             * включении. Смотреть сюда, когда струна уползает: градус в секунду
             * здесь стоит двадцати градусов угла, то есть целой струны. Растёт
             * первые полминуты игры и после прогрева платы — так и должно быть;
             * упёрлось в потолок IV_BOW_BIAS_MAX_DPS — значит петля ловит не
             * смещение, и смотреть надо на монтаж датчика. */
            ESP_LOGI(TAG, "ноль гироскопа: калибровка (%+.2f %+.2f %+.2f), "
                          "дослежено (%+.2f %+.2f %+.2f) °/с",
                     s_calib.gyro_bias[0], s_calib.gyro_bias[1], s_calib.gyro_bias[2],
                     st.bias[0], st.bias[1], st.bias[2]);

            if (s_batt_pct != IV_BATT_UNKNOWN) {
                ESP_LOGI(TAG, "батарея: %u %% (%u мВ)%s",
                         s_batt_pct, s_batt_mv, s_usb ? ", питание от USB" : "");
            }
            /* Про разряд говорим один раз, а не каждую секунду: повторяющееся
             * предупреждение перестают читать. Порог отпускается обратно,
             * когда смычок поставили заряжаться. */
            if (iv_batt_is_low(s_batt_pct) && !s_usb) {
                if (!warned_low) {
                    warned_low = true;
                    ESP_LOGW(TAG, "заряд смычка %u %% — пора на зарядку", s_batt_pct);
                }
            } else {
                warned_low = false;
            }

            iv_link_tx_stats_t tx;
            iv_link_bow_tx_stats(&tx);

            ESP_LOGI(TAG, "%s: отправлено %" PRIu32 " (за секунду %" PRIu32
                          "), подтверждено %" PRIu32 ", без ответа %" PRIu32
                          ", ошибок %" PRIu32,
                     paired ? "пара" : "спаривание",
                     tx.sent, tx.sent - sent_prev, tx.acked, tx.failed, tx.errors);
            sent_prev = tx.sent;

            if (rtt_n) {
                /* Оборот минус обработка на грифе — чистый эфир туда и обратно;
                 * половина этого и есть задержка доставки. */
                const uint32_t rtt_avg  = rtt_sum / rtt_n;
                const uint32_t proc_avg = proc_sum / rtt_n;
                const double   air_ms   = (rtt_avg > proc_avg)
                                              ? (rtt_avg - proc_avg) / 2000.0
                                              : 0.0;

                ESP_LOGI(TAG, "оборот по %" PRIu32 " замерам: мин %" PRIu32 ", средн %"
                              PRIu32 ", макс %" PRIu32 " мкс; обработка на грифе %"
                              PRIu32 " мкс -> эфир в одну сторону %.1f мс",
                         rtt_n, rtt_min, rtt_avg, rtt_max, proc_avg, air_ms);

                rtt_min  = UINT32_MAX;
                rtt_max  = 0;
                rtt_sum  = 0;
                proc_sum = 0;
                rtt_n    = 0;
            } else if (paired && !drowsy) {
                /* Пара есть, а эха нет — это уже не про задержку, а про то,
                 * что обратный канал не работает. Молчать об этом нельзя.
                 * В дрёме эха и не ждём: ping уходит раз в десять секунд. */
                ESP_LOGW(TAG, "эхо от грифа не вернулось ни разу за секунду");
            }
        }

        tick++;
        vTaskDelayUntil(&next, pdMS_TO_TICKS(1000 / rate_hz));
    }
}

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    ESP_LOGI(TAG, "воображаемая скрипка — смычок, фаза 6");
    ESP_LOGI(TAG, "чип: %s, ядер: %d, ревизия %d",
             CONFIG_IDF_TARGET, chip.cores, chip.revision);
    ESP_LOGI(TAG, "протокол версии %u, пакет %u байт, темп передачи %d Гц",
             (unsigned)IV_PROTO_VERSION, (unsigned)sizeof(iv_bow_packet_t),
             IV_BOW_RATE_HZ);

    ESP_ERROR_CHECK(bsp_bow_board_init());
    ESP_ERROR_CHECK(bsp_bow_imu_init());
    ESP_ERROR_CHECK(bsp_bow_power_init());
    ESP_ERROR_CHECK(iv_link_init(IV_LINK_ROLE_BOW)); /* он же поднимает NVS */

    /* Прошлая калибровка работает, пока набирается новая: инструмент играет
     * сразу после включения, а не через секунду. */
    iv_calib_reset(&s_calib);
    if (calib_load(&s_calib)) {
        /* Есть чем играть — значит играем. Свежая наберётся фоном, как только
         * смычок окажется неподвижен, и заменит эту. */
        s_calibrating = false;
        ESP_LOGI(TAG, "калибровка из NVS: смещение гироскопа %.2f/%.2f/%.2f °/с, "
                      "масштаб %.4f; играем по ней, свежая наберётся фоном",
                 s_calib.gyro_bias[0], s_calib.gyro_bias[1], s_calib.gyro_bias[2],
                 s_calib.accel_scale);
    } else {
        ESP_LOGI(TAG, "калибровки нет — до неё смычок молчит: "
                      "положи его неподвижно на секунду");
    }

    /* Опрос IMU приоритетнее передачи: пропущенная выборка теряется навсегда,
     * а пакет, ушедший на миллисекунду позже, — нет. */
    xTaskCreatePinnedToCore(tx_task,  "tx",  4096, NULL, 10, &s_tx_task, 0);
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, 11, NULL, 0);

    ESP_LOGI(TAG, "IMU, питание и радио подняты");
}
