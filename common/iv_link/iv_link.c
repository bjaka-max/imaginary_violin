#include "iv_link.h"

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "iv.link";

#define NVS_NAMESPACE "iv_link"
#define NVS_KEY_PEER  "peer"

/* Период холостого прохода задачи приёма. Он же — точность watchdog'а:
 * связь объявляется потерянной через IV_LINK_TIMEOUT_MS + этот период,
 * то есть до 120 мс при требуемых 150. */
#define LINK_TICK_MS  20

/* Сколько подряд неподтверждённых unicast'ов означает «грифа больше нет».
 * Секунда: столько молчания уже не объяснить помехой. После этого смычок
 * возвращается к broadcast и ищет пару заново — иначе он вечно сыпал бы
 * пакеты в пустоту, например после перепаривания на грифе. */
#define TX_FAIL_LIMIT IV_BOW_RATE_HZ

#define RX_QUEUE_LEN  16

static const uint8_t BROADCAST_MAC[IV_LINK_MAC_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct {
    uint8_t src[IV_LINK_MAC_LEN];
    int8_t  rssi;
    int64_t rx_us;
    uint8_t len;
    uint8_t data[sizeof(iv_bow_packet_t)];
} rx_event_t;

static iv_link_role_t s_role;
static QueueHandle_t  s_rx_queue;

/* Будильник для слоя управления: двоичный семафор, поэтому несколько пакетов
 * подряд схлопываются в одно пробуждение — управлению нужен последний пакет,
 * а не их история. */
static SemaphoreHandle_t s_rx_signal;

/* Пара. Пишется только задачей приёма, читается кем угодно. */
static uint8_t        s_peer[IV_LINK_MAC_LEN];
static volatile bool  s_have_peer;

/* Статистика приёма и последний пакет: пишет задача приёма, читает интерфейс.
 * Структуры больше машинного слова, поэтому копия берётся под спинлоком —
 * он держится единицы микросекунд и звуку не мешает. */
static portMUX_TYPE     s_lock = portMUX_INITIALIZER_UNLOCKED;
static iv_link_stats_t  s_stats;
static iv_bow_packet_t  s_last_pkt;
static int8_t           s_last_rssi;
static int64_t          s_last_rx_us;
static bool             s_have_pkt;

/* Признак живой связи отдельным словом, вне спинлока: его читает аудиопоток
 * каждый блок, а он не должен ждать ничего и никогда. Одно слово читается
 * атомарно, и опоздание на один блок (2.7 мс) здесь ничего не решает. */
static volatile bool    s_online;

/* Счётчики, которые трогает колбэк радио. Каждый — одно слово, гонки за
 * единичный инкремент здесь безобидны: это диагностика, а не логика. */
static volatile uint32_t s_rejected;  /* чужой трафик и рассинхрон версий */
static volatile uint32_t s_foreign;   /* валидный пакет, но от чужого смычка */
static volatile uint32_t s_overflow;  /* очередь приёма не успела */

static volatile uint32_t s_tx_sent, s_tx_acked, s_tx_failed, s_tx_errors;
static volatile uint32_t s_tx_fail_run;

/* Проходы задачи приёма: доказательство, что она жива. */
static volatile uint32_t s_task_ticks;

static volatile uint32_t s_rtt_us;
static volatile uint32_t s_rtt_proc;
static volatile bool     s_rtt_fresh;

/* Просьба забыть пару. Флаг, а не прямой вызов: пиры меняет только задача
 * приёма, иначе кнопка и радио могли бы переписывать MAC одновременно —
 * порванная пополам запись оставила бы инструмент без связи до перезагрузки. */
static volatile bool     s_forget_req;

/* ---- NVS -------------------------------------------------------------- */

static void peer_save(const uint8_t mac[IV_LINK_MAC_LEN])
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS не открылась, пара не переживёт перезагрузку");
        return;
    }
    if (nvs_set_blob(h, NVS_KEY_PEER, mac, IV_LINK_MAC_LEN) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

static void peer_erase(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, NVS_KEY_PEER);
    nvs_commit(h);
    nvs_close(h);
}

static bool peer_load(uint8_t mac[IV_LINK_MAC_LEN])
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = IV_LINK_MAC_LEN;
    const esp_err_t err = nvs_get_blob(h, NVS_KEY_PEER, mac, &len);
    nvs_close(h);
    return err == ESP_OK && len == IV_LINK_MAC_LEN;
}

/* ---- пиры ------------------------------------------------------------- */

static esp_err_t peer_add(const uint8_t mac[IV_LINK_MAC_LEN])
{
    esp_now_peer_info_t peer = {
        .channel = IV_WIFI_CHANNEL, /* явно, а не 0: обе платы стоят на нём жёстко */
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,           /* шифровать нечего: телеметрия смычка не секрет */
    };
    memcpy(peer.peer_addr, mac, IV_LINK_MAC_LEN);
    if (esp_now_is_peer_exist(mac)) {
        return esp_now_mod_peer(&peer);
    }
    return esp_now_add_peer(&peer);
}

static void peer_set(const uint8_t mac[IV_LINK_MAC_LEN], bool persist)
{
    if (peer_add(mac) != ESP_OK) {
        ESP_LOGE(TAG, "не удалось добавить пир");
        return;
    }
    memcpy(s_peer, mac, IV_LINK_MAC_LEN);
    s_have_peer = true;
    s_tx_fail_run = 0;
    if (persist) {
        peer_save(mac);
    }
    ESP_LOGI(TAG, "пара: " MACSTR, MAC2STR(mac));
}

static void peer_drop(void)
{
    if (s_have_peer) {
        esp_now_del_peer(s_peer);
    }
    s_have_peer = false;
    s_tx_fail_run = 0;
    memset(s_peer, 0, sizeof(s_peer));
    peer_erase();
}

/* ---- колбэки радио ---------------------------------------------------- */

/* Оба колбэка выполняются в задаче Wi-Fi. Длинных операций здесь быть не должно:
 * всё, что дольше проверки заголовка, уезжает в задачу приёма. */
static void send_cb(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_tx_acked++;
        s_tx_fail_run = 0;
    } else {
        s_tx_failed++;
        s_tx_fail_run++;
    }
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (info == NULL || data == NULL || len <= 0) {
        return;
    }

    const bool ok = (s_role == IV_LINK_ROLE_NECK)
                        ? iv_bow_packet_valid(data, (size_t)len)
                        : iv_neck_packet_valid(data, (size_t)len);
    if (!ok) {
        s_rejected++;
        return;
    }

    rx_event_t e;
    memcpy(e.src, info->src_addr, IV_LINK_MAC_LEN);
    e.rssi  = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
    e.rx_us = esp_timer_get_time();
    e.len   = (uint8_t)len;
    memcpy(e.data, data, (size_t)len);

    if (xQueueSend(s_rx_queue, &e, 0) != pdTRUE) {
        s_overflow++;
    }
}

/* ---- обработка приёма ------------------------------------------------- */

/* rx_us — когда пакет, на который отвечаем, пришёл в колбэк радио. Разница до
 * этого момента и есть время, которое эхо пролежало на грифе; смычок вычтет
 * его из оборота и получит время в эфире. */
static void neck_send(uint8_t type, uint32_t echo_t_us, int64_t rx_us)
{
    if (!s_have_peer) {
        return;
    }
    iv_neck_packet_t ans;
    iv_neck_packet_init(&ans, type, echo_t_us,
                        (uint32_t)(esp_timer_get_time() - rx_us));
    esp_now_send(s_peer, (const uint8_t *)&ans, sizeof(ans));
}

static void neck_on_packet(const rx_event_t *e)
{
    iv_bow_packet_t p;
    memcpy(&p, e->data, sizeof(p));

    if (!s_have_peer) {
        /* Пары нет — первый услышанный смычок и становится парой.
         * Отдельного «режима спаривания» нет намеренно: инструмент из двух
         * устройств, и выбирать не из чего. */
        peer_set(e->src, true);
        neck_send(IV_NECK_MSG_PAIR_ACK, p.t_us, e->rx_us);
    } else if (memcmp(e->src, s_peer, IV_LINK_MAC_LEN) != 0) {
        s_foreign++;
        return;
    } else if (p.flags & IV_FLAG_PAIRING) {
        /* Смычок нас забыл (перепрошили или сбросили) и вещает в broadcast,
         * хотя мы его помним. Подтверждаем пару заново. */
        neck_send(IV_NECK_MSG_PAIR_ACK, p.t_us, e->rx_us);
    }

    portENTER_CRITICAL(&s_lock);
    const bool back = iv_link_stats_on_packet(&s_stats, p.seq, e->rx_us);
    s_last_pkt   = p;
    s_last_rssi  = e->rssi;
    s_last_rx_us = e->rx_us;
    s_have_pkt   = true;
    portEXIT_CRITICAL(&s_lock);
    s_online = true;

    if (s_rx_signal) {
        xSemaphoreGive(s_rx_signal);
    }

    if (back) {
        ESP_LOGI(TAG, "связь восстановилась");
    }
    if (p.flags & IV_FLAG_PING) {
        neck_send(IV_NECK_MSG_ECHO, p.t_us, e->rx_us);
    }
}

static void bow_on_packet(const rx_event_t *e)
{
    iv_neck_packet_t n;
    memcpy(&n, e->data, sizeof(n));

    switch (n.type) {
    case IV_NECK_MSG_PAIR_ACK:
        if (!s_have_peer || memcmp(e->src, s_peer, IV_LINK_MAC_LEN) != 0) {
            peer_set(e->src, true);
        }
        break;

    case IV_NECK_MSG_ECHO:
        /* Время оборота считаем в 32 битах: t_us переполняется примерно раз в
         * 71 минуту, и беззнаковая разность обрезанных до 32 бит часов даёт
         * верный результат по обе стороны переполнения. */
        s_rtt_us    = (uint32_t)e->rx_us - n.echo_t_us;
        s_rtt_proc  = n.proc_us;
        s_rtt_fresh = true;
        break;

    default:
        break;
    }
}

static void link_task(void *arg)
{
    (void)arg;
    rx_event_t e;

    while (1) {
        s_task_ticks++;

        if (xQueueReceive(s_rx_queue, &e, pdMS_TO_TICKS(LINK_TICK_MS)) == pdTRUE) {
            if (s_role == IV_LINK_ROLE_NECK) {
                neck_on_packet(&e);
            } else {
                bow_on_packet(&e);
            }
        }

        if (s_role == IV_LINK_ROLE_NECK) {
            if (s_forget_req) {
                s_forget_req = false;
                peer_drop();
                portENTER_CRITICAL(&s_lock);
                iv_link_stats_reset(&s_stats);
                s_have_pkt = false;
                portEXIT_CRITICAL(&s_lock);
                s_online = false;
                ESP_LOGI(TAG, "пара забыта: следующий услышанный смычок станет новым");
            }

            portENTER_CRITICAL(&s_lock);
            const bool lost = iv_link_stats_tick(&s_stats, esp_timer_get_time());
            portEXIT_CRITICAL(&s_lock);
            if (lost) {
                s_online = false;
                ESP_LOGW(TAG, "связь потеряна: смычок молчит дольше %d мс",
                         IV_LINK_TIMEOUT_MS);
            }
        } else if (s_have_peer && s_tx_fail_run >= TX_FAIL_LIMIT) {
            /* Гриф перестал подтверждать приём — ищем пару заново. */
            ESP_LOGW(TAG, "гриф не подтверждает приём %" PRIu32 " пакетов подряд — "
                          "возвращаюсь к спариванию", (uint32_t)s_tx_fail_run);
            peer_drop();
        }
    }
}

/* ---- инициализация ---------------------------------------------------- */

static esp_err_t wifi_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "очистка NVS");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "цикл событий");
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init");
    /* Настройки Wi-Fi в NVS не нужны: к точкам доступа мы не подключаемся,
     * а лишняя запись во flash при старте — лишняя задержка. */
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "режим STA");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "старт Wi-Fi");
    /* Без этого модем засыпает между beacon'ами и приём опаздывает на десятки
     * миллисекунд — весь бюджет задержки. Цена — питание, ей займётся фаза 6. */
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "энергосбережение");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(IV_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE),
                        TAG, "канал");
    return ESP_OK;
}

esp_err_t iv_link_init(iv_link_role_t role)
{
    s_role = role;
    iv_link_stats_reset(&s_stats);

    s_rx_queue = xQueueCreate(RX_QUEUE_LEN, sizeof(rx_event_t));
    ESP_RETURN_ON_FALSE(s_rx_queue, ESP_ERR_NO_MEM, TAG, "очередь приёма");

    if (role == IV_LINK_ROLE_NECK) {
        s_rx_signal = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_rx_signal, ESP_ERR_NO_MEM, TAG, "сигнал приёма");
    }

    ESP_RETURN_ON_ERROR(wifi_start(), TAG, "Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "ESP-NOW");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(recv_cb), TAG, "колбэк приёма");
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(send_cb), TAG, "колбэк отправки");

    if (role == IV_LINK_ROLE_BOW) {
        /* Broadcast — тоже пир, без него esp_now_send откажет. */
        ESP_RETURN_ON_ERROR(peer_add(BROADCAST_MAC), TAG, "broadcast");
    }

    uint8_t saved[IV_LINK_MAC_LEN];
    if (peer_load(saved)) {
        peer_set(saved, false);
        ESP_LOGI(TAG, "пара восстановлена из NVS");
    } else {
        ESP_LOGI(TAG, "пары нет, ждём спаривания");
    }

    uint8_t own[IV_LINK_MAC_LEN];
    iv_link_own_mac(own);
    ESP_LOGI(TAG, "%s, канал %d, свой MAC " MACSTR,
             role == IV_LINK_ROLE_BOW ? "смычок" : "гриф", IV_WIFI_CHANNEL, MAC2STR(own));

    BaseType_t ok = xTaskCreatePinnedToCore(link_task, "link", 4096, NULL, 10, NULL, 0);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "задача приёма");
    return ESP_OK;
}

void iv_link_own_mac(uint8_t mac[IV_LINK_MAC_LEN])
{
    esp_wifi_get_mac(WIFI_IF_STA, mac);
}

esp_err_t iv_link_radio_pause(void)
{
    ESP_LOGI(TAG, "радио остановлено");
    return esp_wifi_stop();
}

esp_err_t iv_link_radio_resume(void)
{
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "старт Wi-Fi");
    /* Обе настройки живут не дольше запущенного Wi-Fi, и обе обязательны:
     * без WIFI_PS_NONE приём опаздывает на десятки миллисекунд, без явного
     * канала обе платы могут оказаться на разных. */
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), TAG, "энергосбережение");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(IV_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE),
                        TAG, "канал");

    /* Пиры остановку переживают, но проверять это на живом инструменте
     * дороже, чем просто перезаписать их: peer_add умеет и добавлять,
     * и обновлять. */
    if (s_role == IV_LINK_ROLE_BOW) {
        peer_add(BROADCAST_MAC);
    }
    if (s_have_peer) {
        peer_add(s_peer);
    }

    ESP_LOGI(TAG, "радио поднято");
    return ESP_OK;
}

/* ---- смычок ----------------------------------------------------------- */

esp_err_t iv_link_bow_send(const iv_bow_packet_t *p)
{
    const uint8_t *dst = s_have_peer ? s_peer : BROADCAST_MAC;
    const esp_err_t err = esp_now_send(dst, (const uint8_t *)p, sizeof(*p));
    if (err == ESP_OK) {
        s_tx_sent++;
    } else {
        s_tx_errors++;
    }
    return err;
}

bool iv_link_bow_paired(void)
{
    return s_have_peer;
}

void iv_link_bow_tx_stats(iv_link_tx_stats_t *out)
{
    out->sent   = s_tx_sent;
    out->acked  = s_tx_acked;
    out->failed = s_tx_failed;
    out->errors = s_tx_errors;
}

bool iv_link_bow_take_rtt(uint32_t *rtt_us, uint32_t *proc_us)
{
    if (!s_rtt_fresh) {
        return false;
    }
    s_rtt_fresh = false;
    *rtt_us  = s_rtt_us;
    *proc_us = s_rtt_proc;
    return true;
}

/* ---- гриф ------------------------------------------------------------- */

uint32_t iv_link_task_ticks(void)
{
    return s_task_ticks;
}

void iv_link_rx_errors(iv_link_rx_errors_t *out)
{
    out->rejected = s_rejected;
    out->foreign  = s_foreign;
    out->overflow = s_overflow;
}

bool iv_link_neck_wait(uint32_t timeout_ms)
{
    if (!s_rx_signal) {
        return false;
    }
    return xSemaphoreTake(s_rx_signal, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void iv_link_neck_stats(iv_link_stats_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_stats;
    portEXIT_CRITICAL(&s_lock);
}

bool iv_link_neck_last(iv_bow_packet_t *out, int8_t *rssi, int64_t *rx_us)
{
    portENTER_CRITICAL(&s_lock);
    const bool have = s_have_pkt;
    if (have) {
        *out = s_last_pkt;
        if (rssi) {
            *rssi = s_last_rssi;
        }
        if (rx_us) {
            *rx_us = s_last_rx_us;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    return have;
}

bool iv_link_neck_online(void)
{
    return s_online;
}

bool iv_link_neck_peer(uint8_t mac[IV_LINK_MAC_LEN])
{
    if (!s_have_peer) {
        return false;
    }
    memcpy(mac, s_peer, IV_LINK_MAC_LEN);
    return true;
}

esp_err_t iv_link_neck_forget(void)
{
    s_forget_req = true;
    return ESP_OK;
}
