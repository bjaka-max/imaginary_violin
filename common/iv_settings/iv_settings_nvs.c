/* Хранение настроек в NVS. Единственный файл компонента, зависящий от
 * ESP-IDF, — поэтому в хостовую сборку он не входит, а всё, что можно
 * проверить на компьютере, лежит в iv_settings.c. */
#include "iv_settings.h"

#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "iv.settings";

#define NVS_NAMESPACE "iv_set"
#define NVS_KEY       "v1"

bool iv_settings_load(iv_settings_t *s)
{
    iv_settings_defaults(s);

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "настроек нет, беру умолчания");
        return false;
    }

    uint8_t buf[32];
    size_t  len = sizeof(buf);
    const esp_err_t err = nvs_get_blob(h, NVS_KEY, buf, &len);
    nvs_close(h);

    if (err != ESP_OK || !iv_settings_decode(s, buf, len)) {
        /* decode не тронул s, а там уже лежат умолчания. */
        ESP_LOGW(TAG, "запись настроек не читается, беру умолчания");
        iv_settings_defaults(s);
        return false;
    }

    ESP_LOGI(TAG, "настройки: строй A4 %.1f Гц, громкость %u, яркость %u, "
                  "полутоны %s, центр смычка %+.1f°",
             (double)s->a4_hz, s->volume, s->brightness,
             s->snap ? "вкл" : "выкл", (double)s->angle_center);
    return true;
}

bool iv_settings_save(const iv_settings_t *s)
{
    uint8_t buf[32];
    const size_t len = iv_settings_encode(s, buf, sizeof(buf));
    if (len == 0) {
        return false;
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS не открылся, настройки не сохранены");
        return false;
    }

    /* Сначала читаем то, что уже лежит. Запись во flash стоит миллисекунд и
     * ресурса ячеек, а сохранять настройки приходится в том числе на
     * выключении, когда времени нет: если ничего не менялось, писать нечего. */
    uint8_t  old[32];
    size_t   old_len = sizeof(old);
    if (nvs_get_blob(h, NVS_KEY, old, &old_len) == ESP_OK
        && old_len == len && memcmp(old, buf, len) == 0) {
        nvs_close(h);
        return true;
    }

    esp_err_t err = nvs_set_blob(h, NVS_KEY, buf, len);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "настройки не сохранились: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "настройки сохранены");
    return true;
}
