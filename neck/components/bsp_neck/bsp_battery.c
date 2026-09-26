#include "bsp_neck.h"
#include "bsp_internal.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "bsp.batt";

/* Затухание входа. Двенадцать децибел — единственный вариант, при котором
 * полный заряд (1.4 В на делителе) попадает в измеряемый диапазон: у меньших
 * затуханий потолок ниже, и батарея упиралась бы в него, притворяясь
 * разряженной. */
#define BATT_ATTEN ADC_ATTEN_DB_12

/* Сколько выборок усредняется. У АЦП ESP32 заметный собственный шум — единицы
 * милливольт, — а нам считать по напряжению проценты заряда, где десять
 * милливольт это процент. Восемь выборок стоят десятки микросекунд и меряются
 * раз в секунду. */
#define BATT_SAMPLES 8

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;
static adc_channel_t             s_channel;

esp_err_t bsp_battery_init(void)
{
    adc_unit_t unit;
    /* Канал спрашиваем у драйвера по номеру вывода, а не берём из таблицы:
     * соответствие «вывод — канал» у чипов разное, и ошибка здесь выглядела бы
     * как исправно работающий, но всегда разряженный аккумулятор. */
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(BSP_BATT_ADC_GPIO, &unit, &s_channel),
                        TAG, "вывод %d не входит АЦП", BSP_BATT_ADC_GPIO);

    const adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = unit };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "АЦП");

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BATT_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, s_channel, &chan_cfg),
                        TAG, "канал АЦП");

    /* Калибровка из eFuse. Без неё сырые отсчёты гуляют от чипа к чипу на
     * проценты, и заряд считался бы по чужой характеристике. Отсутствие
     * калибровки не фатально — но тогда о ней надо сказать вслух. */
    const adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = unit,
        .chan     = s_channel,
        .atten    = BATT_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) != ESP_OK) {
        s_cali = NULL;
        ESP_LOGW(TAG, "калибровки АЦП нет: напряжение батареи считается приблизительно");
    }

    ESP_LOGI(TAG, "батарея: GPIO%d, делитель 1:%d", BSP_BATT_ADC_GPIO, BSP_BATT_DIVIDER);
    return ESP_OK;
}

bool bsp_battery_read_mv(uint16_t *millivolts)
{
    if (s_adc == NULL || millivolts == NULL) {
        return false;
    }

    int sum = 0;
    for (int i = 0; i < BATT_SAMPLES; ++i) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, s_channel, &raw) != ESP_OK) {
            return false;
        }
        sum += raw;
    }
    const int raw_avg = sum / BATT_SAMPLES;

    int mv = 0;
    if (s_cali) {
        if (adc_cali_raw_to_voltage(s_cali, raw_avg, &mv) != ESP_OK) {
            return false;
        }
    } else {
        /* Запасной путь без калибровки: номинальные 3.1 В на полной шкале
         * 12 бит. Годится, чтобы отличить полный аккумулятор от пустого,
         * и не годится ни для чего точнее. */
        mv = raw_avg * 3100 / 4095;
    }

    *millivolts = (uint16_t)(mv * BSP_BATT_DIVIDER);
    return true;
}
