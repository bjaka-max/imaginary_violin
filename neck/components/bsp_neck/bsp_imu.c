#include "bsp_neck.h"
#include "bsp_internal.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp.imu";

/* QMI8658. Адреса регистров сверены с библиотекой, которую использует
 * демо производителя, а не взяты по памяти. */
#define REG_WHOAMI      0x00
#define REG_REVISION    0x01
#define REG_CTRL1       0x02
#define REG_CTRL2       0x03  /* акселерометр: диапазон и частота */
#define REG_CTRL3       0x04  /* гироскоп: диапазон и частота */
#define REG_CTRL5       0x06  /* фильтры нижних частот */
#define REG_CTRL7       0x08  /* включение датчиков */
#define REG_STATUS0     0x2E
#define REG_TEMP_L      0x33
#define REG_AX_L        0x35

#define WHOAMI_VALUE    0x05

/* Диапазоны выбраны под инструмент: гриф не испытывает больших ускорений,
 * но ловит быстрые мелкие движения (вибрато). */
#define ACC_RANGE_G     4.0f    /* CTRL2: +-4g */
#define GYR_RANGE_DPS   512.0f  /* CTRL3: +-512 dps */

static i2c_master_dev_handle_t s_dev = NULL;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    const uint8_t b[2] = {reg, val};
    return i2c_master_transmit(s_dev, b, sizeof(b), pdMS_TO_TICKS(50));
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, pdMS_TO_TICKS(50));
}

esp_err_t bsp_imu_init(void)
{
    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BSP_IMU_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bsp_i2c_sys, &cfg, &s_dev),
                        TAG, "устройство IMU");

    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(reg_read(REG_WHOAMI, &id, 1), TAG, "чтение WHO_AM_I");
    ESP_RETURN_ON_FALSE(id == WHOAMI_VALUE, ESP_ERR_NOT_FOUND, TAG,
                        "QMI8658 не отвечает: WHO_AM_I = 0x%02x, ожидалось 0x%02x",
                        id, WHOAMI_VALUE);

    uint8_t rev = 0;
    reg_read(REG_REVISION, &rev, 1);

    /* CTRL1: автоинкремент адреса при чтении — иначе блок данных не вычитать
     * одной транзакцией. */
    ESP_RETURN_ON_ERROR(reg_write(REG_CTRL1, 0x60), TAG, "CTRL1");
    /* CTRL2: акселерометр +-4g, 500 Гц. */
    ESP_RETURN_ON_ERROR(reg_write(REG_CTRL2, 0x14), TAG, "CTRL2");
    /* CTRL3: гироскоп +-512 dps, 500 Гц. */
    ESP_RETURN_ON_ERROR(reg_write(REG_CTRL3, 0x54), TAG, "CTRL3");
    /* CTRL5: фильтры нижних частот на обоих датчиках — гасим дребезг,
     * не трогая полезную полосу движений руки. */
    ESP_RETURN_ON_ERROR(reg_write(REG_CTRL5, 0x11), TAG, "CTRL5");
    /* CTRL7: включаем акселерометр и гироскоп. */
    ESP_RETURN_ON_ERROR(reg_write(REG_CTRL7, 0x03), TAG, "CTRL7");

    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "QMI8658 поднят, ревизия 0x%02x", rev);
    return ESP_OK;
}

esp_err_t bsp_imu_read(bsp_imu_data_t *out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out");
    ESP_RETURN_ON_FALSE(s_dev, ESP_ERR_INVALID_STATE, TAG, "IMU не поднят");

    /* Температура и оба датчика лежат подряд, поэтому берём одной транзакцией:
     * на общей шине лишние обращения дороже, чем пара байт. */
    uint8_t b[14] = {0};
    ESP_RETURN_ON_ERROR(reg_read(REG_TEMP_L, b, sizeof(b)), TAG, "чтение данных");

    const int16_t temp = (int16_t)((b[1] << 8) | b[0]);
    const int16_t ax   = (int16_t)((b[3] << 8) | b[2]);
    const int16_t ay   = (int16_t)((b[5] << 8) | b[4]);
    const int16_t az   = (int16_t)((b[7] << 8) | b[6]);
    const int16_t gx   = (int16_t)((b[9] << 8) | b[8]);
    const int16_t gy   = (int16_t)((b[11] << 8) | b[10]);
    const int16_t gz   = (int16_t)((b[13] << 8) | b[12]);

    const float acc_scale = ACC_RANGE_G / 32768.0f;
    const float gyr_scale = GYR_RANGE_DPS / 32768.0f;

    out->ax = ax * acc_scale;
    out->ay = ay * acc_scale;
    out->az = az * acc_scale;
    out->gx = gx * gyr_scale;
    out->gy = gy * gyr_scale;
    out->gz = gz * gyr_scale;
    out->temp_c = temp / 256.0f;
    return ESP_OK;
}
