/* Контроллер питания M5PM1 смычка.
 *
 * Даташит на M5PM1 не опубликован, поэтому регистры взяты не из головы, а из
 * драйвера M5Unified (src/utility/power/M5PM1_Class.cpp): там же видно, что
 * напряжения он отдаёт готовыми милливольтами двумя байтами от младшего —
 * пересчитывать отсчёты АЦП не нужно.
 *
 * Инициализация повторяет драйвер и делает ровно две осмысленные вещи:
 * выключает засыпание шины I2C в простое (иначе первое обращение после паузы
 * теряется) и выключает свой сторожевой таймер (иначе он вправе выключить
 * смычок посреди игры). */
#include "bsp_bow.h"
#include "bsp_bow_internal.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp.bow.pwr";

#define PM_ADDR      0x6E
#define PM_FREQ_HZ   100000  /* как в драйвере M5: контроллер не быстрый */
#define PM_TIMEOUT_MS 50

#define PM_REG_DEVICE_ID 0x00
#define PM_REG_PWR_SRC   0x04  /* бит0 — 5VIN есть, то есть питание от USB */
#define PM_REG_I2C_CFG   0x09
#define PM_REG_WDT_CNT   0x0A
#define PM_REG_VBAT_L    0x22  /* два байта, младший первым, милливольты */

#define PM_PWR_SRC_VIN   (1u << 0)

static i2c_master_dev_handle_t s_pm = NULL;

static esp_err_t pm_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_pm, &reg, 1, buf, len,
                                       PM_TIMEOUT_MS);
}

static esp_err_t pm_write8(uint8_t reg, uint8_t value)
{
    const uint8_t cmd[2] = { reg, value };
    return i2c_master_transmit(s_pm, cmd, sizeof(cmd), PM_TIMEOUT_MS);
}

esp_err_t bsp_bow_power_init(void)
{
    ESP_RETURN_ON_ERROR(bsp_bow_board_init(), TAG, "шина I2C");

    const i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = PM_ADDR,
        .scl_speed_hz    = PM_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bsp_bow_i2c, &cfg, &s_pm),
                        TAG, "устройство M5PM1");

    uint8_t id[4] = {0};
    if (pm_read(PM_REG_DEVICE_ID, id, sizeof(id)) != ESP_OK) {
        /* Не фатально: без контроллера смычок играет, просто не знает заряда.
         * Ронять из-за этого прошивку было бы хуже, чем молчать про батарею. */
        i2c_master_bus_rm_device(s_pm);
        s_pm = NULL;
        ESP_LOGW(TAG, "M5PM1 не отвечает: заряд батареи показан не будет");
        return ESP_OK;
    }

    /* Засыпание шины в простое — не наш случай: к контроллеру обращаются раз
     * в секунду, и каждое обращение после паузы уходило бы в никуда. */
    ESP_RETURN_ON_ERROR(pm_write8(PM_REG_I2C_CFG, 0x00), TAG, "сон шины");
    /* Сторожевой таймер контроллера выключаем: он ждёт, что прошивка будет
     * его гладить, а не гладит его никто — сработавший таймер выключил бы
     * смычок посреди игры. */
    ESP_RETURN_ON_ERROR(pm_write8(PM_REG_WDT_CNT, 0x00), TAG, "сторожевой таймер");

    ESP_LOGI(TAG, "M5PM1 поднят, id %02x%02x%02x%02x", id[0], id[1], id[2], id[3]);
    return ESP_OK;
}

bool bsp_bow_power_read(uint16_t *batt_mv, bool *usb)
{
    if (s_pm == NULL) {
        return false;
    }

    uint8_t v[2] = {0};
    if (pm_read(PM_REG_VBAT_L, v, sizeof(v)) != ESP_OK) {
        return false;
    }
    if (batt_mv) {
        *batt_mv = (uint16_t)(((uint16_t)v[1] << 8) | v[0]);
    }

    if (usb) {
        uint8_t src = 0;
        /* Про USB не знать не страшно: тогда считаем, что его нет, и
         * показываем обычный заряд. */
        *usb = (pm_read(PM_REG_PWR_SRC, &src, 1) == ESP_OK)
               && (src & PM_PWR_SRC_VIN);
    }
    return true;
}
