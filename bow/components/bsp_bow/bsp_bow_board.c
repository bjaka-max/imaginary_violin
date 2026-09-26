#include "bsp_bow.h"
#include "bsp_bow_internal.h"

#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "bsp.bow";

i2c_master_bus_handle_t bsp_bow_i2c = NULL;

esp_err_t bsp_bow_board_init(void)
{
    if (bsp_bow_i2c) {
        return ESP_OK;
    }
    const i2c_master_bus_config_t bus_cfg = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = BSP_BOW_I2C_PORT,
        .sda_io_num        = BSP_BOW_I2C_SDA,
        .scl_io_num        = BSP_BOW_I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bsp_bow_i2c), TAG, "шина I2C");
    ESP_LOGI(TAG, "внутренняя шина I2C поднята: SDA %d, SCL %d",
             BSP_BOW_I2C_SDA, BSP_BOW_I2C_SCL);
    return ESP_OK;
}
