/* Внутренние связи между файлами BSP. Наружу не выставляется. */
#ifndef BSP_INTERNAL_H
#define BSP_INTERNAL_H

#include "driver/i2c_master.h"
#include "esp_io_expander.h"
#include "esp_check.h"

extern i2c_master_bus_handle_t  bsp_i2c_sys;
extern i2c_master_bus_handle_t  bsp_i2c_touch;
extern esp_io_expander_handle_t bsp_expander;

void bsp_lcd_reset(void);
esp_err_t bsp_backlight_pwm_init(void);

#endif /* BSP_INTERNAL_H */
