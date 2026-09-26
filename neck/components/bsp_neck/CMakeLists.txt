idf_component_register(
    SRCS "bsp_board.c" "bsp_button.c" "bsp_display.c" "bsp_touch.c" "bsp_imu.c" "bsp_audio.c" "bsp_battery.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_ledc esp_lcd esp_driver_i2c esp_driver_i2s esp_driver_spi esp_timer esp_adc
             esp_io_expander_tca9554 esp_lcd_axs15231b esp_codec_dev iv_touch)
