idf_component_register(SRCS "bsp_bow_board.c" "bsp_bow_imu.c" "bsp_bow_power.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c bmi270)
