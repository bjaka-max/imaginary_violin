idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES iv_protocol iv_dsp iv_link iv_motion iv_power bsp_bow
                                esp_timer nvs_flash)
