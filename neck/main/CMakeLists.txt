idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES iv_protocol iv_dsp iv_link iv_params iv_motion iv_control
                                iv_menu iv_power iv_settings iv_synth iv_ui bsp_neck esp_psram)
