idf_component_register(SRCS "iv_link.c" "iv_link_stats.c"
                       INCLUDE_DIRS "include"
                       REQUIRES iv_protocol esp_wifi esp_netif esp_event esp_timer nvs_flash)
