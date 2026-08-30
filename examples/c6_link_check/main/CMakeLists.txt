idf_component_register(
    SRCS "c6_link_check_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_netif esp_event nvs_flash
)
