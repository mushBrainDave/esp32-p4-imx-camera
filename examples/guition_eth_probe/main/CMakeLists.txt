idf_component_register(SRCS "ethernet_example_main.c"
                       PRIV_REQUIRES esp_netif esp_eth esp_http_server esp_timer
                       INCLUDE_DIRS ".")
