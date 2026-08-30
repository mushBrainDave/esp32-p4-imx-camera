idf_component_register(
    SRCS "imx708_wifi_snapshot_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx imx_wifi esp_driver_jpeg esp_http_server esp_timer
)
