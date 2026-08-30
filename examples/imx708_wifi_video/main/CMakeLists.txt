idf_component_register(
    SRCS "imx708_wifi_video_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx imx_wifi imx_fmp4 esp_http_server esp_timer
)
