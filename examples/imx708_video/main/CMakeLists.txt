idf_component_register(
    SRCS "imx708_video_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx imx_serial_img esp_driver_uart esp_timer
)
