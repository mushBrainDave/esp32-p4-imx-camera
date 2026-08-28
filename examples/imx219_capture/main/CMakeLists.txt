idf_component_register(
    SRCS "imx219_capture_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx
)
