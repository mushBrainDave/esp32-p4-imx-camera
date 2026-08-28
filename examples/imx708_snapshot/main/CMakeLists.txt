idf_component_register(
    SRCS "imx708_snapshot_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx fatfs esp_driver_sdmmc
)
